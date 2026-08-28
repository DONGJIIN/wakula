"""把障碍识别结果转换为 Nav2 可使用的“入口—对正—交接”引导。

Nav2 擅长在自由空间中到达一个位姿，但不能完成抬腿、攀爬、钻杆或跨坑。本节点因此
不把目标放到实体障碍后方，也不调用 NavigateToPose；它只发布位于障碍前方自由空间
内的相对入口位姿。当距离和角度都满足条件时发布 READY，未来运动控制器可据此申请
接管。越障完成后，应由更上层任务管理器恢复原 Nav2 目标。

所有位置都在输入消息的 frame（通常为 base_link）中，x 向前、y 向左、yaw 逆时针
为正。纯函数 ``compute_guidance`` 供单元测试和 rosbag 离线评估复用。

入口距离、对正容差、低通和 READY 迟滞统一配置在
``config/terrain_navigation.yaml``。这些参数必须和 Nav2 footprint/inflation、任务层停滞
交接范围以及真实越障控制器的接近能力联合标定，不能只为了尽快 READY 而单独放宽。
"""

from dataclasses import dataclass, replace
from math import atan2, cos, isfinite, pi, sin, tan
import signal

from geometry_msgs.msg import PoseStamped
from quadruped_interfaces.msg import NavigationSafety, TraversalGuidance

from quadruped_planning.parameter_validation import validate_guidance_parameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


TRAVERSAL_TYPES = {
    NavigationSafety.OBSTACLE_STEP,
    NavigationSafety.OBSTACLE_PIT,
    NavigationSafety.OBSTACLE_WALL,
    NavigationSafety.OBSTACLE_BAR,
}

PHASE_NAMES = {
    TraversalGuidance.PHASE_INVALID: "INVALID",
    TraversalGuidance.PHASE_CLEAR: "CLEAR",
    TraversalGuidance.PHASE_APPROACH: "APPROACH",
    TraversalGuidance.PHASE_ALIGN: "ALIGN",
    TraversalGuidance.PHASE_READY: "READY",
}


def surface_axis_heading(slope_pitch: float, slope_roll: float) -> float | None:
    """Return the nearest yaw correction that aligns the body with a surface.

    The two input angles are the forward and lateral components of the fitted
    plane in ``base_link``.  Their gradient gives the uphill direction.  Since
    uphill and downhill share the same traversable axis, the result is folded
    into ``[-pi/2, pi/2]``.  Flat/noisy or wall-like fits return ``None``.

    This is deliberately geometry-only.  It fixes diagonal approaches to a
    ramp or stair without reading a Gazebo entity name or a competition-map
    coordinate, so the same logic can be calibrated on the future real robot.
    """
    pitch = float(slope_pitch)
    roll = float(slope_roll)
    if not isfinite(pitch) or not isfinite(roll):
        return None
    gradient = (tan(pitch) ** 2 + tan(roll) ** 2) ** 0.5
    # Ignore less than about 6 degrees as plane-fit noise and more than about
    # 25 degrees as a mixed vertical face rather than an approach surface.
    if gradient < tan(0.105) or gradient > tan(0.436):
        return None
    heading = atan2(tan(roll), tan(pitch))
    while heading > pi * 0.5:
        heading -= pi
    while heading < -pi * 0.5:
        heading += pi
    return heading


@dataclass(frozen=True)
class GuidanceDecision:
    """与 ROS 消息一一对应的可测试决策结果。"""

    phase: int = TraversalGuidance.PHASE_INVALID
    obstacle_type: int = TraversalGuidance.OBSTACLE_UNKNOWN
    perception_valid: bool = False
    traversal_required: bool = False
    ready_for_handoff: bool = False
    confidence: float = 0.0
    distance: float = 0.0
    lateral_offset: float = 0.0
    heading_error: float = 0.0
    approach_x: float = 0.0
    approach_y: float = 0.0
    approach_yaw: float = 0.0
    speed_limit: float = 0.0


class GuidanceStabilizer:
    """对入口目标做指数平滑，并为 READY 增加连续帧确认和退出迟滞。

    点云连通区域的边缘会随视角和缺点轻微变化。若直接按单帧距离/角度切换，停在
    1.20 m 边界附近时会在 ALIGN 与 READY 间闪烁。这里仅平滑同一障碍的连续量；
    INVALID、障碍类别变化和无越障目标会立即重置，绝不把上一处障碍的入口带到下一处。
    """

    def __init__(
        self,
        *,
        handoff_distance: float,
        alignment_tolerance: float,
        approach_start_distance: float,
        target_smoothing_alpha: float,
        distance_hysteresis: float,
        angle_hysteresis: float,
        ready_confirmation_frames: int,
        type_confirmation_frames: int,
        approach_speed_limit: float,
        alignment_speed_limit: float,
    ):
        self.handoff_distance = max(0.01, float(handoff_distance))
        self.alignment_tolerance = max(0.001, float(alignment_tolerance))
        self.approach_start_distance = max(
            self.handoff_distance, float(approach_start_distance)
        )
        self.alpha = max(0.01, min(1.0, float(target_smoothing_alpha)))
        # 距离和角度的量纲不同，必须分开配置；把一个“0.05”同时解释成 5 cm 和
        # 0.05 rad 会让后续真机标定难以判断究竟该调哪一个边界。
        self.distance_hysteresis = max(0.0, float(distance_hysteresis))
        self.angle_hysteresis = max(0.0, float(angle_hysteresis))
        self.ready_frames = max(1, int(ready_confirmation_frames))
        self.type_frames = max(1, int(type_confirmation_frames))
        self.approach_speed = max(0.0, min(1.0, approach_speed_limit))
        self.alignment_speed = max(0.0, min(1.0, alignment_speed_limit))
        self.current = GuidanceDecision()
        self.ready_count = 0
        self.pending_type = None
        self.pending_type_count = 0

    def reset(self, decision: GuidanceDecision) -> GuidanceDecision:
        """清除历史并以当前明确状态重新起步。"""
        self.current = decision
        self.ready_count = 0
        self.pending_type = None
        self.pending_type_count = 0
        return decision

    def _smooth(self, previous: float, current: float) -> float:
        """执行有界的一阶低通；alpha=1 保留原始值。"""
        return previous + self.alpha * (current - previous)

    def update(self, candidate: GuidanceDecision) -> GuidanceDecision:
        """返回时序稳定结果；无效输入立即撤销 READY。"""
        if not candidate.perception_valid or not candidate.traversal_required:
            return self.reset(candidate)
        same_target = (
            self.current.perception_valid
            and self.current.traversal_required
            # 同一入口的点云可能在台阶、坑沿、墙面之间抖动。距离/横偏连续时先保持
            # 当前类别，只有新类别连续出现才切换，避免 READY 计数被每帧清零。
            and abs(self.current.distance - candidate.distance) <= 0.65
            and abs(self.current.lateral_offset - candidate.lateral_offset) <= 0.55
        )
        if not same_target:
            # 新障碍第一帧可立即 APPROACH/ALIGN，但 READY 必须重新累计，避免单帧误交接。
            self.current = candidate
            self.ready_count = 0
            if candidate.phase == TraversalGuidance.PHASE_READY:
                self.ready_count = 1
                if self.ready_frames > 1:
                    self.current = replace(
                        candidate,
                        phase=TraversalGuidance.PHASE_ALIGN,
                        ready_for_handoff=False,
                        speed_limit=self.alignment_speed,
                    )
            return self.current

        if candidate.obstacle_type != self.current.obstacle_type:
            if self.pending_type == candidate.obstacle_type:
                self.pending_type_count += 1
            else:
                self.pending_type = candidate.obstacle_type
                self.pending_type_count = 1
            if self.pending_type_count < self.type_frames:
                candidate = replace(candidate, obstacle_type=self.current.obstacle_type)
            else:
                self.pending_type = None
                self.pending_type_count = 0
        else:
            self.pending_type = None
            self.pending_type_count = 0

        smoothed = replace(
            candidate,
            confidence=self._smooth(
                self.current.confidence, candidate.confidence
            ),
            distance=self._smooth(self.current.distance, candidate.distance),
            lateral_offset=self._smooth(
                self.current.lateral_offset, candidate.lateral_offset
            ),
            heading_error=self._smooth(
                self.current.heading_error, candidate.heading_error
            ),
            approach_x=self._smooth(
                self.current.approach_x, candidate.approach_x
            ),
            approach_y=self._smooth(
                self.current.approach_y, candidate.approach_y
            ),
            approach_yaw=self._smooth(
                self.current.approach_yaw, candidate.approach_yaw
            ),
        )
        # 已进入某阶段后使用稍宽退出边界，吸收厘米级距离噪声和小角度抖动。
        ready_distance = self.handoff_distance
        ready_angle = self.alignment_tolerance
        if self.current.phase == TraversalGuidance.PHASE_READY:
            ready_distance += self.distance_hysteresis
            ready_angle += self.angle_hysteresis
        if (
            smoothed.distance <= ready_distance
            and abs(smoothed.heading_error) <= ready_angle
        ):
            self.ready_count += 1
            if (
                self.current.phase == TraversalGuidance.PHASE_READY
                or self.ready_count >= self.ready_frames
            ):
                phase = TraversalGuidance.PHASE_READY
                speed = 0.0
            else:
                phase = TraversalGuidance.PHASE_ALIGN
                speed = self.alignment_speed
        else:
            self.ready_count = 0
            approach_boundary = self.approach_start_distance
            if self.current.phase == TraversalGuidance.PHASE_APPROACH:
                approach_boundary -= self.distance_hysteresis
            phase = (
                TraversalGuidance.PHASE_APPROACH
                if smoothed.distance > approach_boundary
                else TraversalGuidance.PHASE_ALIGN
            )
            speed = (
                self.approach_speed
                if phase == TraversalGuidance.PHASE_APPROACH
                else self.alignment_speed
            )
        self.current = replace(
            smoothed,
            phase=phase,
            ready_for_handoff=phase == TraversalGuidance.PHASE_READY,
            speed_limit=speed,
        )
        return self.current


def compute_guidance(
    safety: NavigationSafety,
    *,
    approach_start_distance: float,
    handoff_distance: float,
    alignment_tolerance: float,
    max_lateral_target: float,
    approach_speed_limit: float,
    alignment_speed_limit: float,
    minimum_slope_for_handoff: float,
) -> GuidanceDecision:
    """生成保守入口目标；任何非法字段都返回 INVALID/零速度。

    STEP/PIT/WALL/BAR 需要越障控制器。POLE 只有在量测高度达到规则绕杆立柱的
    0.45 m 下限时才进入比赛绕杆流程；约 0.32 m 的限高杆支柱仍是普通导航线索，避免
    只看到一侧支柱时误启动绕杆。CLEAR 仅在点云确认坡度超过阈值时按坡面候选处理。
    这里的“需要”只是任务 Action 候选，不能直接解释为关节或足端命令。
    """
    values = (
        safety.confidence,
        safety.distance,
        safety.lateral_offset,
        safety.slope_pitch,
        safety.structure_heading,
        safety.structure_heading_confidence,
        approach_start_distance,
        handoff_distance,
        alignment_tolerance,
        max_lateral_target,
        approach_speed_limit,
        alignment_speed_limit,
        minimum_slope_for_handoff,
    )
    obstacle_type = int(safety.obstacle_type)
    if (
        not safety.perception_valid
        or obstacle_type < NavigationSafety.OBSTACLE_CLEAR
        or obstacle_type > NavigationSafety.OBSTACLE_POLE
        or not all(isfinite(float(value)) for value in values)
        or float(safety.confidence) < 0.0
        or float(safety.confidence) > 1.0
        or float(safety.distance) < 0.0
        or not 0.0 <= float(safety.structure_heading_confidence) <= 1.0
        or handoff_distance <= 0.0
        or approach_start_distance < handoff_distance
    ):
        return GuidanceDecision()

    slope_candidate = (
        obstacle_type == NavigationSafety.OBSTACLE_CLEAR
        and abs(float(safety.slope_pitch))
        >= max(0.0, minimum_slope_for_handoff)
    )
    pole_course_candidate = (
        obstacle_type == NavigationSafety.OBSTACLE_POLE
        and isfinite(float(safety.obstacle_height))
        and float(safety.obstacle_height) >= 0.45
    )
    traversal_required = (
        obstacle_type in TRAVERSAL_TYPES
        or slope_candidate
        or pole_course_candidate
    )
    if not traversal_required:
        return GuidanceDecision(
            phase=TraversalGuidance.PHASE_CLEAR,
            obstacle_type=obstacle_type,
            perception_valid=True,
            confidence=float(safety.confidence),
            distance=float(safety.distance),
            lateral_offset=float(safety.lateral_offset),
            speed_limit=max(0.0, min(1.0, float(safety.speed_limit))),
        )

    distance = float(safety.distance)
    lateral = float(safety.lateral_offset)
    # 防止单帧边缘缺点把入口目标推到相机/雷达视场之外；原始偏移仍完整写入消息，便于
    # 下游判断证据是否可靠，只有建议目标 y 被夹紧。
    lateral_target = max(
        -abs(max_lateral_target), min(abs(max_lateral_target), lateral)
    )
    centre_heading = atan2(lateral_target, max(distance, 0.05))
    # For a sloped STEP/CLEAR surface the plane axis is stronger orientation
    # evidence than the centre of its visible crop.  Keep a small centring
    # term so an axis-aligned robot also moves into the usable obstacle width.
    axis_heading = surface_axis_heading(
        float(safety.slope_pitch), float(safety.slope_roll)
    )
    # Flat stairs, bridge decks and walls have no useful plane gradient.  For
    # those cases the point-cloud front-edge normal is the stronger evidence.
    # A conservative confidence gate prevents sparse/isotropic clusters from
    # overriding the well-tested centre-of-obstacle fallback.
    if axis_heading is None and float(safety.structure_heading_confidence) >= 0.45:
        axis_heading = float(safety.structure_heading)
    heading = (
        centre_heading
        if axis_heading is None
        else axis_heading + 0.25 * centre_heading
    )
    approach_x = max(0.0, distance - handoff_distance)
    approach_y = lateral_target

    if distance > approach_start_distance:
        phase = TraversalGuidance.PHASE_APPROACH
        speed = approach_speed_limit
    elif distance > handoff_distance or abs(heading) > alignment_tolerance:
        phase = TraversalGuidance.PHASE_ALIGN
        speed = alignment_speed_limit
    else:
        phase = TraversalGuidance.PHASE_READY
        speed = 0.0

    return GuidanceDecision(
        phase=phase,
        obstacle_type=obstacle_type,
        perception_valid=True,
        traversal_required=True,
        ready_for_handoff=phase == TraversalGuidance.PHASE_READY,
        confidence=float(safety.confidence),
        distance=distance,
        lateral_offset=lateral,
        heading_error=heading,
        approach_x=approach_x,
        approach_y=approach_y,
        approach_yaw=heading,
        speed_limit=max(0.0, min(1.0, float(speed))),
    )


class TraversalGuidanceNode(Node):
    """发布越障入口建议，不取得 Nav2 或运动控制权。"""

    def __init__(self, **node_kwargs):
        super().__init__("traversal_guidance", **node_kwargs)
        defaults = (
            ("input_timeout", 0.8),
            ("approach_start_distance", 1.8),
            ("handoff_distance", 1.20),
            ("alignment_tolerance", 0.10),
            ("max_lateral_target", 0.45),
            ("approach_speed_limit", 0.25),
            ("alignment_speed_limit", 0.12),
            ("minimum_slope_for_handoff", 0.12),
            ("target_smoothing_alpha", 0.35),
            ("distance_hysteresis", 0.05),
            ("angle_hysteresis", 0.035),
            ("ready_confirmation_frames", 3),
            ("type_confirmation_frames", 3),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)
        self.parameters = {
            name: float(self.get_parameter(name).value) for name, _ in defaults
        }
        validate_guidance_parameters(self.parameters)
        self.stabilizer = GuidanceStabilizer(
            handoff_distance=self.parameters["handoff_distance"],
            alignment_tolerance=self.parameters["alignment_tolerance"],
            approach_start_distance=self.parameters["approach_start_distance"],
            target_smoothing_alpha=self.parameters["target_smoothing_alpha"],
            distance_hysteresis=self.parameters["distance_hysteresis"],
            angle_hysteresis=self.parameters["angle_hysteresis"],
            ready_confirmation_frames=int(
                self.parameters["ready_confirmation_frames"]
            ),
            type_confirmation_frames=int(
                self.parameters["type_confirmation_frames"]
            ),
            approach_speed_limit=self.parameters["approach_speed_limit"],
            alignment_speed_limit=self.parameters["alignment_speed_limit"],
        )
        self.guidance_pub = self.create_publisher(
            TraversalGuidance, "/traversal/guidance", 10
        )
        self.phase_pub = self.create_publisher(String, "/traversal/phase", 10)
        self.approach_pose_pub = self.create_publisher(
            PoseStamped, "/traversal/approach_pose", 10
        )
        self.create_subscription(
            NavigationSafety,
            "/terrain/navigation_safety",
            self.safety_callback,
            10,
        )
        self.last_receive_time = None
        self.last_header = None
        self.last_phase_signature = None
        self.timeout_published = False
        self.create_timer(0.1, self.timeout_callback)
        self.get_logger().info(
            "Traversal guidance ready: Nav2 approach/alignment only; "
            "no gait command"
        )

    def safety_callback(self, safety: NavigationSafety) -> None:
        """把一帧原子安全状态转换为同时间戳的入口建议。"""
        self.last_receive_time = self.get_clock().now()
        self.last_header = safety.header
        self.timeout_published = False
        decision = compute_guidance(
            safety,
            **{
                key: value
                for key, value in self.parameters.items()
                if key != "input_timeout"
                and key != "target_smoothing_alpha"
                and key != "distance_hysteresis"
                and key != "angle_hysteresis"
                and key != "ready_confirmation_frames"
                and key != "type_confirmation_frames"
            },
        )
        decision = self.stabilizer.update(decision)
        self.publish_decision(decision, safety.header)

    def timeout_callback(self) -> None:
        """输入断流后发布一次 INVALID，避免 READY 状态永久粘住。"""
        if self.last_receive_time is None:
            return
        age = (
            self.get_clock().now() - self.last_receive_time
        ).nanoseconds * 1e-9
        if (
            age > max(0.1, self.parameters["input_timeout"])
            and not self.timeout_published
        ):
            header = self.last_header
            header.stamp = self.get_clock().now().to_msg()
            # timeout 既要对外撤销 READY，也必须清空内部平滑历史。否则输入恢复后，
            # 第一帧可能与断流前的旧障碍做低通，造成虚假的入口位置。
            invalid = self.stabilizer.reset(GuidanceDecision())
            self.publish_decision(invalid, header)
            self.timeout_published = True

    def publish_decision(self, decision: GuidanceDecision, header) -> None:
        """原子发布强类型状态、可读阶段及 RViz 可视化入口位姿。"""
        msg = TraversalGuidance()
        msg.header = header
        for field in GuidanceDecision.__dataclass_fields__:
            setattr(msg, field, getattr(decision, field))
        self.guidance_pub.publish(msg)
        phase_name = PHASE_NAMES.get(decision.phase, "INVALID")
        self.phase_pub.publish(String(data=phase_name))

        # 独立 PoseStamped 没有有效位字段，因此仅在确有越障目标时发布。消费者仍应把它
        # 与同时间戳 guidance 配对，不能使用 RViz 中残留的旧箭头控制机器人。
        if decision.perception_valid and decision.traversal_required:
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = decision.approach_x
            pose.pose.position.y = decision.approach_y
            pose.pose.orientation.z = sin(decision.approach_yaw * 0.5)
            pose.pose.orientation.w = cos(decision.approach_yaw * 0.5)
            self.approach_pose_pub.publish(pose)

        signature = (
            decision.phase,
            decision.obstacle_type,
            decision.ready_for_handoff,
        )
        if signature != self.last_phase_signature:
            self.last_phase_signature = signature
            self.get_logger().info(
                f"Traversal guidance: {phase_name}, "
                f"obstacle={decision.obstacle_type}, "
                f"distance={decision.distance:.2f} m, "
                f"lateral={decision.lateral_offset:.2f} m, handoff="
                f"{'READY' if decision.ready_for_handoff else 'not ready'}"
            )


def main(args=None):
    """ROS 2 可执行入口，兼容终端 Ctrl-C 和 launch 关闭。"""
    rclpy.init(args=args)
    node = TraversalGuidanceNode()
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

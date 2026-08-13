"""把障碍识别结果转换为 Nav2 可使用的“入口—对正—交接”引导。

Nav2 擅长在自由空间中到达一个位姿，但不能完成抬腿、攀爬、钻杆或跨坑。本节点因此
不把目标放到实体障碍后方，也不调用 NavigateToPose；它只发布位于障碍前方自由空间
内的相对入口位姿。当距离和角度都满足条件时发布 READY，未来运动控制器可据此申请
接管。越障完成后，应由更上层任务管理器恢复原 Nav2 目标。

所有位置都在输入消息的 frame（通常为 base_link）中，x 向前、y 向左、yaw 逆时针
为正。纯函数 ``compute_guidance`` 供单元测试和 rosbag 离线评估复用。
"""

from dataclasses import dataclass
from math import atan2, cos, isfinite, sin
import signal

from geometry_msgs.msg import PoseStamped
from quadruped_interfaces.msg import NavigationSafety, TraversalGuidance
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

    STEP/PIT/WALL/BAR 需要越障控制器；POLE 是需要 Nav2 从旁边绕行的实体，不触发
    接管。CLEAR 仅在点云确认坡度超过阈值时按坡面越障候选处理。这里的“需要”只是
    候选接口，最终仍应由比赛路线/预期障碍类型进行授权，不能直接解释为腿部命令。
    """
    values = (
        safety.confidence,
        safety.distance,
        safety.lateral_offset,
        safety.slope_pitch,
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
        or handoff_distance <= 0.0
        or approach_start_distance < handoff_distance
    ):
        return GuidanceDecision()

    slope_candidate = (
        obstacle_type == NavigationSafety.OBSTACLE_CLEAR
        and abs(float(safety.slope_pitch))
        >= max(0.0, minimum_slope_for_handoff)
    )
    traversal_required = obstacle_type in TRAVERSAL_TYPES or slope_candidate
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
    heading = atan2(lateral_target, max(distance, 0.05))
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

    def __init__(self):
        super().__init__("traversal_guidance")
        defaults = (
            ("input_timeout", 0.8),
            ("approach_start_distance", 1.5),
            ("handoff_distance", 0.75),
            ("alignment_tolerance", 0.10),
            ("max_lateral_target", 0.45),
            ("approach_speed_limit", 0.25),
            ("alignment_speed_limit", 0.12),
            ("minimum_slope_for_handoff", 0.12),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)
        self.parameters = {
            name: float(self.get_parameter(name).value) for name, _ in defaults
        }
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
            },
        )
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
            self.publish_decision(GuidanceDecision(), header)
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

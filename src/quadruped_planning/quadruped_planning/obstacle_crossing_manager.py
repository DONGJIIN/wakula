"""将点云几何和 OpenCV 辅助证据融合为保守越障建议。

职责边界：本节点只发布模式、动作意图和速度缩放，不生成腿部轨迹，也不宣称越障已经
完成。几何点云是动作等级的主证据；单目 OpenCV 没有可靠尺度，只能在 WALK 时请求
减速或停车复核。项目当前没有腿部越障执行器，因此 STEP/CLIMB 只作为感知分类发布，
速度门会保持停车；真机控制系统完成后再单独接入执行层。
"""

from math import isfinite
from typing import Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


Decision = Tuple[str, str, float]
MODE_SEVERITY = {"WALK": 0, "STEP": 1, "CLIMB": 2, "STOP": 3}

# /terrain/features 固定字段下标。这里重复声明通信合同，避免 planning 反向依赖
# perception Python 包；若将来改为自定义 msg，应删除这些兼容常量。
TERRAIN_OBSTACLE_HEIGHT = 2
TERRAIN_VALID_POINTS = 3
TERRAIN_GROUND_SLOPE = 4
TERRAIN_ROUGHNESS = 5
TERRAIN_FRONTAL_HEIGHT = 6
TERRAIN_PIT_DEPTH = 9
TERRAIN_OBSTACLE_TYPE = 11

# 与 TerrainFeatures.msg 一致；保留数组输入是为了兼容既有 rosbag。
GEOMETRY_CLEAR, GEOMETRY_STEP, GEOMETRY_PIT = 1, 2, 3
GEOMETRY_WALL, GEOMETRY_BAR, GEOMETRY_POLE = 4, 5, 6


class ConservativeDecisionFilter:
    """STOP 立即生效，动作升级/风险降低分别需要连续证据。

    例如 WALK→STOP 不允许因防抖而延迟；STOP→WALK 则必须持续观察到安全证据。
    这是一种非对称迟滞，解决阈值附近 WALK/STEP 来回跳变的问题。
    """

    def __init__(
        self, clear_frames: int, initial: Decision, hazard_frames: int = 2
    ):
        self.clear_frames = max(1, int(clear_frames))
        self.hazard_frames = max(1, int(hazard_frames))
        self.current = initial
        self.pending_mode = None
        self.pending_count = 0

    def update(self, candidate: Decision) -> Decision:
        """输入当前帧候选结果，返回经过安全迟滞后的稳定结果。"""
        current_level = MODE_SEVERITY.get(self.current[0], MODE_SEVERITY["STOP"])
        candidate_level = MODE_SEVERITY.get(candidate[0], MODE_SEVERITY["STOP"])
        if candidate_level == current_level:
            self.current = candidate
            self.pending_mode = None
            self.pending_count = 0
            return self.current
        if candidate_level > current_level:
            # 紧急 STOP 不允许防抖延迟；STEP/CLIMB 则要求短暂连续几何证据，
            # 防止深度飞点在单帧内触发错误地形分类。
            if candidate[0] == "STOP" or self.hazard_frames == 1:
                self.current = candidate
                self.pending_mode = None
                self.pending_count = 0
                return self.current
            if candidate[0] != self.pending_mode:
                self.pending_mode = candidate[0]
                self.pending_count = 1
            else:
                self.pending_count += 1
            if self.pending_count >= self.hazard_frames:
                self.current = candidate
                self.pending_mode = None
                self.pending_count = 0
            return self.current
        if candidate[0] != self.pending_mode:
            self.pending_mode = candidate[0]
            self.pending_count = 1
        else:
            self.pending_count += 1
        if self.pending_count >= self.clear_frames:
            self.current = candidate
            self.pending_mode = None
            self.pending_count = 0
        return self.current


def validate_height_thresholds(
    step: float, climb: float, stop: float
) -> Tuple[float, float, float]:
    """保证高度阈值有限、非负且严格递增，否则恢复保守默认值（米）。"""
    values = (step, climb, stop)
    if all(isfinite(value) for value in values) and 0.0 <= step < climb < stop:
        return values
    return 0.08, 0.18, 0.32


def select_terrain_decision(
    obstacle_height: float,
    points: float,
    slope: float,
    roughness: float,
    min_points: int,
    step_threshold: float,
    climb_threshold: float,
    stop_threshold: float,
    max_slope: float,
    max_roughness: float,
) -> Decision:
    """依据一帧几何特征返回 ``(模式, 动作, 速度比例)``。

    判定顺序从最高风险向下，避免 0.35 m 高墙先命中 STEP 阈值。坡度使用绝对值，
    因为过陡上坡和下坡都可能超过四足平台稳定能力。
    """
    values = (
        obstacle_height,
        points,
        slope,
        roughness,
        step_threshold,
        climb_threshold,
        stop_threshold,
        max_slope,
        max_roughness,
    )
    # 无效或稀疏点云必须停机，不能把“没看见”当作“可以通过”。
    invalid_limits = (
        not 0.0 <= step_threshold < climb_threshold < stop_threshold
        or max_slope <= 0.0
        or max_roughness <= 0.0
        or min_points < 1
    )
    if (
        not all(isfinite(value) for value in values)
        or invalid_limits
        or points < min_points
    ):
        return "STOP", "WAIT_FOR_TERRAIN", 0.0
    # 从最危险条件向下判断，确保高墙不会被较低的 STEP 阈值截获。
    absolute_slope = abs(slope)
    if obstacle_height >= stop_threshold or absolute_slope >= max_slope * 1.5:
        return "STOP", "REPLAN_OR_REQUEST_FOOTSTEPS", 0.0
    if obstacle_height >= climb_threshold or absolute_slope >= max_slope:
        return "CLIMB", "STOP_FOR_CLIMB_OR_REPLAN", 0.0
    if obstacle_height >= step_threshold or roughness >= max_roughness:
        return "STEP", "STOP_FOR_STEP", 0.0
    return "WALK", "NAVIGATE", 1.0


def visual_evidence_in_path(
    evidence: Sequence[float], min_confidence: float, center_margin: float
) -> bool:
    """校验一条已经过视觉节点多帧确认的归一化障碍证据。

    ``center_*``、``width``、``height`` 均为相对图像尺寸的 0～1 值。这里只判断目标
    是否落在前向图像通道，不把像素框误当作米制障碍尺寸。
    """
    if len(evidence) < 6 or not all(
        isfinite(float(value)) for value in evidence[:6]
    ):
        return False
    type_code, confidence, center_x, center_y, width, height = map(
        float, evidence[:6]
    )
    margin = max(0.0, min(0.49, center_margin))
    rounded_code = round(type_code)
    known_type = 1 <= rounded_code <= 4 and abs(type_code - rounded_code) < 1e-3
    return (
        known_type
        and min_confidence <= confidence <= 1.0
        and margin <= center_x <= 1.0 - margin
        and 0.0 <= center_y <= 1.0
        and 0.0 < width <= 1.0
        and 0.0 < height <= 1.0
    )


def apply_visual_assist(
    decision: Decision, visual_active: bool, vision_speed_scale: float
) -> Decision:
    """仅在几何判断 WALK 时减速等待深度复核，绝不由视觉直接升级越障动作。"""
    mode, action, speed = decision
    if mode != "WALK" or not visual_active:
        return decision
    return mode, "VERIFY_VISUAL_OBSTACLE_WITH_DEPTH", min(speed, vision_speed_scale)


def apply_geometry_classification(
    decision: Decision, obstacle_type: int, pit_depth: float
) -> Decision:
    """在高度判定之上加入显式几何危险规则。

    坑洞、墙面、横杆在没有真机运动控制器时一律停车；立柱交给 Nav2 绕行并限速。
    未知类别不改变旧行为，便于回放旧 rosbag。
    """
    if obstacle_type == GEOMETRY_PIT and pit_depth > 0.0:
        return "STOP", "REPLAN_AROUND_PIT", 0.0
    if obstacle_type == GEOMETRY_BAR:
        return "STOP", "STOP_FOR_LOW_BAR", 0.0
    if obstacle_type == GEOMETRY_POLE and decision[0] == "WALK":
        return "WALK", "NAVIGATE_AROUND_POLE", min(decision[2], 0.35)
    if obstacle_type == GEOMETRY_WALL:
        return "STOP", "REPLAN_AROUND_WALL", 0.0
    return decision


class ObstacleCrossingManager(Node):
    """订阅感知特征，执行安全融合并持续发布可供速度门使用的决策心跳。"""

    def __init__(self):
        super().__init__("obstacle_crossing_manager")
        self.declare_parameter("step_threshold", 0.08)
        self.declare_parameter("climb_threshold", 0.18)
        self.declare_parameter("stop_threshold", 0.32)
        self.declare_parameter("max_slope", 0.45)
        self.declare_parameter("max_roughness", 0.06)
        self.declare_parameter("min_points", 30)
        self.declare_parameter("sensor_timeout", 0.7)
        self.declare_parameter("clear_confirmation_frames", 3)
        self.declare_parameter("hazard_confirmation_frames", 2)
        self.declare_parameter("vision_assist_enabled", True)
        self.declare_parameter("vision_timeout", 0.6)
        self.declare_parameter("vision_min_confidence", 0.55)
        self.declare_parameter("vision_center_margin", 0.20)
        self.declare_parameter("vision_speed_scale", 0.35)
        configured_thresholds = (
            float(self.get_parameter("step_threshold").value),
            float(self.get_parameter("climb_threshold").value),
            float(self.get_parameter("stop_threshold").value),
        )
        (
            self.step_threshold,
            self.climb_threshold,
            self.stop_threshold,
        ) = validate_height_thresholds(*configured_thresholds)
        if configured_thresholds != (
            self.step_threshold,
            self.climb_threshold,
            self.stop_threshold,
        ):
            self.get_logger().warning(
                "Invalid height thresholds; restored 0.08/0.18/0.32 m"
            )
        configured_slope = float(self.get_parameter("max_slope").value)
        configured_roughness = float(self.get_parameter("max_roughness").value)
        configured_min_points = int(self.get_parameter("min_points").value)
        configured_timeout = float(self.get_parameter("sensor_timeout").value)
        # 配置文件属于外部输入。非有限值或非正安全上限不能流入比较表达式，
        # 否则 NaN 会让所有比较为假，负阈值则可能让系统永久进入越障状态。
        self.max_slope = (
            configured_slope
            if isfinite(configured_slope) and configured_slope > 0.0
            else 0.45
        )
        self.max_roughness = (
            configured_roughness
            if isfinite(configured_roughness) and configured_roughness > 0.0
            else 0.06
        )
        self.min_points = max(1, configured_min_points)
        self.sensor_timeout = (
            configured_timeout
            if isfinite(configured_timeout) and configured_timeout > 0.0
            else 0.7
        )
        self.decision_filter = ConservativeDecisionFilter(
            int(self.get_parameter("clear_confirmation_frames").value),
            ("STOP", "WAIT_FOR_TERRAIN", 0.0),
            int(self.get_parameter("hazard_confirmation_frames").value),
        )
        self.vision_enabled = bool(self.get_parameter("vision_assist_enabled").value)
        self.vision_timeout = max(
            0.0, float(self.get_parameter("vision_timeout").value)
        )
        self.vision_min_confidence = max(
            0.0,
            min(1.0, float(self.get_parameter("vision_min_confidence").value)),
        )
        self.vision_center_margin = float(
            self.get_parameter("vision_center_margin").value
        )
        self.vision_speed_scale = max(
            0.0,
            min(1.0, float(self.get_parameter("vision_speed_scale").value)),
        )

        self.mode_pub = self.create_publisher(String, "/crossing/mode", 10)
        self.action_pub = self.create_publisher(String, "/crossing/action", 10)
        self.speed_pub = self.create_publisher(Float32, "/crossing/speed_scale", 10)
        self.visual_active_pub = self.create_publisher(
            Bool, "/crossing/visual_assist_active", 10
        )
        self.create_subscription(
            Float32MultiArray,
            "/terrain/features",
            self.features_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/vision/obstacle_evidence",
            self.vision_callback,
            10,
        )
        self.last_features_time = self.get_clock().now()
        self.last_vision_time = None
        self.visual_target = False
        self.last_decision = None
        self.timer = self.create_timer(0.1, self.timeout_callback)
        self.publish_decision("STOP", "WAIT_FOR_TERRAIN", 0.0)
        self.get_logger().info("Obstacle-crossing state machine ready")

    def features_callback(self, msg: Float32MultiArray) -> None:
        """解析一帧地形特征；短消息或非法数值最终按 STOP 处理。"""
        if len(msg.data) < 4:
            self._publish_candidate(("STOP", "WAIT_FOR_TERRAIN", 0.0))
            return
        self.last_features_time = self.get_clock().now()
        obstacle_height = (
            float(msg.data[TERRAIN_FRONTAL_HEIGHT])
            if len(msg.data) > TERRAIN_FRONTAL_HEIGHT
            else float(msg.data[TERRAIN_OBSTACLE_HEIGHT])
        )
        points = float(msg.data[TERRAIN_VALID_POINTS])
        slope = (
            float(msg.data[TERRAIN_GROUND_SLOPE])
            if len(msg.data) > TERRAIN_GROUND_SLOPE
            else 0.0
        )
        roughness = (
            float(msg.data[TERRAIN_ROUGHNESS])
            if len(msg.data) > TERRAIN_ROUGHNESS
            else 0.0
        )
        pit_depth = (
            float(msg.data[TERRAIN_PIT_DEPTH])
            if len(msg.data) > TERRAIN_PIT_DEPTH
            else 0.0
        )
        obstacle_type = (
            int(round(msg.data[TERRAIN_OBSTACLE_TYPE]))
            if len(msg.data) > TERRAIN_OBSTACLE_TYPE
            and isfinite(float(msg.data[TERRAIN_OBSTACLE_TYPE]))
            else 0
        )

        # 点云决定动作等级，视觉仅能在 WALK 状态要求减速复核。
        raw_decision = select_terrain_decision(
            obstacle_height,
            points,
            slope,
            roughness,
            self.min_points,
            self.step_threshold,
            self.climb_threshold,
            self.stop_threshold,
            self.max_slope,
            self.max_roughness,
        )
        raw_decision = apply_geometry_classification(
            raw_decision, obstacle_type, pit_depth
        )
        decision = self.decision_filter.update(raw_decision)
        visual_active = self._fresh_visual_target()
        mode, action, speed = apply_visual_assist(
            decision, visual_active, self.vision_speed_scale
        )
        self.visual_active_pub.publish(Bool(data=visual_active))
        self.publish_decision(mode, action, speed)

    def _publish_candidate(self, candidate: Decision) -> None:
        """让错误/超时结果也经过统一迟滞；STOP 因等级最高仍会立即生效。"""
        decision = self.decision_filter.update(candidate)
        self.publish_decision(*decision)

    def vision_callback(self, msg: Float32MultiArray) -> None:
        """保存视觉证据及接收时间；过期证据不会继续限制新决策。"""
        self.last_vision_time = self.get_clock().now()
        self.visual_target = visual_evidence_in_path(
            msg.data,
            self.vision_min_confidence,
            self.vision_center_margin,
        )

    def _fresh_visual_target(self) -> bool:
        """仅当视觉启用、字段有效且消息未超时时返回真。"""
        if not self.vision_enabled or self.last_vision_time is None:
            return False
        age = (self.get_clock().now() - self.last_vision_time).nanoseconds / 1e9
        return age <= self.vision_timeout and self.visual_target

    def timeout_callback(self) -> None:
        """独立于订阅回调检查地形心跳，断流时主动重复发布 STOP。"""
        age = (self.get_clock().now() - self.last_features_time).nanoseconds / 1e9
        if age > self.sensor_timeout:
            self._publish_candidate(("STOP", "WAIT_FOR_TERRAIN", 0.0))

    def publish_decision(self, mode: str, action: str, speed: float) -> None:
        """发布三个兼容话题；每帧发布心跳，但只在内容变化时记录日志。"""
        mode_msg = String()
        mode_msg.data = mode
        action_msg = String()
        action_msg.data = action
        speed_msg = Float32()
        speed_msg.data = speed
        self.mode_pub.publish(mode_msg)
        self.action_pub.publish(action_msg)
        self.speed_pub.publish(speed_msg)
        decision = (mode, action, speed)
        if decision != self.last_decision:
            self.get_logger().info(
                f"Crossing mode -> {mode}, action -> {action}, speed -> {speed:.2f}"
            )
            self.last_decision = decision


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCrossingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

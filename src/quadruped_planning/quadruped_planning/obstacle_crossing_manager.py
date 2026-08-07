"""Terrain-to-behavior state machine for the quadruped prototype."""

from math import isfinite
from typing import Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


Decision = Tuple[str, str, float]


def validate_height_thresholds(
    step: float, climb: float, stop: float
) -> Tuple[float, float, float]:
    """Return ordered non-negative thresholds or conservative defaults."""
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
    """Return the conservative motion recommendation for terrain geometry."""
    values = (obstacle_height, points, slope, roughness)
    # 无效或稀疏点云必须停机，不能把“没看见”当作“可以通过”。
    if not all(isfinite(value) for value in values) or points < min_points:
        return "STOP", "WAIT_FOR_TERRAIN", 0.0
    # 从最危险条件向下判断，确保高墙不会被较低的 STEP 阈值截获。
    if obstacle_height >= stop_threshold or slope >= max_slope * 1.5:
        return "STOP", "REPLAN_OR_REQUEST_FOOTSTEPS", 0.0
    if obstacle_height >= climb_threshold or slope >= max_slope:
        return "CLIMB", "CROSS_CLIMB", 0.20
    if obstacle_height >= step_threshold or roughness >= max_roughness:
        return "STEP", "CROSS_STEP", 0.45
    return "WALK", "NAVIGATE", 1.0


def visual_evidence_in_path(
    evidence: Sequence[float], min_confidence: float, center_margin: float
) -> bool:
    """Validate one atomic, temporally confirmed OpenCV obstacle result."""
    if len(evidence) < 6 or not all(
        isfinite(float(value)) for value in evidence[:6]
    ):
        return False
    type_code, confidence, center_x, _, width, height = map(float, evidence[:6])
    margin = max(0.0, min(0.49, center_margin))
    rounded_code = round(type_code)
    known_type = 1 <= rounded_code <= 4 and abs(type_code - rounded_code) < 1e-3
    return (
        known_type
        and confidence >= min_confidence
        and margin <= center_x <= 1.0 - margin
        and width > 0.0
        and height > 0.0
    )


def apply_visual_assist(
    decision: Decision, visual_active: bool, vision_speed_scale: float
) -> Decision:
    """Slow clear-terrain navigation while depth confirms visual evidence."""
    mode, action, speed = decision
    if mode != "WALK" or not visual_active:
        return decision
    return mode, "VERIFY_VISUAL_OBSTACLE_WITH_DEPTH", min(speed, vision_speed_scale)


class ObstacleCrossingManager(Node):
    """Turn local terrain features into safe gait recommendations."""

    def __init__(self):
        super().__init__("obstacle_crossing_manager")
        self.declare_parameter("step_threshold", 0.08)
        self.declare_parameter("climb_threshold", 0.18)
        self.declare_parameter("stop_threshold", 0.32)
        self.declare_parameter("max_slope", 0.45)
        self.declare_parameter("max_roughness", 0.06)
        self.declare_parameter("min_points", 30)
        self.declare_parameter("sensor_timeout", 0.7)
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
        self.max_slope = float(self.get_parameter("max_slope").value)
        self.max_roughness = float(self.get_parameter("max_roughness").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)
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
        if len(msg.data) < 4:
            self.publish_decision("STOP", "WAIT_FOR_TERRAIN", 0.0)
            return
        self.last_features_time = self.get_clock().now()
        obstacle_height = float(msg.data[6]) if len(msg.data) > 6 else float(msg.data[2])
        points = float(msg.data[3])
        slope = abs(float(msg.data[4])) if len(msg.data) > 4 else 0.0
        roughness = float(msg.data[5]) if len(msg.data) > 5 else 0.0

        # 点云决定动作等级，视觉仅能在 WALK 状态要求减速复核。
        decision = select_terrain_decision(
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
        visual_active = self._fresh_visual_target()
        mode, action, speed = apply_visual_assist(
            decision, visual_active, self.vision_speed_scale
        )
        self.visual_active_pub.publish(Bool(data=visual_active))
        self.publish_decision(mode, action, speed)

    def vision_callback(self, msg: Float32MultiArray) -> None:
        self.last_vision_time = self.get_clock().now()
        self.visual_target = visual_evidence_in_path(
            msg.data,
            self.vision_min_confidence,
            self.vision_center_margin,
        )

    def _fresh_visual_target(self) -> bool:
        if not self.vision_enabled or self.last_vision_time is None:
            return False
        age = (self.get_clock().now() - self.last_vision_time).nanoseconds / 1e9
        return age <= self.vision_timeout and self.visual_target

    def timeout_callback(self) -> None:
        age = (self.get_clock().now() - self.last_features_time).nanoseconds / 1e9
        if age > self.sensor_timeout:
            self.publish_decision("STOP", "WAIT_FOR_TERRAIN", 0.0)

    def publish_decision(self, mode: str, action: str, speed: float) -> None:
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

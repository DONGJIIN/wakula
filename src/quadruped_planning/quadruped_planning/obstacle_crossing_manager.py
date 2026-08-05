"""Terrain-to-behavior state machine for the quadruped prototype."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String


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
        self.step_threshold = float(self.get_parameter("step_threshold").value)
        self.climb_threshold = float(self.get_parameter("climb_threshold").value)
        self.stop_threshold = float(self.get_parameter("stop_threshold").value)
        self.max_slope = float(self.get_parameter("max_slope").value)
        self.max_roughness = float(self.get_parameter("max_roughness").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)

        self.mode_pub = self.create_publisher(String, "/crossing/mode", 10)
        self.action_pub = self.create_publisher(String, "/crossing/action", 10)
        self.speed_pub = self.create_publisher(Float32, "/crossing/speed_scale", 10)
        self.create_subscription(Float32MultiArray, "/terrain/features", self.features_callback, 10)
        self.last_features_time = self.get_clock().now()
        self.last_mode = None
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

        if points < self.min_points:
            mode, action, speed = "STOP", "WAIT_FOR_TERRAIN", 0.0
        elif obstacle_height >= self.stop_threshold or slope >= self.max_slope * 1.5:
            mode, action, speed = "STOP", "REPLAN_OR_REQUEST_FOOTSTEPS", 0.0
        elif obstacle_height >= self.climb_threshold or slope >= self.max_slope:
            mode, action, speed = "CLIMB", "CROSS_CLIMB", 0.20
        elif obstacle_height >= self.step_threshold or roughness >= self.max_roughness:
            mode, action, speed = "STEP", "CROSS_STEP", 0.45
        else:
            mode, action, speed = "WALK", "NAVIGATE", 1.0
        self.publish_decision(mode, action, speed)

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
        if mode != self.last_mode:
            self.get_logger().info(f"Crossing mode -> {mode}, action -> {action}, speed -> {speed:.2f}")
            self.last_mode = mode


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCrossingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

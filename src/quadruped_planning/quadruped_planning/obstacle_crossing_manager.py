"""Select a conservative crossing mode from terrain perception features."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String


class ObstacleCrossingManager(Node):
    """Convert terrain height span into a behavior request and speed limit."""

    def __init__(self):
        super().__init__("obstacle_crossing_manager")
        self.declare_parameter("step_threshold", 0.08)
        self.declare_parameter("climb_threshold", 0.18)
        self.declare_parameter("stop_threshold", 0.32)
        self.step_threshold = float(self.get_parameter("step_threshold").value)
        self.climb_threshold = float(self.get_parameter("climb_threshold").value)
        self.stop_threshold = float(self.get_parameter("stop_threshold").value)

        self.mode_pub = self.create_publisher(String, "crossing/mode", 10)
        self.speed_pub = self.create_publisher(Float32, "crossing/speed_scale", 10)
        self.subscription = self.create_subscription(
            Float32MultiArray, "terrain/features", self.features_callback, 10
        )
        self.last_mode = None
        self.get_logger().info("Obstacle-crossing manager ready")

    def features_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warning("Terrain feature message is incomplete")
            return
        height_span = float(msg.data[2])
        if height_span >= self.stop_threshold:
            mode, speed = "STOP", 0.0
        elif height_span >= self.climb_threshold:
            mode, speed = "CLIMB", 0.20
        elif height_span >= self.step_threshold:
            mode, speed = "STEP", 0.45
        else:
            mode, speed = "WALK", 1.0

        mode_msg = String()
        mode_msg.data = mode
        speed_msg = Float32()
        speed_msg.data = speed
        self.mode_pub.publish(mode_msg)
        self.speed_pub.publish(speed_msg)
        if mode != self.last_mode:
            self.get_logger().info(
                f"Crossing mode -> {mode} (height span {height_span:.3f} m)"
            )
            self.last_mode = mode


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


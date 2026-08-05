"""Safety gate between Nav2 velocity output and a quadruped driver.

This is intentionally conservative: terrain analysis can slow or stop the
base command, while a vendor adapter can later translate the crossing mode
into a footstep controller action.
"""

from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


class CmdVelGate(Node):
    def __init__(self):
        super().__init__("quadruped_cmd_vel_gate")
        self.declare_parameter("input_topic", "/cmd_vel_nav")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("default_speed_scale", 1.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.timeout = float(self.get_parameter("command_timeout").value)
        self.speed_scale = float(self.get_parameter("default_speed_scale").value)
        # Fail-safe until terrain perception reports a valid mode.
        self.mode = "STOP"
        self.latest_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        self.create_subscription(String, "/crossing/mode", self.mode_callback, 10)
        self.create_subscription(
            Float32, "/crossing/speed_scale", self.scale_callback, 10
        )
        self.timer = self.create_timer(0.05, self.publish_safe_command)
        self.get_logger().info(f"Velocity gate: {input_topic} -> {output_topic}")

    def cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def mode_callback(self, msg: String) -> None:
        self.mode = msg.data.upper()

    def scale_callback(self, msg: Float32) -> None:
        self.speed_scale = max(0.0, min(1.0, float(msg.data)))

    def publish_safe_command(self) -> None:
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        output = Twist()
        if age <= self.timeout and self.mode != "STOP":
            output.linear.x = self.latest_cmd.linear.x * self.speed_scale
            output.linear.y = self.latest_cmd.linear.y * self.speed_scale
            output.linear.z = self.latest_cmd.linear.z
            output.angular.x = self.latest_cmd.angular.x
            output.angular.y = self.latest_cmd.angular.y
            output.angular.z = self.latest_cmd.angular.z * self.speed_scale
        self.pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

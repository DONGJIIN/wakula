"""Fail-safe speed gate between Nav2 smoothing and collision monitoring."""

from math import isfinite

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


def gated_twist(
    source: Twist, scale: float, command_fresh: bool, decision_fresh: bool
) -> Twist:
    """Return a scaled command only when both independent heartbeats are fresh."""
    output = Twist()
    # 两条独立心跳任一失效都输出默认构造的零 Twist。
    if not command_fresh or not decision_fresh or scale <= 0.0:
        return output
    output.linear.x = source.linear.x * scale
    output.linear.y = source.linear.y * scale
    output.linear.z = source.linear.z * scale
    output.angular.x = source.angular.x * scale
    output.angular.y = source.angular.y * scale
    output.angular.z = source.angular.z * scale
    return output


class CmdVelGate(Node):
    """Stop motion whenever the planner or crossing decision becomes stale."""

    def __init__(self):
        super().__init__("quadruped_cmd_vel_gate")
        self.declare_parameter("input_topic", "/cmd_vel_smoothed")
        self.declare_parameter("output_topic", "/cmd_vel_terrain_safe")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("decision_timeout", 0.7)
        self.declare_parameter("default_speed_scale", 0.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.timeout = float(self.get_parameter("command_timeout").value)
        self.decision_timeout = float(
            self.get_parameter("decision_timeout").value
        )
        self.speed_scale = max(
            0.0,
            min(1.0, float(self.get_parameter("default_speed_scale").value)),
        )
        self.latest_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_decision_time = self.get_clock().now()

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        self.create_subscription(
            Float32, "/crossing/speed_scale", self.scale_callback, 10
        )
        self.timer = self.create_timer(0.05, self.publish_safe_command)
        self.get_logger().info(f"Velocity gate: {input_topic} -> {output_topic}")

    def cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def scale_callback(self, msg: Float32) -> None:
        value = float(msg.data)
        self.speed_scale = max(0.0, min(1.0, value)) if isfinite(value) else 0.0
        self.last_decision_time = self.get_clock().now()

    def publish_safe_command(self) -> None:
        now = self.get_clock().now()
        command_age = (now - self.last_cmd_time).nanoseconds / 1e9
        decision_age = (now - self.last_decision_time).nanoseconds / 1e9
        # 每 50 ms 重新计算，而不是沿用上一条非零速度，防止失联后继续走。
        output = gated_twist(
            self.latest_cmd,
            self.speed_scale,
            command_age <= self.timeout,
            decision_age <= self.decision_timeout,
        )
        self.pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

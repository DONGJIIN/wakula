"""Nav2 与底盘之间的失效安全速度门。

只有 Nav2 速度命令、地形安全评估和导航健康三条独立心跳都有效且新鲜时才允许非零
输出。节点不规划路线、不创建新的运动意图，只约束 Nav2 已经计算出的 Twist。它也不替代
硬件急停、驱动器看门狗或姿态保护，它只是防止 ROS 节点失联后沿用最后一条速度。
"""

from math import isfinite

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def gated_twist(
    source: Twist,
    limit: float,
    command_fresh: bool,
    decision_fresh: bool,
    navigation_healthy: bool = True,
    health_fresh: bool = True,
) -> Twist:
    """仅在命令、地形评估和导航健康状态均有效时缩放 Twist。"""
    output = Twist()
    # 三条独立安全条件任一失效都输出默认构造的零 Twist。
    if (
        not command_fresh
        or not decision_fresh
        or not navigation_healthy
        or not health_fresh
        or not isfinite(limit)
        or limit <= 0.0
    ):
        return output
    safe_limit = min(1.0, limit)
    output.linear.x = source.linear.x * safe_limit
    output.linear.y = source.linear.y * safe_limit
    output.linear.z = source.linear.z * safe_limit
    output.angular.x = source.angular.x * safe_limit
    output.angular.y = source.angular.y * safe_limit
    output.angular.z = source.angular.z * safe_limit
    return output


class NavigationSpeedGate(Node):
    """以 20 Hz 重算导航速度，命令或地形评估任一超时即归零。"""

    def __init__(self):
        super().__init__("navigation_speed_gate")
        self.declare_parameter("input_topic", "/cmd_vel_smoothed")
        self.declare_parameter("output_topic", "/cmd_vel_terrain_safe")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("assessment_timeout", 0.7)
        self.declare_parameter("navigation_health_timeout", 0.5)
        self.declare_parameter("require_navigation_health", True)
        self.declare_parameter("default_speed_limit", 0.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        configured_command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        configured_assessment_timeout = float(
            self.get_parameter("assessment_timeout").value
        )
        self.command_timeout = (
            configured_command_timeout
            if isfinite(configured_command_timeout) and configured_command_timeout > 0.0
            else 0.5
        )
        self.assessment_timeout = (
            configured_assessment_timeout
            if isfinite(configured_assessment_timeout)
            and configured_assessment_timeout > 0.0
            else 0.7
        )
        configured_health_timeout = float(
            self.get_parameter("navigation_health_timeout").value
        )
        self.health_timeout = (
            configured_health_timeout
            if isfinite(configured_health_timeout)
            and configured_health_timeout > 0.0
            else 0.5
        )
        self.require_navigation_health = bool(
            self.get_parameter("require_navigation_health").value
        )
        configured_limit = float(self.get_parameter("default_speed_limit").value)
        self.speed_limit = (
            max(0.0, min(1.0, configured_limit))
            if isfinite(configured_limit)
            else 0.0
        )
        self.latest_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_assessment_time = self.get_clock().now()
        self.navigation_healthy = not self.require_navigation_health
        self.last_health_time = None

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        self.create_subscription(
            Float32, "/terrain/speed_limit", self.limit_callback, 10
        )
        self.create_subscription(
            Bool, "/navigation/healthy", self.health_callback, 10
        )
        self.timer = self.create_timer(0.05, self.publish_safe_command)
        self.get_logger().info(f"Velocity gate: {input_topic} -> {output_topic}")

    def cmd_callback(self, msg: Twist) -> None:
        """缓存 Nav2 最新速度及本机接收时刻。"""
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def limit_callback(self, msg: Float32) -> None:
        """接收 0～1 速度上限；NaN/Inf 按零处理。"""
        value = float(msg.data)
        self.speed_limit = max(0.0, min(1.0, value)) if isfinite(value) else 0.0
        self.last_assessment_time = self.get_clock().now()

    def health_callback(self, msg: Bool) -> None:
        """缓存导航健康心跳；false 或断流都会关闭非零速度输出。"""
        self.navigation_healthy = bool(msg.data)
        self.last_health_time = self.get_clock().now()

    def publish_safe_command(self) -> None:
        """依据本机 ROS 时钟计算心跳年龄并始终发布一条明确命令。"""
        now = self.get_clock().now()
        command_age = (now - self.last_cmd_time).nanoseconds / 1e9
        assessment_age = (now - self.last_assessment_time).nanoseconds / 1e9
        health_age = (
            float("inf")
            if self.last_health_time is None
            else (now - self.last_health_time).nanoseconds / 1e9
        )
        # 每 50 ms 重新计算，而不是沿用上一条非零速度，防止失联后继续走。
        output = gated_twist(
            self.latest_cmd,
            self.speed_limit,
            command_age <= self.command_timeout,
            assessment_age <= self.assessment_timeout,
            self.navigation_healthy,
            not self.require_navigation_health
            or health_age <= self.health_timeout,
        )
        self.pub.publish(output)


def main(args=None):
    """启动 Nav2 速度安全门。"""
    rclpy.init(args=args)
    node = NavigationSpeedGate()
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

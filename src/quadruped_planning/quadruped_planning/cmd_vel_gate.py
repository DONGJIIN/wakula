"""Nav2 与底盘之间的失效安全速度门。

只有 Nav2 速度命令和越障决策两条独立心跳都新鲜时才允许非零输出。节点本身不替代
硬件急停、驱动器看门狗或姿态保护，它只是防止 ROS 节点失联后沿用最后一条速度。
"""

from math import isfinite

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32


def gated_twist(
    source: Twist,
    scale: float,
    command_fresh: bool,
    decision_fresh: bool,
    safety_stop: bool = False,
    crossing_active: bool = False,
    safety_fresh: bool = True,
) -> Twist:
    """仅在心跳有效且无安全/动作互锁时缩放 Twist，否则输出零。"""
    output = Twist()
    # 两条独立心跳任一失效都输出默认构造的零 Twist。
    if (
        not command_fresh
        or not decision_fresh
        or safety_stop
        or not safety_fresh
        or crossing_active
        or scale <= 0.0
    ):
        return output
    output.linear.x = source.linear.x * scale
    output.linear.y = source.linear.y * scale
    output.linear.z = source.linear.z * scale
    output.angular.x = source.angular.x * scale
    output.angular.y = source.angular.y * scale
    output.angular.z = source.angular.z * scale
    return output


class CmdVelGate(Node):
    """以 20 Hz 重算安全速度，规划或地形决策任一超时即停车。"""

    def __init__(self):
        super().__init__("quadruped_cmd_vel_gate")
        self.declare_parameter("input_topic", "/cmd_vel_smoothed")
        self.declare_parameter("output_topic", "/cmd_vel_terrain_safe")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("decision_timeout", 0.7)
        self.declare_parameter("safety_timeout", 0.5)
        self.declare_parameter("default_speed_scale", 0.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.timeout = float(self.get_parameter("command_timeout").value)
        self.decision_timeout = float(
            self.get_parameter("decision_timeout").value
        )
        self.safety_timeout = max(
            0.05, float(self.get_parameter("safety_timeout").value)
        )
        self.speed_scale = max(
            0.0,
            min(1.0, float(self.get_parameter("default_speed_scale").value)),
        )
        self.latest_cmd = Twist()
        # 安全监督器启动前保持停车；收到明确 false 后才允许 Nav2 速度通过。
        self.safety_stop = True
        self.crossing_active = False
        self.last_cmd_time = self.get_clock().now()
        self.last_decision_time = self.get_clock().now()
        self.last_safety_time = None

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        self.create_subscription(
            Float32, "/crossing/speed_scale", self.scale_callback, 10
        )
        self.create_subscription(Bool, "/safety/stop", self.safety_callback, 10)
        active_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            "/crossing/execution_active",
            self.crossing_active_callback,
            active_qos,
        )
        self.timer = self.create_timer(0.05, self.publish_safe_command)
        self.get_logger().info(f"Velocity gate: {input_topic} -> {output_topic}")

    def cmd_callback(self, msg: Twist) -> None:
        """缓存 Nav2 最新速度及本机接收时刻。"""
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def scale_callback(self, msg: Float32) -> None:
        """接收 0～1 速度比例；NaN/Inf 或越界值按安全范围处理。"""
        value = float(msg.data)
        self.speed_scale = max(0.0, min(1.0, value)) if isfinite(value) else 0.0
        self.last_decision_time = self.get_clock().now()

    def safety_callback(self, msg: Bool) -> None:
        """安全监督器的停车请求不可被速度缩放覆盖。"""
        self.safety_stop = bool(msg.data)
        self.last_safety_time = self.get_clock().now()

    def crossing_active_callback(self, msg: Bool) -> None:
        """越障控制器接管腿部时暂停普通 Nav2 速度。"""
        self.crossing_active = bool(msg.data)

    def publish_safe_command(self) -> None:
        """依据本机 ROS 时钟计算心跳年龄并始终发布一条明确命令。"""
        now = self.get_clock().now()
        command_age = (now - self.last_cmd_time).nanoseconds / 1e9
        decision_age = (now - self.last_decision_time).nanoseconds / 1e9
        safety_age = (
            float("inf")
            if self.last_safety_time is None
            else (now - self.last_safety_time).nanoseconds / 1e9
        )
        # 每 50 ms 重新计算，而不是沿用上一条非零速度，防止失联后继续走。
        output = gated_twist(
            self.latest_cmd,
            self.speed_scale,
            command_age <= self.timeout,
            decision_age <= self.decision_timeout,
            self.safety_stop,
            self.crossing_active,
            safety_age <= self.safety_timeout,
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

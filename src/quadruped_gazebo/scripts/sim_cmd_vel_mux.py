#!/usr/bin/env python3
"""Gazebo 测试载体专用的手动/自主速度仲裁器。

核心算法继续把最终安全速度发布到标准 ``/cmd_vel``；键盘测试单独发布到
``/cmd_vel_teleop``。收到手动命令后的短时间内，手动输入拥有更高优先级，随后自动恢复
算法输入。最终命令写入仅供 Gazebo bridge 使用的 ``/cmd_vel_gazebo``，从而避免 Collision
Monitor 的周期零速度覆盖键盘的单次 j/l 指令。

该节点只安装在 quadruped_gazebo，不属于真机速度仲裁或运动控制实现。
"""

from math import isfinite
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _positive(value: float, fallback: float) -> float:
    """过滤 NaN、无穷和非正参数，保证看门狗不会被错误配置永久关闭。"""
    return value if isfinite(value) and value > 0.0 else fallback


def select_command(
    now: float,
    manual: Twist,
    manual_stamp: float | None,
    autonomous: Twist,
    autonomous_stamp: float | None,
    manual_timeout: float,
    autonomous_timeout: float,
) -> Twist:
    """按“新鲜手动 > 新鲜自主 > 零速度”选择输出，不修改原消息内容。"""
    if manual_stamp is not None and 0.0 <= now - manual_stamp <= manual_timeout:
        return manual
    if (
        autonomous_stamp is not None
        and 0.0 <= now - autonomous_stamp <= autonomous_timeout
    ):
        return autonomous
    return Twist()


class SimCmdVelMux(Node):
    """使用单调墙钟实现仿真速度优先级和断流停车。"""

    def __init__(self) -> None:
        super().__init__("sim_cmd_vel_mux")
        self.declare_parameter("autonomous_topic", "/cmd_vel")
        self.declare_parameter("manual_topic", "/cmd_vel_teleop")
        self.declare_parameter("output_topic", "/cmd_vel_gazebo")
        self.declare_parameter("manual_timeout", 0.7)
        self.declare_parameter("autonomous_timeout", 0.5)
        self.declare_parameter("publish_rate", 30.0)

        self.manual_timeout = _positive(
            float(self.get_parameter("manual_timeout").value), 0.7
        )
        self.autonomous_timeout = _positive(
            float(self.get_parameter("autonomous_timeout").value), 0.5
        )
        publish_rate = _positive(float(self.get_parameter("publish_rate").value), 30.0)

        self.manual = Twist()
        self.autonomous = Twist()
        self.manual_stamp = None
        self.autonomous_stamp = None
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(
            Twist,
            str(self.get_parameter("manual_topic").value),
            self._manual_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("autonomous_topic").value),
            self._autonomous_callback,
            10,
        )
        # 节点刻意使用系统时间而不是 /clock：仿真暂停/退出后仍能发布零速度并清除旧命令。
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            "Simulation velocity mux: /cmd_vel_teleop (priority) + /cmd_vel "
            "-> /cmd_vel_gazebo"
        )

    def _manual_callback(self, msg: Twist) -> None:
        """缓存键盘/手柄命令；包括零 Twist，确保松键会立即停车。"""
        self.manual = msg
        self.manual_stamp = time.monotonic()

    def _autonomous_callback(self, msg: Twist) -> None:
        """缓存算法最终安全命令，不在此处绕过任何 Nav2 安全链。"""
        self.autonomous = msg
        self.autonomous_stamp = time.monotonic()

    def _publish(self) -> None:
        """按优先级发布一个唯一 Gazebo 速度源，输入断流则持续输出零速度。"""
        self.publisher.publish(
            select_command(
                time.monotonic(),
                self.manual,
                self.manual_stamp,
                self.autonomous,
                self.autonomous_stamp,
                self.manual_timeout,
                self.autonomous_timeout,
            )
        )


def main(args=None) -> None:
    """ROS 2 console entry point。"""
    rclpy.init(args=args)
    node = SimCmdVelMux()
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

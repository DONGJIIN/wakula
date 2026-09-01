#!/usr/bin/env python3
"""Gazebo 测试载体专用的手动/自主速度仲裁器。

核心算法继续把最终安全速度发布到标准 ``/cmd_vel``；键盘测试单独发布到
``/cmd_vel_teleop``，Xbox 节点发布到 ``/cmd_vel_joy``。收到手动命令后的短时间内，最新的
人工输入拥有最高优先级，随后才恢复算法输入。最终命令写入仅供 Gazebo bridge 使用的
``/cmd_vel_gazebo``，从而避免人工候选与导航速度门同时竞争 Gazebo 的单一速度入口。

``/navigation/autonomy_stop`` 只锁住自动导航分支。结束自主 launch 后，若没有人工输入，
最终输出立即归零；键盘或手柄继续发布时仍可人工接管，不需要重新启动自主功能。

``/teleop/emergency_stop`` 是 Xbox B 键锁存的仿真全局软件停车请求：为 true 时人工和自主
候选都被清空并持续输出零；Start 安全解锁后仍须收到一条新命令才会再次运动。它仅用于验证
ROS 仲裁合同，不能替代真机的实体急停、驱动失能或底层看门狗。

该节点只安装在 quadruped_gazebo，不属于真机速度仲裁或运动控制实现。
"""

from math import isfinite
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


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
    autonomy_stop: bool = False,
    emergency_stop: bool = False,
) -> Twist:
    """按“全局软件停车 > 新鲜人工 > 自主锁 > 新鲜自主 > 零”选择输出。"""
    if emergency_stop:
        return Twist()
    if manual_stamp is not None and 0.0 <= now - manual_stamp <= manual_timeout:
        return manual
    if autonomy_stop:
        return Twist()
    if (
        autonomous_stamp is not None
        and 0.0 <= now - autonomous_stamp <= autonomous_timeout
    ):
        return autonomous
    return Twist()


class SimCmdVelMux(Node):
    """使用单调墙钟实现仿真速度优先级和断流停车。"""

    def __init__(self) -> None:
        """Create the sole Gazebo Twist output and its independent safety gates.

        ``autonomy_stop`` suppresses only the autonomous candidate so an operator can
        take over.  The transient-local Xbox software stop is different: it clears and
        blocks every Twist candidate until explicitly released.  Pose-based traversal
        is guarded separately by the simulation Action server because it bypasses this
        velocity mux entirely.
        """
        super().__init__("sim_cmd_vel_mux")
        self.declare_parameter("autonomous_topic", "/cmd_vel")
        self.declare_parameter("manual_topic", "/cmd_vel_teleop")
        self.declare_parameter("joystick_topic", "/cmd_vel_joy")
        self.declare_parameter("joystick_active_topic", "/teleop/active")
        self.declare_parameter("emergency_stop_topic", "/teleop/emergency_stop")
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
        self.joystick = Twist()
        self.autonomous = Twist()
        self.manual_stamp = None
        self.joystick_stamp = None
        self.joystick_active = False
        self.autonomous_stamp = None
        self.autonomy_stop = False
        self.emergency_stop = False
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
            str(self.get_parameter("joystick_topic").value),
            self._joystick_callback,
            10,
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            str(self.get_parameter("joystick_active_topic").value),
            self._joystick_active_callback,
            state_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self._emergency_stop_callback,
            state_qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("autonomous_topic").value),
            self._autonomous_callback,
            10,
        )
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            "/navigation/autonomy_stop",
            self._stop_callback,
            stop_qos,
        )
        # 节点刻意使用系统时间而不是 /clock：仿真暂停/退出后仍能发布零速度并清除旧命令。
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            "Simulation velocity mux: latest /cmd_vel_teleop or /cmd_vel_joy "
            "(manual priority) + /cmd_vel -> /cmd_vel_gazebo"
        )

    def _manual_callback(self, msg: Twist) -> None:
        """缓存键盘/手柄命令；包括零 Twist，确保松键会立即停车。"""
        self.manual = msg
        self.manual_stamp = time.monotonic()

    def _autonomous_callback(self, msg: Twist) -> None:
        """缓存算法最终安全命令，不在此处绕过任何 Nav2 安全链。"""
        self.autonomous = msg
        self.autonomous_stamp = time.monotonic()

    def _joystick_callback(self, msg: Twist) -> None:
        """缓存 Xbox 候选速度；与键盘同时存在时采用时间更新的一路。"""
        self.joystick = msg
        if self.joystick_active:
            self.joystick_stamp = time.monotonic()

    def _joystick_active_callback(self, msg: Bool) -> None:
        """只有按住 Xbox LB 并通过节点安全状态机后，手柄才参与最终仲裁。"""
        self.joystick_active = bool(msg.data)
        if not self.joystick_active:
            self.joystick_stamp = None

    def _stop_callback(self, msg: Bool) -> None:
        """锁住自动导航并先发一帧零速；随后有效人工输入可在下一周期接管。"""
        self.autonomy_stop = bool(msg.data)
        if self.autonomy_stop:
            # 不等待 30 Hz 定时器，也不复用最后一条自主 Twist。该零速只负责终止自主
            # 运动；下一次定时发布仍按“人工 > 自主锁 > 自主”仲裁。
            self.autonomous = Twist()
            self.autonomous_stamp = None
            self.publisher.publish(Twist())

    def _emergency_stop_callback(self, msg: Bool) -> None:
        """锁存仿真全局软件停车，并在触发边沿原子清除全部旧速度候选。"""
        self.emergency_stop = bool(msg.data)
        if not self.emergency_stop:
            return
        # 清缓存保证 Start 解锁后不会复用急停前仍处于 timeout 窗口内的非零 Twist。
        self.manual = Twist()
        self.joystick = Twist()
        self.autonomous = Twist()
        self.manual_stamp = None
        self.joystick_stamp = None
        self.autonomous_stamp = None
        self.publisher.publish(Twist())

    def _publish(self) -> None:
        """按优先级发布一个唯一 Gazebo 速度源，输入断流则持续输出零速度。"""
        # 键盘与手柄都是人工候选，采用最后收到的一路。人工候选位于自动导航锁之前，
        # 因此 Ctrl-C 只清除自主运动，不剥夺操作员的人工接管权。
        if self.joystick_stamp is not None and (
            self.manual_stamp is None or self.joystick_stamp > self.manual_stamp
        ):
            manual = self.joystick
            manual_stamp = self.joystick_stamp
        else:
            manual = self.manual
            manual_stamp = self.manual_stamp
        self.publisher.publish(
            select_command(
                time.monotonic(),
                manual,
                manual_stamp,
                self.autonomous,
                self.autonomous_stamp,
                self.manual_timeout,
                self.autonomous_timeout,
                self.autonomy_stop,
                self.emergency_stop,
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
    except RuntimeError:
        # Jazzy can invalidate a subscription handle while the executor is taking
        # the final DDS sample during a launch-wide SIGINT.  That teardown race is
        # harmless only after the ROS context has already stopped; a RuntimeError
        # during normal operation must still surface instead of being hidden.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

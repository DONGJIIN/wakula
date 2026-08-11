"""把标准 Xbox ``sensor_msgs/Joy`` 安全转换为四足底盘 ``Twist``。

默认控制布局（Linux ``joy_node`` 常见映射）：

* 左摇杆上下：前进/后退；左摇杆左右：横移；右摇杆左右：原地转向；
* LB：摇杆回中时按下完成安全解锁，之后必须持续按住；松手立即输出零速度；
* A/X/Y：切换低速/正常/快速三档；
* B：锁存软件急停；Start：在松开 LB 且摇杆回中时解除软件急停；
* RB、Back、Guide、左右摇杆按下、LT/RT 和十字键：当前预留，不产生动作。

节点只生成机身期望速度，不包含步态、关节或越障控制。默认输出 ``/cmd_vel_joy``，避免
与 Nav2/Collision Monitor 同时发布 ``/cmd_vel``；真机应使用速度仲裁器选择手柄或自主导航。
软件急停只是 ROS 层保护，不能替代实体急停、驱动失能和底层通信看门狗。
"""

from dataclasses import dataclass
from math import copysign, isfinite
from typing import Sequence

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


# Xbox 按键的常见 Linux 顺序。配置文件仍可覆盖真正参与控制的按键，以适配不同驱动。
XBOX_BUTTON_A = 0          # 低速档
XBOX_BUTTON_B = 1          # 锁存软件急停
XBOX_BUTTON_X = 2          # 正常档
XBOX_BUTTON_Y = 3          # 快速档
XBOX_BUTTON_LB = 4         # 按住才允许运动（使能/死人开关）
XBOX_BUTTON_RB = 5         # 预留，不产生动作
XBOX_BUTTON_BACK = 6       # 预留，不产生动作
XBOX_BUTTON_START = 7      # 安全条件满足时解除软件急停
XBOX_BUTTON_GUIDE = 8      # 预留；部分系统不会把它发送给 joy_node
XBOX_BUTTON_LEFT_STICK = 9   # 预留，不产生动作
XBOX_BUTTON_RIGHT_STICK = 10  # 预留，不产生动作

# 常见轴顺序：LT/RT 与十字键也是轴，但当前刻意不参与运动，减少误触发来源。
XBOX_AXIS_LEFT_X = 0       # 左摇杆左右：横移
XBOX_AXIS_LEFT_Y = 1       # 左摇杆上下：前进/后退
XBOX_AXIS_LEFT_TRIGGER = 2  # 预留，不产生动作
XBOX_AXIS_RIGHT_X = 3      # 右摇杆左右：偏航
XBOX_AXIS_RIGHT_Y = 4      # 预留，不产生动作
XBOX_AXIS_RIGHT_TRIGGER = 5  # 预留，不产生动作
XBOX_AXIS_DPAD_X = 6       # 预留，不产生动作
XBOX_AXIS_DPAD_Y = 7       # 预留，不产生动作


@dataclass(frozen=True)
class TeleopConfig:
    """与硬件映射和速度有关的纯数据配置，便于脱离 ROS 做单元测试。"""

    axis_linear_x: int = XBOX_AXIS_LEFT_Y
    axis_linear_y: int = XBOX_AXIS_LEFT_X
    axis_angular_z: int = XBOX_AXIS_RIGHT_X
    button_slow: int = XBOX_BUTTON_A
    button_stop: int = XBOX_BUTTON_B
    button_normal: int = XBOX_BUTTON_X
    button_fast: int = XBOX_BUTTON_Y
    button_deadman: int = XBOX_BUTTON_LB
    button_clear_stop: int = XBOX_BUTTON_START
    deadzone: float = 0.12
    linear_x_direction: float = 1.0
    linear_y_direction: float = 1.0
    angular_z_direction: float = 1.0
    lateral_speed_scale: float = 0.60
    slow_linear_speed: float = 0.12
    slow_angular_speed: float = 0.35
    normal_linear_speed: float = 0.25
    normal_angular_speed: float = 0.60
    fast_linear_speed: float = 0.40
    fast_angular_speed: float = 0.90


@dataclass(frozen=True)
class TeleopResult:
    """一次 Joy 更新后的原子结果，包含命令、使能状态、急停状态和提示事件。"""

    twist: Twist
    active: bool
    emergency_stop: bool
    speed_mode: str
    event: str = ""


def safe_axis(axes: Sequence[float], index: int) -> float:
    """读取有限且合法的轴值；短数组、错误下标和 NaN/Inf 一律按零处理。"""
    if index < 0 or index >= len(axes):
        return 0.0
    value = float(axes[index])
    return max(-1.0, min(1.0, value)) if isfinite(value) else 0.0


def button_pressed(buttons: Sequence[int], index: int) -> bool:
    """安全读取按键；驱动缺少某个按键时返回 false，而不是抛出越界异常。"""
    return 0 <= index < len(buttons) and bool(buttons[index])


def apply_deadzone(value: float, deadzone: float) -> float:
    """消除摇杆中心漂移，并把死区外剩余行程重新线性映射到 0..1。"""
    bounded = max(-1.0, min(1.0, float(value))) if isfinite(value) else 0.0
    threshold = max(0.0, min(0.95, float(deadzone)))
    if abs(bounded) <= threshold:
        return 0.0
    return copysign((abs(bounded) - threshold) / (1.0 - threshold), bounded)


def zero_twist() -> Twist:
    """创建明确的全零 Twist；避免复用并意外修改上一条消息。"""
    return Twist()


class XboxTeleopController:
    """保存按键边沿、速度档位和锁存急停的 ROS 无关状态机。"""

    SPEED_MODES = ("slow", "normal", "fast")

    def __init__(self, config: TeleopConfig):
        """以正常档、未解锁、未急停状态启动；运动前必须在摇杆回中时按下 LB。"""
        self.config = config
        self.speed_mode = "normal"
        self.emergency_stop = False
        self.armed = False
        # 输入断流后置位；即使重连时 LB 仍被按住，也必须先松开再重新按下。
        self.rearm_requires_release = False
        self.rearm_wait_reported = False
        self.previous_buttons: Sequence[int] = ()

    def disarm_for_timeout(self) -> None:
        """手柄断流时撤销使能，并要求 LB 完成一次明确的松开动作。"""
        self.armed = False
        self.rearm_requires_release = True
        # 每次新断流只允许打印一次等待提示，避免按 /joy 频率刷屏。
        self.rearm_wait_reported = False

    def _rising_edge(self, buttons: Sequence[int], index: int) -> bool:
        """只在按键从松开变为按下时触发一次，防止每帧重复切档或清急停。"""
        return button_pressed(buttons, index) and not button_pressed(
            self.previous_buttons, index
        )

    def _sticks_centered(self, axes: Sequence[float]) -> bool:
        """解除急停前确认三路运动轴都位于死区内。"""
        return all(
            apply_deadzone(safe_axis(axes, index), self.config.deadzone) == 0.0
            for index in (
                self.config.axis_linear_x,
                self.config.axis_linear_y,
                self.config.axis_angular_z,
            )
        )

    def _speed_limits(self) -> tuple[float, float]:
        """返回当前档位的最大平移速度和最大偏航角速度。"""
        if self.speed_mode == "slow":
            return self.config.slow_linear_speed, self.config.slow_angular_speed
        if self.speed_mode == "fast":
            return self.config.fast_linear_speed, self.config.fast_angular_speed
        return self.config.normal_linear_speed, self.config.normal_angular_speed

    def update(self, axes: Sequence[float], buttons: Sequence[int]) -> TeleopResult:
        """处理一帧手柄状态；安全事件优先于速度档位和运动输出。"""
        event = ""
        deadman = button_pressed(buttons, self.config.button_deadman)
        deadman_was_pressed = button_pressed(
            self.previous_buttons, self.config.button_deadman
        )

        # B 是最高优先级：无论摇杆或 LB 状态如何，按下即锁存急停并输出零速度。
        if self._rising_edge(buttons, self.config.button_stop):
            self.emergency_stop = True
            self.armed = False
            event = "emergency_stop_latched"

        # Start 不能在仍按住使能或摇杆未回中时解除急停，防止解除瞬间突然移动。
        elif self._rising_edge(buttons, self.config.button_clear_stop):
            if not deadman and self._sticks_centered(axes):
                self.emergency_stop = False
                event = "emergency_stop_cleared"
            else:
                event = "emergency_stop_clear_rejected"

        # A/X/Y 只改变比例上限，不会自己产生速度；即使急停中也允许预选恢复后的档位。
        if self._rising_edge(buttons, self.config.button_slow):
            self.speed_mode = "slow"
            event = event or "speed_slow"
        elif self._rising_edge(buttons, self.config.button_normal):
            self.speed_mode = "normal"
            event = event or "speed_normal"
        elif self._rising_edge(buttons, self.config.button_fast):
            self.speed_mode = "fast"
            event = event or "speed_fast"

        # LB 是两阶段使能：只有从松开变为按下的那一帧摇杆已回中，才进入 armed。
        # 若带着非零摇杆按下 LB，必须松开后重新按下，防止手柄放置姿态导致突然起步。
        if not deadman:
            self.armed = False
            self.rearm_requires_release = False
            self.rearm_wait_reported = False
        elif self.rearm_requires_release:
            # 断流前若 LB 处于按下状态，重连帧不能直接恢复运动。只有收到明确的
            # LB 松开帧后，下一次“回中 + 按下”才允许重新解锁。
            self.armed = False
            if not self.rearm_wait_reported:
                event = event or "teleop_waiting_for_deadman_release"
                self.rearm_wait_reported = True
        elif not deadman_was_pressed:
            if not self.emergency_stop and self._sticks_centered(axes):
                self.armed = True
                event = event or "teleop_armed"
            else:
                self.armed = False
                event = event or "teleop_arm_rejected"

        active = deadman and self.armed and not self.emergency_stop
        command = zero_twist()
        if active:
            linear_limit, angular_limit = self._speed_limits()
            command.linear.x = apply_deadzone(
                safe_axis(axes, self.config.axis_linear_x), self.config.deadzone
            ) * max(0.0, linear_limit) * self.config.linear_x_direction
            command.linear.y = apply_deadzone(
                safe_axis(axes, self.config.axis_linear_y), self.config.deadzone
            ) * (
                max(0.0, linear_limit)
                * max(0.0, min(1.0, self.config.lateral_speed_scale))
                * self.config.linear_y_direction
            )
            command.angular.z = apply_deadzone(
                safe_axis(axes, self.config.axis_angular_z), self.config.deadzone
            ) * max(0.0, angular_limit) * self.config.angular_z_direction

        # 保存当前按键快照必须放在所有边沿判断之后。
        self.previous_buttons = tuple(buttons)
        return TeleopResult(
            twist=command,
            active=active,
            emergency_stop=self.emergency_stop,
            speed_mode=self.speed_mode,
            event=event,
        )


class XboxTeleopNode(Node):
    """订阅 /joy，以固定心跳发布 /cmd_vel_joy 和手柄安全状态。"""

    def __init__(self):
        """读取可覆盖映射、创建发布订阅，并默认保持零速度。"""
        super().__init__("xbox_teleop")

        # 话题参数让该节点保持标准接口，同时允许将来放入命名空间或速度仲裁器。
        self.declare_parameter("input_topic", "/joy")
        self.declare_parameter("output_topic", "/cmd_vel_joy")
        self.declare_parameter("active_topic", "/teleop/active")
        self.declare_parameter("emergency_stop_topic", "/teleop/emergency_stop")
        self.declare_parameter("speed_mode_topic", "/teleop/speed_mode")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("joy_timeout", 0.5)
        self.declare_parameter("deadzone", 0.12)

        # 参与控制的轴。LT/RT、十字键和右摇杆 Y 均不读取，因此不会误触发移动。
        self.declare_parameter("axis_linear_x", XBOX_AXIS_LEFT_Y)
        self.declare_parameter("axis_linear_y", XBOX_AXIS_LEFT_X)
        self.declare_parameter("axis_angular_z", XBOX_AXIS_RIGHT_X)
        # 方向参数只允许 ±1；若某个驱动轴方向相反，改 YAML 即可，无需改算法。
        self.declare_parameter("linear_x_direction", 1.0)
        self.declare_parameter("linear_y_direction", 1.0)
        self.declare_parameter("angular_z_direction", 1.0)
        # 横移通常比前进能力弱，默认限制为当前平移档位的 60%。
        self.declare_parameter("lateral_speed_scale", 0.60)

        # A/B/X/Y/LB/Start 的功能映射；所有其他 Xbox 按键明确预留。
        self.declare_parameter("button_slow", XBOX_BUTTON_A)
        self.declare_parameter("button_stop", XBOX_BUTTON_B)
        self.declare_parameter("button_normal", XBOX_BUTTON_X)
        self.declare_parameter("button_fast", XBOX_BUTTON_Y)
        self.declare_parameter("button_deadman", XBOX_BUTTON_LB)
        self.declare_parameter("button_clear_stop", XBOX_BUTTON_START)

        # 速度是未标定的保守上限；真机能力确定后只改 YAML，不改摇杆计算逻辑。
        for name, default in (
            ("slow_linear_speed", 0.12),
            ("slow_angular_speed", 0.35),
            ("normal_linear_speed", 0.25),
            ("normal_angular_speed", 0.60),
            ("fast_linear_speed", 0.40),
            ("fast_angular_speed", 0.90),
        ):
            self.declare_parameter(name, default)

        config = TeleopConfig(
            axis_linear_x=self._index_parameter("axis_linear_x", XBOX_AXIS_LEFT_Y),
            axis_linear_y=self._index_parameter("axis_linear_y", XBOX_AXIS_LEFT_X),
            axis_angular_z=self._index_parameter("axis_angular_z", XBOX_AXIS_RIGHT_X),
            button_slow=self._index_parameter("button_slow", XBOX_BUTTON_A),
            button_stop=self._index_parameter("button_stop", XBOX_BUTTON_B),
            button_normal=self._index_parameter("button_normal", XBOX_BUTTON_X),
            button_fast=self._index_parameter("button_fast", XBOX_BUTTON_Y),
            button_deadman=self._index_parameter("button_deadman", XBOX_BUTTON_LB),
            button_clear_stop=self._index_parameter(
                "button_clear_stop", XBOX_BUTTON_START
            ),
            deadzone=self._bounded_parameter("deadzone", 0.12, 0.0, 0.95),
            linear_x_direction=self._direction_parameter("linear_x_direction"),
            linear_y_direction=self._direction_parameter("linear_y_direction"),
            angular_z_direction=self._direction_parameter("angular_z_direction"),
            lateral_speed_scale=self._bounded_parameter(
                "lateral_speed_scale", 0.60, 0.0, 1.0
            ),
            slow_linear_speed=self._positive_parameter("slow_linear_speed", 0.12),
            slow_angular_speed=self._positive_parameter("slow_angular_speed", 0.35),
            normal_linear_speed=self._positive_parameter(
                "normal_linear_speed", 0.25
            ),
            normal_angular_speed=self._positive_parameter(
                "normal_angular_speed", 0.60
            ),
            fast_linear_speed=self._positive_parameter("fast_linear_speed", 0.40),
            fast_angular_speed=self._positive_parameter("fast_angular_speed", 0.90),
        )
        self.controller = XboxTeleopController(config)
        self.joy_timeout = self._positive_parameter("joy_timeout", 0.5)
        publish_rate = self._bounded_parameter("publish_rate", 20.0, 1.0, 100.0)
        self.latest_result = TeleopResult(zero_twist(), False, False, "normal")
        self.last_joy_time = None

        # 状态采用 transient-local，使后来启动的仲裁/监控节点能立即读到最近状态。
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.command_pub = self.create_publisher(Twist, output_topic, 10)
        self.active_pub = self.create_publisher(
            Bool, str(self.get_parameter("active_topic").value), state_qos
        )
        self.stop_pub = self.create_publisher(
            Bool, str(self.get_parameter("emergency_stop_topic").value), state_qos
        )
        self.mode_pub = self.create_publisher(
            String, str(self.get_parameter("speed_mode_topic").value), state_qos
        )
        # joy_node 通常使用 SensorDataQoS；这里采用相同 profile，兼容有线和无线驱动。
        self.create_subscription(
            Joy, input_topic, self.joy_callback, qos_profile_sensor_data
        )
        self.create_timer(1.0 / publish_rate, self.publish_command)
        self.get_logger().info(
            f"Xbox teleop: {input_topic} -> {output_topic}; "
            "center sticks, then hold LB to enable"
        )

    def _index_parameter(self, name: str, fallback: int) -> int:
        """读取非负下标；错误配置回退到已知 Xbox 默认值。"""
        value = int(self.get_parameter(name).value)
        return value if value >= 0 else fallback

    def _positive_parameter(self, name: str, fallback: float) -> float:
        """读取有限正数参数，拒绝 NaN/Inf、零和负速度/超时。"""
        value = float(self.get_parameter(name).value)
        return value if isfinite(value) and value > 0.0 else fallback

    def _direction_parameter(self, name: str) -> float:
        """把任意有限正/负配置归一为 +1/-1；零和非法值按 +1 处理。"""
        value = float(self.get_parameter(name).value)
        return -1.0 if isfinite(value) and value < 0.0 else 1.0

    def _bounded_parameter(
        self, name: str, fallback: float, lower: float, upper: float
    ) -> float:
        """读取闭区间内的有限参数，非法值回退到保守默认值。"""
        value = float(self.get_parameter(name).value)
        return value if isfinite(value) and lower <= value <= upper else fallback

    def joy_callback(self, msg: Joy) -> None:
        """更新控制状态；B 急停在回调中立即发布一次零速度，缩短停车延迟。"""
        now = self.get_clock().now()
        # 回调可能恰好先于周期发布器运行，因此这里也检查消息间隔，确保断流重连
        # 无论发生在定时器的哪个相位，都不会沿用断流前的 armed 状态。
        if (
            self.last_joy_time is not None
            and (now - self.last_joy_time).nanoseconds / 1e9 > self.joy_timeout
        ):
            self.controller.disarm_for_timeout()
        self.latest_result = self.controller.update(msg.axes, msg.buttons)
        self.last_joy_time = now
        if self.latest_result.event:
            self.get_logger().info(
                f"Xbox event={self.latest_result.event}, "
                f"speed_mode={self.latest_result.speed_mode}"
            )
        if self.latest_result.emergency_stop:
            self.command_pub.publish(zero_twist())

    def publish_command(self) -> None:
        """固定频率发布命令；无手柄或超过 timeout 未更新时强制发布零速度。"""
        now = self.get_clock().now()
        age = (
            float("inf")
            if self.last_joy_time is None
            else (now - self.last_joy_time).nanoseconds / 1e9
        )
        fresh = age <= self.joy_timeout
        if not fresh:
            # 重复调用是幂等的。保留 previous_buttons，才能识别断流前 LB 是否仍按下；
            # rearm_requires_release 会保证重连后必须先收到一次松开帧。
            self.controller.disarm_for_timeout()
        command = self.latest_result.twist if fresh else zero_twist()
        active = self.latest_result.active and fresh
        self.command_pub.publish(command if active else zero_twist())
        self.active_pub.publish(Bool(data=active))
        self.stop_pub.publish(Bool(data=self.latest_result.emergency_stop))
        self.mode_pub.publish(String(data=self.latest_result.speed_mode))


def main(args=None):
    """启动 Xbox 手柄速度适配节点。"""
    rclpy.init(args=args)
    node = XboxTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 退出前尽力发布一次零速度。launch/SIGINT 可能已经先关闭 ROS context，因此必须
        # 检查 rclpy.ok() 并容忍发布竞态；底盘仍必须有自己的超时看门狗。
        try:
            if rclpy.ok():
                node.command_pub.publish(zero_twist())
        except RuntimeError:
            # rclpy 的底层 RCLError 继承 RuntimeError；这里只容忍 context 关闭竞态。
            pass
        finally:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

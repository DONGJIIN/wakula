"""汇总标准机器人状态并发布统一的软件停车互锁。

该节点可以在硬件型号未确定时先冻结上层合同：未来 IMU、关节驱动、电池管理器和急停
设备只需发布 ROS 标准消息。默认不强制要求尚不存在的硬件话题，但一旦某来源出现，
其非法数据或断流会触发停车；真机阶段应把对应 ``require_*`` 参数全部设为 true。
"""

import math
import signal
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from quadruped_interfaces.msg import HardwareStatus


@dataclass(frozen=True)
class SafetyLimits:
    """与具体硬件解耦的初始安全阈值；真机测试后必须重新标定。"""

    max_roll: float = 0.70
    max_pitch: float = 0.70
    minimum_battery_voltage: float = 0.0
    sensor_timeout: float = 1.0
    require_imu: bool = False
    require_joint_states: bool = False
    require_battery: bool = False
    max_abs_joint_position: float = 6.4
    require_hardware_status: bool = False


def quaternion_to_roll_pitch(x: float, y: float, z: float, w: float):
    """将单位四元数转换为 roll/pitch；退化或非有限输入返回 ``None``。"""
    values = (x, y, z, w)
    norm = math.sqrt(sum(value * value for value in values))
    if not all(math.isfinite(value) for value in values) or norm < 1e-9:
        return None
    x, y, z, w = (value / norm for value in values)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    return roll, pitch


def joint_state_is_valid(names, position, velocity, effort) -> bool:
    """Validate required kinematics while accepting unavailable effort as NaN.

    ``joint_state_broadcaster`` legitimately publishes an all-NaN effort array
    when a position-only backend has no torque sensor.  A mixed finite/NaN
    effort array remains invalid because it indicates a partial data failure.
    """
    count = len(names)
    if not names or len(position) != count:
        return False
    if velocity and len(velocity) != count:
        return False
    if effort and len(effort) != count:
        return False
    if not all(math.isfinite(float(value)) for value in position):
        return False
    if velocity and not all(math.isfinite(float(value)) for value in velocity):
        return False
    if effort:
        finite = tuple(math.isfinite(float(value)) for value in effort)
        all_nan = all(math.isnan(float(value)) for value in effort)
        if not all(finite) and not all_nan:
            return False
    return True


def joint_positions_within_limit(position, maximum_absolute: float) -> bool:
    """通用末级关节保护；精确的逐关节限位仍由 ros2_control 配置负责。"""
    return (
        math.isfinite(maximum_absolute)
        and maximum_absolute > 0.0
        and all(
            math.isfinite(float(value)) and abs(float(value)) <= maximum_absolute
            for value in position
        )
    )


def evaluate_safety(
    emergency_stop: bool,
    orientation: Optional[Tuple[float, float]],
    joint_values_valid: Optional[bool],
    battery_voltage: Optional[float],
    source_ages: Sequence[Optional[float]],
    limits: SafetyLimits,
) -> Tuple[bool, Tuple[str, ...]]:
    """纯函数评估安全状态，返回 ``(是否停车, 原因列表)``。

    ``source_ages`` 顺序固定为 IMU、JointState、BatteryState。``None`` 表示从未收到；
    可选来源从未出现不停车，但出现后超时仍停车，防止运行中拔线被误认为“未安装”。
    """
    reasons = []
    if emergency_stop:
        reasons.append("emergency_stop")

    if orientation is not None:
        roll, pitch = orientation
        if not math.isfinite(roll) or not math.isfinite(pitch):
            reasons.append("invalid_imu")
        elif abs(roll) > limits.max_roll or abs(pitch) > limits.max_pitch:
            reasons.append("attitude_limit")
    elif limits.require_imu:
        reasons.append("missing_imu")

    if joint_values_valid is False:
        reasons.append("invalid_joint_state")
    elif joint_values_valid is None and limits.require_joint_states:
        reasons.append("missing_joint_states")

    if battery_voltage is not None:
        if not math.isfinite(battery_voltage):
            reasons.append("invalid_battery")
        elif battery_voltage < limits.minimum_battery_voltage:
            reasons.append("low_battery")
    elif limits.require_battery:
        reasons.append("missing_battery")

    required = (
        limits.require_imu,
        limits.require_joint_states,
        limits.require_battery,
    )
    stale_names = ("stale_imu", "stale_joint_states", "stale_battery")
    for age, is_required, name in zip(source_ages, required, stale_names):
        if age is not None and age > limits.sensor_timeout:
            reasons.append(name)
        elif age is None and is_required:
            # missing_* 已提供更准确原因，不重复添加 stale_*。
            continue
    return bool(reasons), tuple(dict.fromkeys(reasons))


class SystemSafetySupervisor(Node):
    """把急停、姿态、关节、电池和心跳统一为 ``/safety/stop``。"""

    def __init__(self):
        super().__init__("system_safety_supervisor")
        self.declare_parameter("max_roll", 0.70)
        self.declare_parameter("max_pitch", 0.70)
        self.declare_parameter("minimum_battery_voltage", 0.0)
        self.declare_parameter("sensor_timeout", 1.0)
        self.declare_parameter("require_imu", False)
        self.declare_parameter("require_joint_states", False)
        self.declare_parameter("require_battery", False)
        self.declare_parameter("max_abs_joint_position", 6.4)
        self.declare_parameter("require_hardware_status", False)
        self.limits = SafetyLimits(
            max_roll=max(0.05, float(self.get_parameter("max_roll").value)),
            max_pitch=max(0.05, float(self.get_parameter("max_pitch").value)),
            minimum_battery_voltage=max(
                0.0,
                float(self.get_parameter("minimum_battery_voltage").value),
            ),
            sensor_timeout=max(
                0.05, float(self.get_parameter("sensor_timeout").value)
            ),
            require_imu=bool(self.get_parameter("require_imu").value),
            require_joint_states=bool(
                self.get_parameter("require_joint_states").value
            ),
            require_battery=bool(self.get_parameter("require_battery").value),
            max_abs_joint_position=max(
                0.1, float(self.get_parameter("max_abs_joint_position").value)
            ),
            require_hardware_status=bool(
                self.get_parameter("require_hardware_status").value
            ),
        )

        self.emergency_stop = False
        self.orientation = None
        self.joint_values_valid = None
        self.battery_voltage = None
        self.last_times = [None, None, None]
        self.last_state = None
        self.hardware_status_ok = None
        self.last_hardware_time = None

        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.stop_pub = self.create_publisher(Bool, "/safety/stop", safety_qos)
        self.state_pub = self.create_publisher(String, "/safety/state", 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.create_subscription(
            JointState, "/joint_states", self.joint_callback, 10
        )
        self.create_subscription(
            BatteryState, "/battery_state", self.battery_callback, 10
        )
        self.create_subscription(
            Bool, "/safety/emergency_stop", self.estop_callback, 10
        )
        self.create_subscription(
            HardwareStatus, "/hardware/status", self.hardware_callback, 10
        )
        self.create_service(
            SetBool, "/safety/set_emergency_stop", self.estop_service
        )
        self.create_timer(0.05, self.evaluate_callback)
        self.evaluate_callback()
        self.get_logger().info("System safety supervisor ready")

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        self.orientation = quaternion_to_roll_pitch(q.x, q.y, q.z, q.w)
        if self.orientation is None:
            self.orientation = (float("nan"), float("nan"))
        self.last_times[0] = self.get_clock().now()

    def joint_callback(self, msg: JointState) -> None:
        self.joint_values_valid = joint_state_is_valid(
            msg.name, msg.position, msg.velocity, msg.effort
        ) and joint_positions_within_limit(
            msg.position, self.limits.max_abs_joint_position
        )
        self.last_times[1] = self.get_clock().now()

    def battery_callback(self, msg: BatteryState) -> None:
        self.battery_voltage = float(msg.voltage)
        self.last_times[2] = self.get_clock().now()

    def estop_callback(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)

    def hardware_callback(self, msg: HardwareStatus) -> None:
        """硬件 FAULT、断连或非零故障码均禁止运动。"""
        self.hardware_status_ok = (
            msg.state in (HardwareStatus.STANDBY, HardwareStatus.ENABLED)
            and bool(msg.command_ready)
            and int(msg.fault_code) == 0
        )
        self.last_hardware_time = self.get_clock().now()

    def estop_service(self, request, response):
        """提供无需硬件的故障注入入口；真机急停仍应有独立硬件回路。"""
        self.emergency_stop = bool(request.data)
        response.success = True
        response.message = "software emergency stop updated"
        self.evaluate_callback()
        return response

    def evaluate_callback(self) -> None:
        now = self.get_clock().now()
        ages = tuple(
            None if stamp is None else (now - stamp).nanoseconds / 1e9
            for stamp in self.last_times
        )
        stop, reasons = evaluate_safety(
            self.emergency_stop,
            self.orientation,
            self.joint_values_valid,
            self.battery_voltage,
            ages,
            self.limits,
        )
        reasons = list(reasons)
        hardware_age = (
            None
            if self.last_hardware_time is None
            else (now - self.last_hardware_time).nanoseconds / 1e9
        )
        if self.hardware_status_ok is False:
            reasons.append("hardware_fault")
        elif self.hardware_status_ok is None and self.limits.require_hardware_status:
            reasons.append("missing_hardware_status")
        if hardware_age is not None and hardware_age > self.limits.sensor_timeout:
            reasons.append("stale_hardware_status")
        reasons = tuple(dict.fromkeys(reasons))
        stop = bool(reasons)
        state = "STOP:" + ",".join(reasons) if stop else "OK"
        self.stop_pub.publish(Bool(data=stop))
        self.state_pub.publish(String(data=state))
        self._publish_diagnostic(stop, reasons)
        if state != self.last_state:
            # rclpy 要求同一源码调用点不能在运行时改变严重级别，因此分开调用。
            if stop:
                self.get_logger().error(f"Safety state -> {state}")
            else:
                self.get_logger().info(f"Safety state -> {state}")
            self.last_state = state

    def _publish_diagnostic(self, stop: bool, reasons) -> None:
        status = DiagnosticStatus()
        status.name = "quadruped/system_safety"
        status.hardware_id = "hardware_independent_supervisor"
        status.level = DiagnosticStatus.ERROR if stop else DiagnosticStatus.OK
        status.message = "motion inhibited" if stop else "safety checks passed"
        status.values = [
            KeyValue(key="reasons", value=",".join(reasons) or "none"),
            KeyValue(
                key="hardware_inputs_required",
                value=str(
                    self.limits.require_imu
                    or self.limits.require_joint_states
                    or self.limits.require_battery
                ).lower(),
            ),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostic_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SystemSafetySupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

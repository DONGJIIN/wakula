"""运行期监测 LaserScan、里程计、TF 与定位突跳。"""

import math
import signal

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from slam.parameter_validation import (
    HEALTH_PARAMETER_NAMES,
    validate_navigation_health_parameters,
)


def scan_contract_is_valid(
    msg: LaserScan,
    minimum_samples: int = 90,
    minimum_field_of_view: float = math.pi,
) -> bool:
    """验证 LaserScan 的结构合同，而不仅是 ``ranges`` 中有多少有限值。

    SLAM 需要有意义的角度序列和传感器坐标系。驱动配置错误时仍可能持续发布一串
    看似正常的距离值，例如 ``angle_increment=0``、只有十几个采样点、空 frame，或
    声称 360° 但数组长度只覆盖很小角度。只统计有效回波会让这些坏帧通过健康门，
    随后表现为地图重影、旋转漂移或 Nav2 无法清障。

    ``LaserScan`` 的最后一个采样对应 ``angle_min + (N-1)*angle_increment``；允许两个
    角分辨率的误差，以兼容驱动对端点“包含/不包含”的不同实现。
    """
    sample_count = len(msg.ranges)
    numeric = (
        msg.angle_min,
        msg.angle_max,
        msg.angle_increment,
        msg.time_increment,
        msg.scan_time,
        msg.range_min,
        msg.range_max,
    )
    if (
        sample_count < max(2, int(minimum_samples))
        or not str(msg.header.frame_id).strip()
        or not all(math.isfinite(float(value)) for value in numeric)
        or float(msg.range_min) < 0.0
        or float(msg.range_max) <= float(msg.range_min)
        or float(msg.time_increment) < 0.0
        or float(msg.scan_time) < 0.0
    ):
        return False
    increment = abs(float(msg.angle_increment))
    declared_span = abs(float(msg.angle_max) - float(msg.angle_min))
    sampled_span = increment * (sample_count - 1)
    if increment <= 1e-9 or sampled_span < max(0.0, float(minimum_field_of_view)):
        return False
    return abs(declared_span - sampled_span) <= max(0.05, 2.0 * increment)


def scan_is_valid(
    ranges,
    minimum_valid_ratio: float,
    range_min: float = 0.0,
    range_max: float = math.inf,
) -> bool:
    """检查雷达有效回波比例、量程和非法浮点值。

    正 ``Inf`` 表示量程内无障碍，是 LaserScan 的合法观测；NaN、负无穷以及小于厂家
    ``range_min`` 的零/近零回波不能算作有效数据。
    """
    if not ranges:
        return False
    # 绝大多数驱动用 0 表示无效近距离回波；即使厂商错误填写 range_min=0，也不能
    # 让一整帧零值通过有效率检查。正 Inf 仍按 REP-117 语义视为“量程内无障碍”。
    lower = max(1e-6, float(range_min)) if math.isfinite(range_min) else 1e-6
    upper = float(range_max)
    if not math.isfinite(upper) or upper <= lower:
        upper = math.inf
    valid = sum(
        1
        for value in ranges
        if (
            math.isinf(float(value))
            and float(value) > 0.0
            or math.isfinite(float(value))
            and lower <= float(value) <= upper
        )
    )
    return valid / len(ranges) >= minimum_valid_ratio


def source_stamp_is_current(
    seconds: int,
    nanoseconds: int,
    now_seconds: float,
    maximum_age: float,
    future_tolerance: float = 0.10,
) -> bool:
    """验证传感器采样时刻，而非只相信 DDS 回调刚刚到达。"""
    stamp = float(seconds) + float(nanoseconds) * 1e-9
    values = (stamp, now_seconds, maximum_age, future_tolerance)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    age = float(now_seconds) - stamp
    return (
        stamp > 0.0
        and maximum_age > 0.0
        and -max(0.0, future_tolerance) <= age <= maximum_age
    )


def odometry_is_valid(
    msg: Odometry,
    max_xy_covariance: float,
    expected_frame: str = "",
    expected_child_frame: str = "",
) -> bool:
    """拒绝非有限位姿/速度、错误 frame 以及过大的平面协方差。

    里程计数值即使有限，若 Header 写成 ``map`` 或 child 写成传感器 frame，也会与
    ``odom -> base_link`` TF 形成相互矛盾的运动来源。期望 frame 为空时跳过名称检查，
    便于纯函数单测；在线节点默认严格执行标准 ``odom``/``base_link`` 合同。
    """
    values = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.angular.z,
    )
    covariance = (msg.pose.covariance[0], msg.pose.covariance[7])
    quaternion_norm = math.sqrt(
        sum(float(value) ** 2 for value in values[2:6])
    )
    frames_valid = (
        (not expected_frame or msg.header.frame_id == expected_frame)
        and (
            not expected_child_frame
            or msg.child_frame_id == expected_child_frame
        )
    )
    return (
        frames_valid
        and all(math.isfinite(float(value)) for value in values + covariance)
        and 0.90 <= quaternion_norm <= 1.10
        and all(
            0.0 <= float(value) <= max_xy_covariance for value in covariance
        )
    )


def navigation_failures(
    scan_fresh_and_valid: bool,
    odom_fresh_and_valid: bool,
    tf_valid: bool,
    odom_jump: bool,
):
    """返回确定顺序的失效原因，供在线监控与异常场景回归测试共用。"""
    checks = {
        "scan": bool(scan_fresh_and_valid),
        "odom": bool(odom_fresh_and_valid),
        "tf": bool(tf_valid),
        "odom_jump": not bool(odom_jump),
    }
    return checks, tuple(name for name, passed in checks.items() if not passed)


class OdometryJumpFilter:
    """锁存不连续里程计，并在若干连续稳定样本后自动恢复。

    单帧跳变可能发生在定位重置、编码器异常或 TF 树切换时。如果下一帧立刻清除标志，
    10 Hz 健康定时器可能完全看不到该故障；锁存可保证速度门至少经历明确的停止阶段。
    """

    def __init__(self, maximum_jump: float, recovery_samples: int = 3):
        self.maximum_jump = max(0.01, float(maximum_jump))
        self.recovery_samples = max(1, int(recovery_samples))
        self.previous_xy = None
        self.latched = False
        self.stable_samples = 0

    def update(self, x: float, y: float, sample_valid: bool) -> bool:
        """输入一个已校验样本，返回当前锁存状态；非法样本不污染参考位置。"""
        if not sample_valid or not all(math.isfinite(value) for value in (x, y)):
            return self.latched
        current = (float(x), float(y))
        if self.previous_xy is None:
            self.previous_xy = current
            return self.latched
        jumped = math.hypot(
            current[0] - self.previous_xy[0], current[1] - self.previous_xy[1]
        ) > self.maximum_jump
        self.previous_xy = current
        if jumped:
            self.latched = True
            self.stable_samples = 0
        elif self.latched:
            self.stable_samples += 1
            if self.stable_samples >= self.recovery_samples:
                self.latched = False
                self.stable_samples = 0
        return self.latched


class NavigationHealthMonitor(Node):
    """持续发布可锁存的导航健康状态；任何必需输入断流都变为 false。"""

    def __init__(self, **node_kwargs):
        """建立导航输入健康状态和 transient-local 健康话题。

        transient-local 使稍后启动的速度门能立即获得最近状态，而不必在未知状态下等待一个
        完整检测周期；内部默认值仍为 false，满足失效安全原则。
        """
        super().__init__("navigation_health_monitor", **node_kwargs)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("sensor_timeout", 1.0)
        self.declare_parameter("minimum_scan_valid_ratio", 0.60)
        self.declare_parameter("minimum_scan_samples", 90)
        self.declare_parameter("minimum_scan_field_of_view", 3.14)
        self.declare_parameter("expected_odom_frame", "odom")
        self.declare_parameter("max_xy_covariance", 1.0)
        self.declare_parameter("max_odom_jump", 0.75)
        self.declare_parameter("odom_jump_recovery_samples", 3)
        self.declare_parameter("future_stamp_tolerance", 0.10)
        # This node gates autonomous motion.  Reject the raw YAML before clamping values or
        # creating a transient-local health publisher, otherwise a bad configuration can look
        # like a valid safety decision to late subscribers.
        validate_navigation_health_parameters(
            {name: self.get_parameter(name).value for name in HEALTH_PARAMETER_NAMES}
        )
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.timeout = max(0.1, float(self.get_parameter("sensor_timeout").value))
        self.minimum_scan_valid_ratio = min(
            1.0, max(0.0, float(self.get_parameter("minimum_scan_valid_ratio").value))
        )
        self.minimum_scan_samples = max(
            2, int(self.get_parameter("minimum_scan_samples").value)
        )
        self.minimum_scan_fov = max(
            0.0,
            float(self.get_parameter("minimum_scan_field_of_view").value),
        )
        self.expected_odom_frame = str(
            self.get_parameter("expected_odom_frame").value
        )
        self.max_xy_covariance = max(
            0.0, float(self.get_parameter("max_xy_covariance").value)
        )
        self.max_odom_jump = max(0.01, float(self.get_parameter("max_odom_jump").value))
        self.future_stamp_tolerance = max(
            0.0, float(self.get_parameter("future_stamp_tolerance").value)
        )
        self.last_scan_time = None
        self.last_odom_time = None
        self.scan_valid = False
        self.odom_valid = False
        self.odom_jump_filter = OdometryJumpFilter(
            self.max_odom_jump,
            int(self.get_parameter("odom_jump_recovery_samples").value),
        )
        self.odom_jump = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.health_pub = self.create_publisher(Bool, "/navigation/healthy", qos)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.evaluate)

    def scan_callback(self, msg: LaserScan) -> None:
        """缓存激光有效率判定和接收时间。"""
        now = self.get_clock().now()
        stamp_valid = source_stamp_is_current(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            now.nanoseconds * 1e-9,
            self.timeout,
            self.future_stamp_tolerance,
        )
        self.scan_valid = (
            stamp_valid
            and scan_contract_is_valid(
                msg,
                self.minimum_scan_samples,
                self.minimum_scan_fov,
            )
            and scan_is_valid(
                msg.ranges,
                self.minimum_scan_valid_ratio,
                msg.range_min,
                msg.range_max,
            )
        )
        self.last_scan_time = now

    def odom_callback(self, msg: Odometry) -> None:
        """缓存里程计有限性、协方差、跳变判定和接收时间。"""
        now = self.get_clock().now()
        xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        stamp_valid = source_stamp_is_current(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            now.nanoseconds * 1e-9,
            self.timeout,
            self.future_stamp_tolerance,
        )
        numeric_valid = odometry_is_valid(
            msg,
            self.max_xy_covariance,
            self.expected_odom_frame,
            self.base_frame,
        )
        self.odom_valid = stamp_valid and numeric_valid
        # 只有数值、协方差和源时间均有效的样本才能更新跳变参考，避免 NaN 将后续比较
        # 永久污染。跳变锁存独立于当前帧 odom_valid，恢复需连续稳定样本。
        self.odom_jump = self.odom_jump_filter.update(
            xy[0], xy[1], self.odom_valid
        )
        self.last_odom_time = now

    def _fresh(self, stamp) -> bool:
        """判断输入是否未超过配置的健康超时。"""
        return stamp is not None and 0.0 <= (
            self.get_clock().now() - stamp
        ).nanoseconds / 1e9 <= self.timeout

    def evaluate(self) -> None:
        """汇总传感器、TF 和漂移检查并发布健康状态。"""
        tf_valid = self.tf_buffer.can_transform(
            self.global_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.02),
        )
        checks, failed = navigation_failures(
            self.scan_valid and self._fresh(self.last_scan_time),
            self.odom_valid and self._fresh(self.last_odom_time),
            tf_valid,
            self.odom_jump,
        )
        healthy = not failed
        self.health_pub.publish(Bool(data=healthy))
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR,
            name="quadruped/navigation_health",
            hardware_id="navigation_inputs",
            message="healthy" if healthy else "failed: " + ",".join(failed),
            values=[
                KeyValue(key=name, value=str(passed).lower())
                for name, passed in checks.items()
            ],
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)


def main(args=None):
    """启动导航输入健康监控节点。"""
    rclpy.init(args=args)
    node = NavigationHealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

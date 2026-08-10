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
    lower = max(0.0, float(range_min)) if math.isfinite(range_min) else 0.0
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


def odometry_is_valid(msg: Odometry, max_xy_covariance: float) -> bool:
    """拒绝非有限位姿/速度以及过大的平面协方差。"""
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
    return (
        all(math.isfinite(float(value)) for value in values + covariance)
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


class NavigationHealthMonitor(Node):
    """持续发布可锁存的导航健康状态；任何必需输入断流都变为 false。"""

    def __init__(self):
        super().__init__("navigation_health_monitor")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("sensor_timeout", 1.0)
        self.declare_parameter("minimum_scan_valid_ratio", 0.60)
        self.declare_parameter("max_xy_covariance", 1.0)
        self.declare_parameter("max_odom_jump", 0.75)
        self.declare_parameter("future_stamp_tolerance", 0.10)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.timeout = max(0.1, float(self.get_parameter("sensor_timeout").value))
        self.minimum_scan_valid_ratio = min(
            1.0, max(0.0, float(self.get_parameter("minimum_scan_valid_ratio").value))
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
        self.previous_xy = None
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
        self.scan_valid = stamp_valid and scan_is_valid(
            msg.ranges,
            self.minimum_scan_valid_ratio,
            msg.range_min,
            msg.range_max,
        )
        self.last_scan_time = now

    def odom_callback(self, msg: Odometry) -> None:
        """缓存里程计有限性、协方差、跳变判定和接收时间。"""
        now = self.get_clock().now()
        xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        self.odom_jump = self.previous_xy is not None and math.hypot(
            xy[0] - self.previous_xy[0], xy[1] - self.previous_xy[1]
        ) > self.max_odom_jump
        self.previous_xy = xy
        stamp_valid = source_stamp_is_current(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            now.nanoseconds * 1e-9,
            self.timeout,
            self.future_stamp_tolerance,
        )
        self.odom_valid = stamp_valid and odometry_is_valid(
            msg, self.max_xy_covariance
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

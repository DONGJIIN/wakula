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


def scan_is_valid(ranges, minimum_valid_ratio: float) -> bool:
    """检查雷达有效回波比例；Inf 代表无障碍，在 LaserScan 中仍是合法观测。"""
    if not ranges:
        return False
    valid = sum(1 for value in ranges if not math.isnan(float(value)) and value >= 0.0)
    return valid / len(ranges) >= minimum_valid_ratio


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
    return all(math.isfinite(float(value)) for value in values + covariance) and all(
        0.0 <= float(value) <= max_xy_covariance for value in covariance
    )


def navigation_failures(
    scan_fresh_and_valid: bool,
    odom_fresh_and_valid: bool,
    tf_valid: bool,
    odom_jump: bool,
):
    """返回确定顺序的失效原因，供在线监控与故障注入测试共用。"""
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
        self.scan_valid = scan_is_valid(msg.ranges, self.minimum_scan_valid_ratio)
        self.last_scan_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry) -> None:
        xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        self.odom_jump = self.previous_xy is not None and math.hypot(
            xy[0] - self.previous_xy[0], xy[1] - self.previous_xy[1]
        ) > self.max_odom_jump
        self.previous_xy = xy
        self.odom_valid = odometry_is_valid(msg, self.max_xy_covariance)
        self.last_odom_time = self.get_clock().now()

    def _fresh(self, stamp) -> bool:
        return stamp is not None and 0.0 <= (
            self.get_clock().now() - stamp
        ).nanoseconds / 1e9 <= self.timeout

    def evaluate(self) -> None:
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
            values=[KeyValue(key=name, value=str(passed).lower()) for name, passed in checks.items()],
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)


def main(args=None):
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

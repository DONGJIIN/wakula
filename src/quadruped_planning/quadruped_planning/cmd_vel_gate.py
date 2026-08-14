"""Nav2 与底盘之间的最终失效安全速度门。

只有 Nav2 速度命令、地形安全评估和导航健康三条独立心跳都有效且新鲜时才允许非零
输出。最后再用二维雷达做一个极近距离急停检查，然后直接发布标准 ``/cmd_vel``。

该节点不规划路线、不创建新的运动意图，也不会把可越障目标改成绕行目标；雷达检查只在
物体已经进入紧急制动距离时兜底。真正的障碍分类、入口对正与越障交接仍分别属于
OpenCV/点云、Nav2 和 ``TraverseObstacle``。它也不替代硬件急停、驱动器看门狗或姿态
保护，只防止 ROS 节点失联或近距离碰撞时沿用最后一条速度。
"""

from math import isfinite, pi

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


def gated_twist(
    source: Twist,
    limit: float,
    command_fresh: bool,
    decision_fresh: bool,
    navigation_healthy: bool = True,
    health_fresh: bool = True,
    external_stop: bool = False,
) -> Twist:
    """仅在命令、地形评估和导航健康状态均有效时缩放 Twist。"""
    output = Twist()
    # 三条独立安全条件任一失效都输出默认构造的零 Twist。
    if (
        not command_fresh
        or not decision_fresh
        or not navigation_healthy
        or not health_fresh
        or external_stop
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


def scan_allows_command(
    scan: LaserScan | None,
    command: Twist,
    stop_distance: float,
    sector_half_angle: float,
) -> bool:
    """判断命令方向的极近距离扇区是否仍有制动空间。

    前进只检查正前方、后退只检查正后方；原地旋转检查整圈，因为任意方向过近的物体都
    可能被机身扫到。无效量程被忽略，但一个有效样本低于阈值就立即拒绝命令。这里使用
    ``LaserScan`` 自带角度定义，不假设固定 720 点或固定雷达型号。
    """
    if scan is None or not scan.ranges:
        return False
    linear = float(command.linear.x)
    angular = float(command.angular.z)
    if abs(linear) < 1e-6 and abs(angular) < 1e-6:
        return True
    threshold = max(0.05, float(stop_distance))
    half_angle = min(pi, max(0.05, float(sector_half_angle)))
    check_all = abs(linear) < 1e-6 and abs(angular) >= 1e-6
    target_angle = 0.0 if linear >= 0.0 else pi
    valid_samples = 0
    for index, measured_range in enumerate(scan.ranges):
        distance = float(measured_range)
        if not isfinite(distance) or distance < float(scan.range_min):
            continue
        angle = float(scan.angle_min) + index * float(scan.angle_increment)
        # 把相对目标方向的角差折叠到 [-pi, pi]，正确处理后方跨越 ±pi 的扇区。
        angle_error = (angle - target_angle + pi) % (2.0 * pi) - pi
        if check_all or abs(angle_error) <= half_angle:
            valid_samples += 1
            if distance <= threshold:
                return False
    return valid_samples > 0


class NavigationSpeedGate(Node):
    """以 20 Hz 重算最终速度，任一安全输入超时即归零。"""

    def __init__(self):
        """建立三路安全心跳，并以固定频率发布经过门控的 Twist。

        这里保存消息接收时间而不是源 Header，因为 Twist/Float32/Bool 没有 Header。每次
        定时回调都重新检查超时，所以发布者退出后不会永久沿用最后一条非零命令。
        """
        super().__init__("navigation_speed_gate")
        self.declare_parameter("input_topic", "/cmd_vel_smoothed")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("assessment_timeout", 0.7)
        self.declare_parameter("navigation_health_timeout", 0.5)
        self.declare_parameter("require_navigation_health", True)
        self.declare_parameter("default_speed_limit", 0.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("emergency_stop_distance", 0.22)
        self.declare_parameter("emergency_sector_half_angle", 0.60)
        self.declare_parameter("require_emergency_scan", True)

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
        self.scan_timeout = max(
            0.05, float(self.get_parameter("scan_timeout").value)
        )
        self.emergency_stop_distance = max(
            0.05, float(self.get_parameter("emergency_stop_distance").value)
        )
        self.emergency_sector_half_angle = min(
            pi,
            max(
                0.05,
                float(self.get_parameter("emergency_sector_half_angle").value),
            ),
        )
        self.require_emergency_scan = bool(
            self.get_parameter("require_emergency_scan").value
        )
        configured_limit = float(self.get_parameter("default_speed_limit").value)
        self.speed_limit = (
            max(0.0, min(1.0, configured_limit))
            if isfinite(configured_limit)
            else 0.0
        )
        self.latest_cmd = Twist()
        self.external_stop = False
        self.last_cmd_time = self.get_clock().now()
        self.last_assessment_time = self.get_clock().now()
        self.navigation_healthy = not self.require_navigation_health
        self.last_health_time = None
        self.latest_scan = None
        self.last_scan_time = None

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        self.create_subscription(
            Float32, "/terrain/speed_limit", self.limit_callback, 10
        )
        self.create_subscription(
            Bool, "/navigation/healthy", self.health_callback, 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            # Gazebo 与常见雷达驱动通常使用传感器 QoS；显式 best-effort 可同时兼容
            # reliable 发布者，避免“话题存在但急停门收不到扫描”的静默故障。
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            "/navigation/autonomy_stop",
            self.autonomy_stop_callback,
            stop_qos,
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

    def autonomy_stop_callback(self, msg: Bool) -> None:
        """锁止核心速度链；最终底盘仲裁器还必须用同一信号覆盖手动输入。"""
        self.external_stop = bool(msg.data)

    def scan_callback(self, msg: LaserScan) -> None:
        """保存最新扫描和接收时刻；具体扇区由当前运动方向决定。"""
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

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
        scan_age = (
            float("inf")
            if self.last_scan_time is None
            else (now - self.last_scan_time).nanoseconds / 1e9
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
            self.external_stop,
        )
        # 导航健康监控负责完整的 scan/odom/TF 校验；这里再保留一个局部、可解释的最终
        # 防撞条件。扫描断流或命令方向 22 cm 内有物体时，只把本周期输出置零。
        if self.require_emergency_scan and (
            scan_age > self.scan_timeout
            or not scan_allows_command(
                self.latest_scan,
                output,
                self.emergency_stop_distance,
                self.emergency_sector_half_angle,
            )
        ):
            output = Twist()
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

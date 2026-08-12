"""在最小传感器与定位 TF 就绪后才启动 Nav2 生命周期节点。"""

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from slam.navigation_health_monitor import (
    odometry_is_valid,
    scan_contract_is_valid,
    scan_is_valid,
    source_stamp_is_current,
)


class Nav2ReadinessMonitor(Node):
    """把 Nav2 激活条件集中到一个节点，避免各服务器在输入缺失时反复报错。"""

    def __init__(self):
        """订阅雷达/里程计心跳并建立生命周期管理服务客户端。"""
        super().__init__("nav2_readiness_monitor")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("sensor_timeout", 1.0)
        self.declare_parameter("future_stamp_tolerance", 0.10)
        self.declare_parameter("minimum_scan_valid_ratio", 0.60)
        self.declare_parameter("minimum_scan_samples", 90)
        self.declare_parameter("minimum_scan_field_of_view", 3.14)
        self.declare_parameter("max_xy_covariance", 1.0)
        self.declare_parameter("expected_odom_frame", "odom")
        self.declare_parameter(
            "lifecycle_service",
            "/lifecycle_manager_navigation/manage_nodes",
        )
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        service_name = str(self.get_parameter("lifecycle_service").value)
        self.sensor_timeout = max(
            0.1, float(self.get_parameter("sensor_timeout").value)
        )
        self.future_stamp_tolerance = max(
            0.0, float(self.get_parameter("future_stamp_tolerance").value)
        )
        self.minimum_scan_valid_ratio = min(
            1.0,
            max(0.0, float(self.get_parameter("minimum_scan_valid_ratio").value)),
        )
        self.minimum_scan_samples = max(
            2, int(self.get_parameter("minimum_scan_samples").value)
        )
        self.minimum_scan_fov = max(
            0.0, float(self.get_parameter("minimum_scan_field_of_view").value)
        )
        self.max_xy_covariance = max(
            0.0, float(self.get_parameter("max_xy_covariance").value)
        )
        self.expected_odom_frame = str(
            self.get_parameter("expected_odom_frame").value
        )

        self.scan_received = False
        self.odom_received = False
        self.scan_valid = False
        self.odom_valid = False
        self.last_scan_time = None
        self.last_odom_time = None
        self.startup_requested = False
        self.startup_complete = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.lifecycle_client = self.create_client(
            ManageLifecycleNodes, service_name
        )
        self.create_timer(0.5, self._check_readiness)
        self.get_logger().info(
            "Nav2 is held inactive until scan, odometry and localization TF "
            "are ready"
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        """记录激光心跳，并用健康监控的同一合同校验该帧是否足以建图。

        “收到 DDS 消息”与“可以激活 Nav2”不是同一件事。空 frame、过期 Header、零角
        增量或绝大多数无效回波都会保持 ``scan_valid=False``，避免生命周期节点在坏
        驱动上启动后持续刷错。
        """
        now = self.get_clock().now()
        self.scan_received = True
        self.last_scan_time = now
        self.scan_valid = (
            source_stamp_is_current(
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                now.nanoseconds * 1e-9,
                self.sensor_timeout,
                self.future_stamp_tolerance,
            )
            and scan_contract_is_valid(
                msg, self.minimum_scan_samples, self.minimum_scan_fov
            )
            and scan_is_valid(
                msg.ranges,
                self.minimum_scan_valid_ratio,
                msg.range_min,
                msg.range_max,
            )
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """记录里程计心跳，同时校验时间、frame、四元数和协方差。"""
        now = self.get_clock().now()
        self.odom_received = True
        self.last_odom_time = now
        self.odom_valid = source_stamp_is_current(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            now.nanoseconds * 1e-9,
            self.sensor_timeout,
            self.future_stamp_tolerance,
        ) and odometry_is_valid(
            msg,
            self.max_xy_covariance,
            self.expected_odom_frame,
            self.base_frame,
        )

    def _sensor_is_fresh(self, stamp) -> bool:
        """按节点 ROS 时钟判断输入是否仍在启动允许窗口内。"""
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return 0.0 <= age <= self.sensor_timeout

    def _check_readiness(self) -> None:
        """仅在 scan、odom 和 map→base_link TF 同时就绪时启动 Nav2。"""
        if self.startup_requested:
            return
        # 不只检查“曾经收到”，还检查传感器正在持续更新。
        scan_ready = self.scan_valid and self._sensor_is_fresh(self.last_scan_time)
        odom_ready = self.odom_valid and self._sensor_is_fresh(self.last_odom_time)
        tf_ready = self.tf_buffer.can_transform(
            self.global_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.05),
        )
        if not (scan_ready and odom_ready and tf_ready):
            missing = []
            if not scan_ready:
                missing.append("fresh scan")
            if not odom_ready:
                missing.append("fresh odom")
            if not tf_ready:
                missing.append(f"{self.global_frame}->{self.base_frame} TF")
            self.get_logger().info(
                "Waiting before Nav2 activation: " + ", ".join(missing),
                throttle_duration_sec=5.0,
            )
            return
        if not self.lifecycle_client.service_is_ready():
            self.get_logger().info(
                "Waiting for Nav2 lifecycle manager service",
                throttle_duration_sec=5.0,
            )
            return
        # 由 lifecycle manager 按固定顺序配置和激活全部 Nav2 节点。
        self.startup_requested = True
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self.lifecycle_client.call_async(request)
        future.add_done_callback(self._startup_response)
        self.get_logger().info("Inputs ready; requesting Nav2 activation")

    def _startup_response(self, future) -> None:
        """处理 lifecycle STARTUP 服务结果，并允许失败后重试。"""
        try:
            response = future.result()
        except Exception as exc:
            self.startup_requested = False
            self.get_logger().error(f"Nav2 startup request failed: {exc}")
            return
        if not response.success:
            self.startup_requested = False
            self.get_logger().error("Nav2 lifecycle manager rejected startup")
            return
        self.startup_complete = True
        self.get_logger().info("Nav2 activated successfully")


def main(args=None):
    """运行传感器与定位就绪监控节点。"""
    rclpy.init(args=args)
    node = Nav2ReadinessMonitor()
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

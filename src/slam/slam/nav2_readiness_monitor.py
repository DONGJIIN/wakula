"""Start Nav2 only after the minimum sensor and TF inputs are ready."""

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


class Nav2ReadinessMonitor(Node):
    """Request Nav2 startup only when localization prerequisites exist."""

    def __init__(self):
        super().__init__("nav2_readiness_monitor")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "lifecycle_service",
            "/lifecycle_manager_navigation/manage_nodes",
        )
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        service_name = str(self.get_parameter("lifecycle_service").value)

        self.scan_received = False
        self.odom_received = False
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

    def _scan_callback(self, _msg: LaserScan) -> None:
        self.scan_received = True

    def _odom_callback(self, _msg: Odometry) -> None:
        self.odom_received = True

    def _check_readiness(self) -> None:
        if self.startup_requested:
            return
        tf_ready = self.tf_buffer.can_transform(
            self.global_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.05),
        )
        if not (self.scan_received and self.odom_received and tf_ready):
            missing = []
            if not self.scan_received:
                missing.append("scan")
            if not self.odom_received:
                missing.append("odom")
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
        self.startup_requested = True
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self.lifecycle_client.call_async(request)
        future.add_done_callback(self._startup_response)
        self.get_logger().info("Inputs ready; requesting Nav2 activation")

    def _startup_response(self, future) -> None:
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
    """Run the sensor and localization readiness monitor."""
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

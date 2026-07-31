"""Extract conservative obstacle metrics from an incoming point cloud."""

from math import isfinite

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray


class TerrainAnalyzer(Node):
    """Publish basic terrain features before a full elevation mapper is added."""

    def __init__(self):
        super().__init__("terrain_analyzer")
        self.declare_parameter("input_topic", "/camera/depth/color/points")
        self.declare_parameter("max_points", 30000)
        self.declare_parameter("warning_height", 0.12)
        self.declare_parameter("critical_height", 0.28)

        topic = self.get_parameter("input_topic").value
        self.max_points = int(self.get_parameter("max_points").value)
        self.warning_height = float(self.get_parameter("warning_height").value)
        self.critical_height = float(self.get_parameter("critical_height").value)

        self.features_pub = self.create_publisher(
            Float32MultiArray, "terrain/features", 10
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.cloud_callback, 10
        )
        self.get_logger().info(f"Terrain analyzer listening on {topic}")

    def cloud_callback(self, msg: PointCloud2) -> None:
        z_values = []
        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            z = float(point[2])
            if isfinite(z):
                z_values.append(z)
            if len(z_values) >= self.max_points:
                break
        if not z_values:
            self._publish_diagnostic(
                DiagnosticStatus.WARN, "No valid points", 0, 0.0
            )
            return

        z_values.sort()
        low = z_values[int(0.05 * (len(z_values) - 1))]
        high = z_values[int(0.95 * (len(z_values) - 1))]
        height_span = high - low
        level = DiagnosticStatus.OK
        message = "Terrain passable"
        if height_span >= self.critical_height:
            level, message = DiagnosticStatus.ERROR, "High obstacle detected"
        elif height_span >= self.warning_height:
            level, message = DiagnosticStatus.WARN, "Step terrain detected"

        features = Float32MultiArray()
        features.data = [float(low), float(high), float(height_span), float(len(z_values))]
        self.features_pub.publish(features)
        self._publish_diagnostic(level, message, len(z_values), height_span)

    def _publish_diagnostic(
        self, level: int, message: str, points: int, height_span: float
    ) -> None:
        status = DiagnosticStatus()
        status.level = level
        status.name = "quadruped/terrain_analyzer"
        status.hardware_id = "terrain_sensor"
        status.message = message
        status.values = [
            KeyValue(key="valid_points", value=str(points)),
            KeyValue(key="height_span_m", value=f"{height_span:.4f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

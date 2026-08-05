"""Conservative local terrain features for the quadruped crossing prototype.

The point cloud is expected to be in a frame whose x axis points forward and
whose z axis points upward (normally base_link or a calibrated camera frame).
This is deliberately a lightweight feature extractor, not a replacement for
an elevation mapper or a footstep planner.
"""

from math import isfinite, sqrt

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray


class TerrainAnalyzer(Node):
    """Estimate obstacle height, slope and roughness in a frontal ROI."""

    def __init__(self):
        super().__init__("terrain_analyzer")
        self.declare_parameter("input_topic", "/camera/depth/color/points")
        self.declare_parameter("max_points", 30000)
        self.declare_parameter("front_x_min", 0.10)
        self.declare_parameter("front_x_max", 1.50)
        self.declare_parameter("lateral_half_width", 0.45)
        self.declare_parameter("ground_percentile", 0.10)
        self.declare_parameter("warning_height", 0.08)
        self.declare_parameter("critical_height", 0.28)
        self.declare_parameter("max_slope", 0.45)
        self.declare_parameter("max_roughness", 0.06)
        self.declare_parameter("min_valid_points", 30)

        self.topic = str(self.get_parameter("input_topic").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.x_min = float(self.get_parameter("front_x_min").value)
        self.x_max = float(self.get_parameter("front_x_max").value)
        self.y_half = float(self.get_parameter("lateral_half_width").value)
        self.ground_percentile = float(self.get_parameter("ground_percentile").value)
        self.warning_height = float(self.get_parameter("warning_height").value)
        self.critical_height = float(self.get_parameter("critical_height").value)
        self.max_slope = float(self.get_parameter("max_slope").value)
        self.max_roughness = float(self.get_parameter("max_roughness").value)
        self.min_points = int(self.get_parameter("min_valid_points").value)

        self.features_pub = self.create_publisher(Float32MultiArray, "/terrain/features", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(PointCloud2, self.topic, self.cloud_callback, 10)
        self.get_logger().info(f"Terrain analyzer listening on {self.topic}")

    def cloud_callback(self, msg: PointCloud2) -> None:
        points = []
        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y, z = (float(point[0]), float(point[1]), float(point[2]))
            if not all(isfinite(v) for v in (x, y, z)):
                continue
            if self.x_min <= x <= self.x_max and abs(y) <= self.y_half:
                points.append((x, y, z))
            if len(points) >= self.max_points:
                break

        if len(points) < self.min_points:
            self._publish_diagnostic(
                DiagnosticStatus.WARN, "Insufficient terrain points", len(points), {}
            )
            return

        z_values = sorted(p[2] for p in points)
        ground = self._percentile(z_values, self.ground_percentile)
        high = self._percentile(z_values, 0.98)
        obstacle_height = max(0.0, high - ground)

        # Fit z = slope*x + intercept. Residual spread approximates terrain roughness.
        mean_x = sum(p[0] for p in points) / len(points)
        mean_z = sum(p[2] for p in points) / len(points)
        denom = sum((p[0] - mean_x) ** 2 for p in points)
        slope = 0.0 if denom < 1e-9 else sum(
            (p[0] - mean_x) * (p[2] - mean_z) for p in points
        ) / denom
        intercept = mean_z - slope * mean_x
        roughness = sqrt(
            sum((p[2] - (slope * p[0] + intercept)) ** 2 for p in points) / len(points)
        )
        traversability = max(
            0.0,
            min(
                1.0,
                1.0
                - obstacle_height / max(self.critical_height, 1e-3)
                - abs(slope) / max(self.max_slope, 1e-3) * 0.35
                - roughness / max(self.max_roughness, 1e-3) * 0.35,
            ),
        )

        # Backward-compatible first four values, followed by richer features.
        features = Float32MultiArray()
        features.data = [
            float(ground),
            float(high),
            float(obstacle_height),
            float(len(points)),
            float(slope),
            float(roughness),
            float(obstacle_height),
            float(max(0.0, self.x_max - self.x_min)),
            float(traversability),
        ]
        self.features_pub.publish(features)

        level = DiagnosticStatus.OK
        message = "Terrain passable"
        if obstacle_height >= self.critical_height or abs(slope) > self.max_slope:
            level, message = DiagnosticStatus.ERROR, "Critical terrain: stop or use footstep planner"
        elif obstacle_height >= self.warning_height or roughness > self.max_roughness:
            level, message = DiagnosticStatus.WARN, "Step or rough terrain detected"
        self._publish_diagnostic(
            level,
            message,
            len(points),
            {"obstacle_height_m": obstacle_height, "slope": slope, "roughness_m": roughness},
        )

    @staticmethod
    def _percentile(values, fraction: float) -> float:
        index = int(max(0.0, min(1.0, fraction)) * (len(values) - 1))
        return values[index]

    def _publish_diagnostic(self, level: int, message: str, points: int, values: dict) -> None:
        status = DiagnosticStatus()
        status.level = level
        status.name = "quadruped/terrain_analyzer"
        status.hardware_id = "terrain_sensor"
        status.message = message
        status.values = [KeyValue(key="valid_points", value=str(points))]
        status.values.extend(
            KeyValue(key=key, value=f"{value:.4f}") for key, value in values.items()
        )
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
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Bounded-rate point-cloud terrain features in the robot base frame."""

from typing import Optional, Tuple

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from quadruped_perception.topic_selection import should_accept_source


TerrainResult = Tuple[list, int]
DEFAULT_POINT_CLOUD_TOPICS = [
    "/camera/depth/points",
    "/camera/depth/color/points",
    "/camera/camera/depth/color/points",
    "/camera/depth_registered/points",
    "/camera/points",
    "/points",
    "/velodyne_points",
    "/ouster/points",
    "/livox/lidar",
]


def filter_roi_points(
    xyz: np.ndarray,
    x_min: float,
    x_max: float,
    y_half: float,
    max_points: int,
) -> np.ndarray:
    """Return finite, bounded and deterministically downsampled ROI points."""
    points = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    # ROI 只保留机器人正前方区域，既减少计算量，也避免腿部点云干扰。
    valid = np.isfinite(points).all(axis=1)
    valid &= (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
    valid &= np.abs(points[:, 1]) <= y_half
    points = points[valid]
    if len(points) > max_points:
        indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
        points = points[indices]
    return points


def compute_terrain_features(
    xyz: np.ndarray,
    x_min: float,
    x_max: float,
    y_half: float,
    max_points: int,
    ground_percentile: float,
    critical_height: float,
    max_slope: float,
    max_roughness: float,
    min_points: int,
) -> Optional[TerrainResult]:
    """Compute lightweight terrain features from an Nx3 base-frame array."""
    points = filter_roi_points(xyz, x_min, x_max, y_half, max_points)
    if len(points) < min_points:
        return None

    x_values = points[:, 0].astype(np.float64)
    z_values = points[:, 2].astype(np.float64)
    # 分位数比最大/最小值更不容易受飞点影响。
    ground = float(np.quantile(z_values, np.clip(ground_percentile, 0.0, 1.0)))
    high = float(np.quantile(z_values, 0.98))
    obstacle_height = max(0.0, high - ground)
    # 最小二乘拟合 z = slope*x + intercept，残差均方根表示粗糙度。
    centered_x = x_values - np.mean(x_values)
    denominator = float(np.dot(centered_x, centered_x))
    slope = (
        0.0
        if denominator < 1e-9
        else float(np.dot(centered_x, z_values - np.mean(z_values)) / denominator)
    )
    intercept = float(np.mean(z_values) - slope * np.mean(x_values))
    residuals = z_values - (slope * x_values + intercept)
    roughness = float(np.sqrt(np.mean(np.square(residuals))))
    height_penalty = obstacle_height / max(critical_height, 1e-3)
    slope_penalty = abs(slope) / max(max_slope, 1e-3) * 0.35
    roughness_penalty = roughness / max(max_roughness, 1e-3) * 0.35
    penalty = height_penalty + slope_penalty + roughness_penalty
    traversability = float(np.clip(1.0 - penalty, 0.0, 1.0))
    return (
        [
            ground,
            high,
            obstacle_height,
            float(len(points)),
            slope,
            roughness,
            obstacle_height,
            max(0.0, x_max - x_min),
            traversability,
        ],
        len(points),
    )


class TerrainAnalyzer(Node):
    """Transform a cloud to base_link and estimate frontal traversability."""

    def __init__(self):
        super().__init__("terrain_analyzer")
        self.declare_parameter("input_topic", "")
        self.declare_parameter(
            "input_topic_candidates", DEFAULT_POINT_CLOUD_TOPICS
        )
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("processing_hz", 10.0)
        self.declare_parameter("transform_timeout", 0.05)
        self.declare_parameter("max_points", 30000)
        self.declare_parameter("nav2_cloud_max_points", 5000)
        self.declare_parameter("front_x_min", 0.10)
        self.declare_parameter("front_x_max", 1.50)
        self.declare_parameter("lateral_half_width", 0.45)
        self.declare_parameter("ground_percentile", 0.10)
        self.declare_parameter("warning_height", 0.08)
        self.declare_parameter("critical_height", 0.28)
        self.declare_parameter("max_slope", 0.45)
        self.declare_parameter("max_roughness", 0.06)
        self.declare_parameter("min_valid_points", 30)
        self.declare_parameter("source_switch_timeout", 2.0)

        override_topic = str(self.get_parameter("input_topic").value)
        candidates = list(self.get_parameter("input_topic_candidates").value)
        self.topics = (
            [override_topic]
            if override_topic
            else list(dict.fromkeys(candidates))
        )
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.transform_timeout = max(
            0.0, float(self.get_parameter("transform_timeout").value)
        )
        self.max_points = max(1, int(self.get_parameter("max_points").value))
        self.nav2_cloud_max_points = max(
            1, int(self.get_parameter("nav2_cloud_max_points").value)
        )
        self.x_min = float(self.get_parameter("front_x_min").value)
        self.x_max = float(self.get_parameter("front_x_max").value)
        self.y_half = max(
            0.0, float(self.get_parameter("lateral_half_width").value)
        )
        self.ground_percentile = float(
            self.get_parameter("ground_percentile").value
        )
        self.warning_height = float(self.get_parameter("warning_height").value)
        self.critical_height = float(self.get_parameter("critical_height").value)
        self.max_slope = float(self.get_parameter("max_slope").value)
        self.max_roughness = float(self.get_parameter("max_roughness").value)
        self.min_points = max(
            1, int(self.get_parameter("min_valid_points").value)
        )
        self.source_switch_timeout = max(
            0.1, float(self.get_parameter("source_switch_timeout").value)
        )
        if self.x_max <= self.x_min:
            self.get_logger().warning(
                "front_x_max must exceed front_x_min; using a 0.10 m ROI"
            )
            self.x_max = self.x_min + 0.10

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_cloud = None
        self.last_processed_stamp = None
        self.active_topic = None
        self.last_active_cloud_time = None
        self.features_pub = self.create_publisher(
            Float32MultiArray, "/terrain/features", 10
        )
        self.obstacle_cloud_pub = self.create_publisher(
            PointCloud2, "/perception/obstacle_points", qos_profile_sensor_data
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._cloud_subscriptions = [
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, source=topic: self.cloud_callback(msg, source),
                qos_profile_sensor_data,
            )
            for topic in self.topics
        ]
        processing_hz = min(
            30.0, max(0.5, float(self.get_parameter("processing_hz").value))
        )
        self.create_timer(1.0 / processing_hz, self.processing_callback)
        self.get_logger().info(
            f"Terrain analyzer: {self.topics} -> {self.target_frame} at "
            f"{processing_hz:.1f} Hz"
        )

    def cloud_callback(self, msg: PointCloud2, source: str) -> None:
        """Keep only the newest cloud to avoid processing backlog."""
        now = self.get_clock().now()
        active_age = (
            float("inf")
            if self.last_active_cloud_time is None
            else (now - self.last_active_cloud_time).nanoseconds / 1e9
        )
        if not should_accept_source(
            self.active_topic,
            source,
            active_age,
            self.source_switch_timeout,
        ):
            return
        # 锁定首个有效点云源；当前源超时后允许其他默认话题接管。
        if source != self.active_topic:
            self.active_topic = source
            self.last_processed_stamp = None
            self.get_logger().info(f"Using point-cloud topic {source}")
        self.last_active_cloud_time = now
        self.latest_cloud = msg

    def processing_callback(self) -> None:
        """Transform and process one unseen cloud."""
        msg = self.latest_cloud
        if msg is None:
            return
        stamp = (msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp
        cloud = self._to_target_frame(msg)
        if cloud is None:
            return
        try:
            xyz = point_cloud2.read_points_numpy(
                cloud, field_names=["x", "y", "z"], skip_nans=True
            )
        except (AssertionError, ValueError) as exc:
            self.get_logger().warning(f"Invalid PointCloud2 layout: {exc}")
            return
        # 同一份已转换点云同时供 Nav2 标障和越障几何计算，避免重复 TF。
        nav2_points = filter_roi_points(
            xyz,
            self.x_min,
            self.x_max,
            self.y_half,
            self.nav2_cloud_max_points,
        )
        self.obstacle_cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(cloud.header, nav2_points.tolist())
        )
        result = compute_terrain_features(
            xyz,
            self.x_min,
            self.x_max,
            self.y_half,
            self.max_points,
            self.ground_percentile,
            self.critical_height,
            self.max_slope,
            self.max_roughness,
            self.min_points,
        )
        if result is None:
            self._publish_diagnostic(
                DiagnosticStatus.WARN,
                "Insufficient terrain points",
                len(nav2_points),
                {},
            )
            return
        features, valid_points = result
        self.features_pub.publish(Float32MultiArray(data=features))
        obstacle_height, slope, roughness = features[2], features[4], features[5]
        level = DiagnosticStatus.OK
        message = "Terrain passable"
        if obstacle_height >= self.critical_height or abs(slope) > self.max_slope:
            level, message = DiagnosticStatus.ERROR, "Critical terrain"
        elif obstacle_height >= self.warning_height or roughness > self.max_roughness:
            level, message = DiagnosticStatus.WARN, "Step or rough terrain detected"
        self._publish_diagnostic(
            level,
            message,
            valid_points,
            {
                "obstacle_height_m": obstacle_height,
                "slope": slope,
                "roughness_m": roughness,
            },
        )

    def _to_target_frame(self, msg: PointCloud2) -> Optional[PointCloud2]:
        if not self.target_frame or msg.header.frame_id == self.target_frame:
            return msg
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.transform_timeout),
            )
            return do_transform_cloud(msg, transform)
        except TransformException as exc:
            self.get_logger().warning(
                f"Waiting for point-cloud TF {msg.header.frame_id} -> "
                f"{self.target_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

    def _publish_diagnostic(
        self, level: int, message: str, points: int, values: dict
    ) -> None:
        status = DiagnosticStatus()
        status.level = level
        status.name = "quadruped/terrain_analyzer"
        status.hardware_id = "terrain_sensor"
        status.message = message
        status.values = [KeyValue(key="valid_points", value=str(points))]
        status.values.extend(
            KeyValue(key=key, value=f"{value:.4f}")
            for key, value in values.items()
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)


def main(args=None):
    """Run the bounded-rate terrain analyzer."""
    rclpy.init(args=args)
    node = TerrainAnalyzer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 避免 launch 转发的第二次 SIGINT 在资源销毁阶段打印 traceback。
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

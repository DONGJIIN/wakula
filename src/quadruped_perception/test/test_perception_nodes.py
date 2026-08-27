"""Small ROS-graph tests for perception synchronization and fail-fast startup.

Pure decision tests cover the mathematical branches.  These tests deliberately instantiate the
real node so subscriptions, timers, message types, parameter overrides, and geometry-only timeout
are also protected against regression without requiring Gazebo or sensor hardware.
"""

import time

import pytest
import rclpy
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from quadruped_interfaces.msg import FusedObstacle, TerrainFeatures, VisionObstacle
from quadruped_perception.perception_fusion import PerceptionFusion
from quadruped_perception.terrain_analyzer import TerrainAnalyzer
from quadruped_perception.vision_obstacle_detector import VisionObstacleDetector


@pytest.fixture
def ros_context():
    """Create one isolated rclpy context for each test and always release DDS resources."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _spin_until(executor, predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def _terrain(stamp, obstacle_type=TerrainFeatures.BAR):
    message = TerrainFeatures()
    message.header.stamp = stamp
    message.header.frame_id = "base_link"
    message.valid = True
    message.obstacle_type = obstacle_type
    message.confidence = 0.8
    message.obstacle_height = 0.32
    message.distance = 1.0
    message.width = 0.8
    message.clearance_height = 0.30
    message.valid_points = 120
    return message


def _vision(stamp):
    message = VisionObstacle()
    message.header.stamp = stamp
    message.header.frame_id = "camera_optical_frame"
    message.obstacle_type = VisionObstacle.HEIGHT_BAR
    message.confidence = 0.85
    message.center_x = 0.5
    message.center_y = 0.45
    message.width = 0.6
    message.height = 0.15
    return message


def test_all_perception_nodes_accept_the_shipped_default_contract(ros_context):
    """Construct camera, cloud, and fusion nodes without Gazebo or physical sensors."""
    nodes = []
    try:
        nodes.extend((VisionObstacleDetector(), TerrainAnalyzer(), PerceptionFusion()))
        assert {node.get_name() for node in nodes} == {
            "vision_obstacle_detector",
            "terrain_analyzer",
            "perception_fusion",
        }
    finally:
        for node in reversed(nodes):
            node.destroy_node()


@pytest.mark.parametrize(
    ("node_factory", "overrides", "message"),
    (
        (
            VisionObstacleDetector,
            [
                Parameter("roi_top_ratio", value=0.8),
                Parameter("roi_bottom_ratio", value=0.2),
            ],
            "roi_top_ratio",
        ),
        (
            TerrainAnalyzer,
            [
                Parameter("front_x_min", value=1.0),
                Parameter("front_x_max", value=0.5),
            ],
            "front ROI",
        ),
    ),
)
def test_camera_and_cloud_nodes_reject_invalid_overrides(
    ros_context, node_factory, overrides, message
):
    """Reject geometrically impossible ROIs before any sensor callback can run."""
    with pytest.raises(ValueError, match=message):
        node_factory(parameter_overrides=overrides)


def test_node_pairs_synchronized_messages_on_the_real_ros_graph(ros_context):
    """Publish typed samples through DDS and require one synchronized fused result."""
    fusion = PerceptionFusion()
    driver = Node("perception_fusion_test_driver")
    terrain_pub = driver.create_publisher(TerrainFeatures, "/terrain/features_stamped", 10)
    vision_pub = driver.create_publisher(VisionObstacle, "/vision/obstacle_stamped", 10)
    outputs = []
    driver.create_subscription(FusedObstacle, "/perception/fused_obstacle", outputs.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(fusion)
    executor.add_node(driver)
    try:
        # Allow DDS discovery before the one-shot samples are published.
        _spin_until(executor, lambda: terrain_pub.get_subscription_count() > 0, 0.6)
        stamp = driver.get_clock().now().to_msg()
        terrain_pub.publish(_terrain(stamp))
        vision_pub.publish(_vision(stamp))
        assert _spin_until(executor, lambda: bool(outputs), 1.0)
        assert outputs[-1].obstacle_type == FusedObstacle.BAR
        assert outputs[-1].geometry_confirmed
        assert outputs[-1].vision_confirmed
        assert outputs[-1].header.frame_id == "base_link"
    finally:
        executor.remove_node(driver)
        executor.remove_node(fusion)
        driver.destroy_node()
        fusion.destroy_node()
        executor.shutdown()


def test_node_publishes_geometry_only_after_camera_timeout(ros_context):
    """Keep metric terrain available when the optional camera stream disappears."""
    fusion = PerceptionFusion(
        parameter_overrides=[
            Parameter("sync_slop", value=0.02),
            Parameter("terrain_only_timeout", value=0.06),
        ]
    )
    driver = Node("perception_fallback_test_driver")
    terrain_pub = driver.create_publisher(TerrainFeatures, "/terrain/features_stamped", 10)
    outputs = []
    driver.create_subscription(FusedObstacle, "/perception/fused_obstacle", outputs.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(fusion)
    executor.add_node(driver)
    try:
        _spin_until(executor, lambda: terrain_pub.get_subscription_count() > 0, 0.6)
        terrain_pub.publish(_terrain(driver.get_clock().now().to_msg(), TerrainFeatures.STEP))
        assert _spin_until(executor, lambda: bool(outputs), 0.8)
        assert outputs[-1].obstacle_type == FusedObstacle.STEP
        assert outputs[-1].geometry_confirmed
        assert not outputs[-1].vision_confirmed
    finally:
        executor.remove_node(driver)
        executor.remove_node(fusion)
        driver.destroy_node()
        fusion.destroy_node()
        executor.shutdown()


def test_node_rejects_invalid_sync_parameters_before_creating_topics(ros_context):
    """Reject a negative synchronization window during node construction."""
    with pytest.raises(ValueError, match="invalid fusion parameters.*sync_slop"):
        PerceptionFusion(parameter_overrides=[Parameter("sync_slop", value=-0.1)])


def test_invalid_preferred_sensors_do_not_block_healthy_fallbacks(ros_context):
    """Only structurally usable camera/cloud messages may acquire active-source ownership."""
    vision = VisionObstacleDetector()
    terrain = TerrainAnalyzer()
    try:
        stamp = vision.get_clock().now().to_msg()

        bad_image = Image()
        bad_image.header.stamp = stamp
        bad_image.header.frame_id = "camera_link"
        bad_image.encoding = "bgr8"
        vision.image_callback(bad_image, "/camera/image_raw")
        assert vision.active_topic is None

        bad_image.width = 4
        bad_image.height = 3
        bad_image.step = 12
        bad_image.data = bytes(36)
        bad_image.encoding = "bad_vendor_encoding"
        vision.image_callback(bad_image, "/camera/image_raw")
        assert vision.active_topic is None

        good_image = Image()
        good_image.header.stamp = stamp
        good_image.header.frame_id = "camera_link"
        good_image.width = 4
        good_image.height = 3
        good_image.encoding = "bgr8"
        good_image.step = 12
        good_image.data = bytes(36)
        vision.image_callback(good_image, "/camera/color/image_raw")
        assert vision.active_topic == "/camera/color/image_raw"
        vision.processing_callback()
        assert vision.last_processed_stamp is not None

        bad_cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id="depth_link"), [(1.0, 0.0, 0.0)]
        )
        bad_cloud.fields = bad_cloud.fields[:2]
        terrain.cloud_callback(bad_cloud, "/camera/depth/points")
        assert terrain.active_topic is None

        good_cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id="depth_link"), [(1.0, 0.0, 0.0)]
        )
        terrain.cloud_callback(good_cloud, "/camera/depth/color/points")
        assert terrain.active_topic == "/camera/depth/color/points"
    finally:
        terrain.destroy_node()
        vision.destroy_node()

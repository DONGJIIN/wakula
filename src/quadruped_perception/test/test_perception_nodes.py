"""Small ROS-graph tests for perception synchronization and fail-fast startup.

Pure decision tests cover the mathematical branches.  These tests deliberately instantiate the
real node so subscriptions, timers, message types, parameter overrides, and geometry-only timeout
are also protected against regression without requiring Gazebo or sensor hardware.
"""

import time
from pathlib import Path

import pytest
import rclpy
import yaml
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from quadruped_interfaces.msg import FusedObstacle, TerrainFeatures, VisionObstacle
from quadruped_perception.perception_fusion import PerceptionFusion
from quadruped_perception.terrain_analyzer import TerrainAnalyzer
from quadruped_perception.terrain_geometry import CLEAR, GeometryEstimate
from quadruped_perception.vision_obstacle_detector import (
    ObstacleEvidence,
    VisionObstacleDetector,
)


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


def _flat_ground_points():
    """Return dense base_link ground echoes that satisfy the online CLEAR coverage contract."""
    # Geometry cells require at least two raw echoes.  A 2.5 cm lattice supplies four samples per
    # 5 cm cell and continuously covers x=0.90..1.70 m across the configured body corridor.
    return [
        (0.901 + x_index * 0.025, -0.249 + y_index * 0.025, -0.30)
        for x_index in range(34)
        for y_index in range(20)
    ]


def _cloud(node, points, frame_id="base_link"):
    """Build a structurally valid PointCloud2 with the node's current nonzero ROS stamp."""
    return point_cloud2.create_cloud_xyz32(
        Header(stamp=node.get_clock().now().to_msg(), frame_id=frame_id),
        points,
    )


def _image(node, encoding="bgr8"):
    """Build a tiny metadata-valid image; 32FC3 intentionally fails conversion to bgr8."""
    message = Image()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "camera_link"
    message.width = 2
    message.height = 2
    message.encoding = encoding
    bytes_per_pixel = 12 if encoding == "32FC3" else 3
    message.step = message.width * bytes_per_pixel
    message.data = bytes(message.step * message.height)
    return message


def test_all_perception_nodes_accept_the_shipped_default_contract(ros_context):
    """Construct all nodes and keep direct-run defaults identical to the shipped YAML."""
    nodes = []
    try:
        nodes.extend((VisionObstacleDetector(), TerrainAnalyzer(), PerceptionFusion()))
        assert {node.get_name() for node in nodes} == {
            "vision_obstacle_detector",
            "terrain_analyzer",
            "perception_fusion",
        }
        config_root = Path(__file__).parents[1] / "config"
        terrain_config = yaml.safe_load(
            (config_root / "terrain.yaml").read_text(encoding="utf-8")
        )
        vision_config = yaml.safe_load(
            (config_root / "vision.yaml").read_text(encoding="utf-8")
        )
        expected = {
            "terrain_analyzer": terrain_config["terrain_analyzer"]["ros__parameters"],
            "vision_obstacle_detector": vision_config["vision_obstacle_detector"][
                "ros__parameters"
            ],
            "perception_fusion": vision_config["perception_fusion"]["ros__parameters"],
        }
        for node in nodes:
            for name, yaml_value in expected[node.get_name()].items():
                runtime_value = node.get_parameter(name).value
                if isinstance(runtime_value, (list, tuple)):
                    runtime_value = list(runtime_value)
                assert runtime_value == yaml_value, name
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


def test_future_dated_preferred_sensors_cannot_lock_out_current_backups(ros_context):
    """A source rejected downstream at +200 ms must never own perception arbitration."""
    terrain = TerrainAnalyzer()
    vision = VisionObstacleDetector()
    try:
        future_cloud = _cloud(terrain, _flat_ground_points())
        future_cloud.header.stamp = (
            rclpy.time.Time.from_msg(future_cloud.header.stamp)
            + Duration(seconds=0.20)
        ).to_msg()
        terrain.cloud_callback(future_cloud, "/camera/depth/points")
        assert terrain.active_topic is None
        current_cloud = _cloud(terrain, _flat_ground_points())
        terrain.cloud_callback(current_cloud, "/camera/depth/color/points")
        assert terrain.active_topic == "/camera/depth/color/points"

        future_image = _image(vision)
        future_image.header.stamp = (
            rclpy.time.Time.from_msg(future_image.header.stamp)
            + Duration(seconds=0.20)
        ).to_msg()
        vision.image_callback(future_image, "/camera/image_raw")
        assert vision.active_topic is None
        current_image = _image(vision)
        vision.image_callback(current_image, "/camera/color/image_raw")
        assert vision.active_topic == "/camera/color/image_raw"
    finally:
        vision.destroy_node()
        terrain.destroy_node()


def test_point_cloud_decode_or_tf_failure_releases_source_for_healthy_backup(ros_context):
    """通过结构初检但缺 TF 的首选源必须冷却，不能继续压住可直接变换的备用源。"""
    terrain = TerrainAnalyzer()
    try:
        bad = point_cloud2.create_cloud_xyz32(
            Header(stamp=terrain.get_clock().now().to_msg(), frame_id="missing_tf_frame"),
            [(1.0, 0.0, -0.3)],
        )
        terrain.cloud_callback(bad, "/camera/depth/points")
        assert terrain.active_topic == "/camera/depth/points"
        terrain.processing_callback()
        assert terrain.active_topic is None
        assert "/camera/depth/points" in terrain.source_cooldown_until

        good = point_cloud2.create_cloud_xyz32(
            Header(stamp=terrain.get_clock().now().to_msg(), frame_id="base_link"),
            _flat_ground_points(),
        )
        terrain.cloud_callback(good, "/camera/depth/color/points")
        assert terrain.active_topic == "/camera/depth/color/points"
        terrain.processing_callback()
        assert terrain.last_active_cloud_time is not None

        # The quarantined high-rate topic cannot immediately reacquire ownership.
        retry = point_cloud2.create_cloud_xyz32(
            Header(stamp=terrain.get_clock().now().to_msg(), frame_id="missing_tf_frame"),
            [(1.0, 0.0, -0.3)],
        )
        terrain.cloud_callback(retry, "/camera/depth/points")
        assert terrain.active_topic == "/camera/depth/color/points"

        # A driver can change layout after initially working, for example after reconnecting.
        # The active source must be released immediately rather than holding ownership for 2 s.
        corrupt_active = point_cloud2.create_cloud_xyz32(
            Header(stamp=terrain.get_clock().now().to_msg(), frame_id="base_link"),
            [(1.0, 0.0, -0.3)],
        )
        corrupt_active.fields = corrupt_active.fields[:2]
        terrain.cloud_callback(corrupt_active, "/camera/depth/color/points")
        assert terrain.active_topic is None
        assert "/camera/depth/color/points" in terrain.source_cooldown_until

        replacement = point_cloud2.create_cloud_xyz32(
            Header(stamp=terrain.get_clock().now().to_msg(), frame_id="base_link"),
            [(1.0, 0.0, -0.3)],
        )
        terrain.cloud_callback(replacement, "/camera/points")
        assert terrain.active_topic == "/camera/points"
    finally:
        terrain.destroy_node()


def test_ground_prior_expires_and_requires_consecutive_translation_conflicts(ros_context):
    """地面先验既不能永久存在，也不能因单帧机身抖动立即被清除。"""
    terrain = TerrainAnalyzer(
        parameter_overrides=[
            Parameter("ground_prior_max_age", value=0.20),
            Parameter("ground_prior_max_consecutive_conflicts", value=2),
        ]
    )
    try:
        observed_at = terrain.get_clock().now()
        clear = GeometryEstimate(
            valid=True,
            obstacle_type=CLEAR,
            ground_height=-0.42,
        )
        terrain._update_ground_prior(clear, observed_at)
        assert terrain.ground_height_prior == pytest.approx(-0.42)

        conflict = GeometryEstimate(ground_reference_conflict=True)
        terrain._update_ground_prior(conflict, observed_at)
        assert terrain.ground_height_prior == pytest.approx(-0.42)
        assert terrain.ground_prior_conflict_count == 1
        terrain._update_ground_prior(conflict, observed_at)
        assert terrain.ground_height_prior is None

        terrain._update_ground_prior(clear, observed_at)
        terrain._expire_ground_prior(observed_at + Duration(seconds=0.21))
        assert terrain.ground_height_prior is None
        assert terrain.ground_height_prior_time is None
    finally:
        terrain.destroy_node()


def test_point_cloud_with_only_nonfinite_returns_cannot_hold_source_health(ros_context):
    """结构合法但全为 NaN 的深度流没有几何信息，必须让有限点备用源接管。"""
    terrain = TerrainAnalyzer()
    try:
        empty_depth = point_cloud2.create_cloud_xyz32(
            Header(stamp=terrain.get_clock().now().to_msg(), frame_id="base_link"),
            [(float("nan"), float("nan"), float("nan"))],
        )
        terrain.cloud_callback(empty_depth, "/camera/depth/points")
        terrain.processing_callback()
        assert terrain.active_topic is None
        assert "/camera/depth/points" in terrain.source_cooldown_until

        finite_backup = point_cloud2.create_cloud_xyz32(
            Header(stamp=terrain.get_clock().now().to_msg(), frame_id="base_link"),
            _flat_ground_points(),
        )
        terrain.cloud_callback(finite_backup, "/camera/depth/color/points")
        terrain.processing_callback()
        assert terrain.active_topic == "/camera/depth/color/points"
        assert terrain.last_active_cloud_time is not None
    finally:
        terrain.destroy_node()


def test_sensor_nodes_reset_source_state_before_arbitrating_after_clock_rewind(
    ros_context,
):
    """旧 epoch 的 owner、watermark、pending、prior 和 cooldown 不能拒绝新首帧。"""
    terrain = TerrainAnalyzer()
    vision = VisionObstacleDetector()
    try:
        terrain_now = terrain.get_clock().now()
        terrain.last_ros_time_ns = terrain_now.nanoseconds + 10_000_000_000
        terrain.active_topic = "/old/cloud"
        terrain.last_active_cloud_time = terrain_now
        terrain.last_candidate_cloud_time = terrain_now
        terrain.last_processed_stamp = ("old", 9, 0)
        terrain.last_received_source_stamp = 9.0
        terrain.latest_cloud = (object(), "/old/cloud")
        terrain.source_cooldown_until["/camera/depth/color/points"] = (
            terrain_now.nanoseconds * 1e-9 + 20.0
        )
        terrain.consecutive_geometry_failures = 4
        terrain.ground_height_prior = -0.30
        terrain.ground_height_prior_time = terrain_now
        terrain.ground_prior_conflict_count = 2

        new_cloud = _cloud(terrain, [(1.0, 0.0, -0.30)])
        terrain.cloud_callback(new_cloud, "/camera/depth/color/points")
        assert terrain.active_topic == "/camera/depth/color/points"
        assert terrain.latest_cloud == (new_cloud, "/camera/depth/color/points")
        assert not terrain.source_cooldown_until
        assert terrain.consecutive_geometry_failures == 0
        assert terrain.ground_height_prior is None
        assert terrain.last_active_cloud_time is None

        vision_now = vision.get_clock().now()
        vision.last_ros_time_ns = vision_now.nanoseconds + 10_000_000_000
        vision.active_topic = "/old/image"
        vision.last_active_image_time = vision_now
        vision.last_processed_stamp = ("/old/image", 9, 0)
        vision.last_received_source_stamp = 9.0
        vision.latest_frame = (object(), "/old/image")
        vision.evidence_history.append(object())
        vision.geometry_label = "OLD DEPTH LABEL"
        vision.last_geometry_time = vision_now
        vision.source_cooldown_until["/camera/color/image_raw"] = (
            vision_now.nanoseconds * 1e-9 + 20.0
        )

        new_image = _image(vision)
        vision.image_callback(new_image, "/camera/color/image_raw")
        assert vision.active_topic == "/camera/color/image_raw"
        assert vision.latest_frame == (new_image, "/camera/color/image_raw")
        assert not vision.source_cooldown_until
        assert not vision.evidence_history
        assert vision.geometry_label == "WAITING FOR DEPTH"
        assert vision.last_active_image_time is not None
    finally:
        vision.destroy_node()
        terrain.destroy_node()


def test_image_conversion_failure_cools_source_and_allows_backup_or_retry(ros_context):
    """坏像素编码不能高频重抢；冷却后单相机可重试，期间健康备用源可接管。"""
    vision = VisionObstacleDetector(
        parameter_overrides=[Parameter("source_failure_cooldown", value=0.10)]
    )
    bad_source = "/camera/image_raw"
    backup_source = "/camera/color/image_raw"
    try:
        bad = _image(vision, "32FC3")
        vision.image_callback(bad, bad_source)
        assert vision.active_topic == bad_source
        vision.processing_callback()
        assert vision.active_topic is None
        assert bad_source in vision.source_cooldown_until

        # The same high-rate broken topic remains quarantined instead of reacquiring immediately.
        vision.image_callback(_image(vision, "32FC3"), bad_source)
        assert vision.active_topic is None

        # A one-camera installation retries after cooldown; a second failure starts a new cooldown.
        vision.source_cooldown_until[bad_source] = (
            vision.get_clock().now().nanoseconds * 1e-9 - 0.01
        )
        vision.image_callback(_image(vision, "32FC3"), bad_source)
        assert vision.active_topic == bad_source
        vision.processing_callback()
        assert vision.active_topic is None

        vision.image_callback(_image(vision), backup_source)
        assert vision.active_topic == backup_source
        vision.processing_callback()
        assert vision.last_processed_stamp is not None
    finally:
        vision.destroy_node()


def test_camera_header_gap_resets_history_even_when_buffered_callbacks_are_bursting(
    ros_context,
):
    """Exposure-time discontinuity, not just receive silence, invalidates visual votes."""
    vision = VisionObstacleDetector()
    topic = "/camera/image_raw"
    try:
        newest = _image(vision)
        newest_time = rclpy.time.Time.from_msg(newest.header.stamp)
        buffered_old = _image(vision)
        buffered_old.header.stamp = (
            newest_time - Duration(seconds=1.0)
        ).to_msg()
        vision.image_callback(buffered_old, topic)
        # Model a stable result already accumulated from the old exposure epoch.  The next callback
        # arrives immediately, as happens when DDS/executor drains a buffer after a stall, but its
        # source Header jumps farther than history_reset_timeout (0.75 s).
        vision.evidence_history.extend(
            [ObstacleEvidence("poles", 0.8, 0.5, 0.5, 0.2, 0.4)] * 3
        )
        assert len(vision.evidence_history) == 3
        vision.image_callback(newest, topic)
        assert vision.active_topic == topic
        assert not vision.evidence_history
        assert vision.latest_frame == (newest, topic)
    finally:
        vision.destroy_node()


def test_consecutive_unusable_cloud_geometry_releases_source_without_single_frame_flap(
    ros_context,
):
    """有限 XYZ/TF 仍须看见前向几何；单帧抖动不切源，连续失败才让备用源接管。"""
    terrain = TerrainAnalyzer(
        parameter_overrides=[Parameter("source_geometry_failure_frames", value=2)]
    )
    preferred = "/camera/depth/points"
    backup = "/camera/depth/color/points"
    outside_roi = [(10.0, 0.0, -0.30)] * 40
    try:
        terrain.cloud_callback(_cloud(terrain, outside_roi), preferred)
        terrain.processing_callback()
        assert terrain.active_topic == preferred
        assert terrain.consecutive_geometry_failures == 1

        # One usable frame breaks the sequence, proving a single dropout cannot trigger failover.
        terrain.cloud_callback(_cloud(terrain, _flat_ground_points()), preferred)
        terrain.processing_callback()
        assert terrain.active_topic == preferred
        assert terrain.consecutive_geometry_failures == 0
        assert terrain.last_active_cloud_time is not None

        terrain.cloud_callback(_cloud(terrain, outside_roi), preferred)
        terrain.processing_callback()
        assert terrain.active_topic == preferred
        assert terrain.consecutive_geometry_failures == 1
        terrain.cloud_callback(_cloud(terrain, outside_roi), preferred)
        terrain.processing_callback()
        assert terrain.active_topic is None
        assert preferred in terrain.source_cooldown_until

        terrain.cloud_callback(_cloud(terrain, _flat_ground_points()), backup)
        terrain.processing_callback()
        assert terrain.active_topic == backup
        assert terrain.last_active_cloud_time is not None
        assert terrain.consecutive_geometry_failures == 0
    finally:
        terrain.destroy_node()


def test_fusion_clock_rewind_clears_both_sensor_epochs(ros_context):
    """回拨后旧图像、旧点云和接收时间都不能与新 epoch 的首帧配对。"""
    fusion = PerceptionFusion()
    try:
        cloud = TerrainFeatures()
        camera = VisionObstacle()
        fusion.terrain_queue.append(cloud)
        fusion.vision_queue.append(camera)
        fusion.terrain_receive_times[id(cloud)] = 10.0
        fusion.last_clock_seconds = 10.0

        assert fusion._observe_ros_clock(9.0) == 9.0
        assert not fusion.terrain_queue
        assert not fusion.vision_queue
        assert not fusion.terrain_receive_times
    finally:
        fusion.destroy_node()


def test_fusion_rejects_zero_stamped_typed_inputs_before_queueing(ros_context):
    """外部替换感知节点时，零时间戳数据不能进入同步或纯点云 fallback 队列。"""
    fusion = PerceptionFusion()
    try:
        fusion.terrain_callback(TerrainFeatures())
        fusion.vision_callback(VisionObstacle())
        assert not fusion.terrain_queue
        assert not fusion.vision_queue
        assert not fusion.terrain_receive_times
    finally:
        fusion.destroy_node()

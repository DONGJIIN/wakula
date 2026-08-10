"""Tests for timestamped conservative camera/cloud fusion."""

from quadruped_interfaces.msg import FusedObstacle, TerrainFeatures, VisionObstacle
from quadruped_perception.perception_fusion import (
    find_synchronized_pair,
    fuse_observations,
)


def terrain(obstacle_type=TerrainFeatures.STEP):
    """构造一条有效点云几何测试消息。"""
    msg = TerrainFeatures()
    msg.valid = True
    msg.obstacle_type = obstacle_type
    msg.confidence = 0.70
    msg.obstacle_height = 0.16
    msg.distance = 0.6
    msg.roughness = 0.02
    msg.width = 0.4
    msg.valid_points = 120
    return msg


def test_matching_vision_boosts_confidence_but_not_geometry_requirement():
    """同类视觉提高置信度但不能替代几何有效位。"""
    cloud = terrain(TerrainFeatures.WALL)
    camera = VisionObstacle(obstacle_type=VisionObstacle.WALL, confidence=0.8)
    result = fuse_observations(cloud, camera, 0.03, 0.55)
    assert result.obstacle_type == FusedObstacle.WALL
    assert result.geometry_confirmed
    assert result.confidence > cloud.confidence
    assert result.vision_confirmed
    assert result.valid_points == cloud.valid_points
    assert result.roughness == cloud.roughness


def test_visual_bar_only_refines_existing_positive_geometry():
    """横杆细分类必须同时满足点云离地净空。"""
    camera = VisionObstacle(
        obstacle_type=VisionObstacle.HEIGHT_BAR,
        confidence=0.9,
    )
    compatible = terrain()
    compatible.clearance_height = 0.12
    result = fuse_observations(compatible, camera, 0.02, 0.55)
    assert result.obstacle_type == FusedObstacle.BAR

    ordinary_step = terrain()
    result = fuse_observations(ordinary_step, camera, 0.02, 0.55)
    assert result.obstacle_type == FusedObstacle.STEP
    assert result.confidence < ordinary_step.confidence

    invalid = TerrainFeatures(valid=False, obstacle_type=TerrainFeatures.UNKNOWN)
    result = fuse_observations(invalid, camera, 0.02, 0.55)
    assert result.obstacle_type == FusedObstacle.UNKNOWN
    assert not result.geometry_confirmed


def _stamp(msg, seconds, nanoseconds=0):
    """为测试消息写入 ROS Header 时间戳。"""
    msg.header.stamp.sec = seconds
    msg.header.stamp.nanosec = nanoseconds
    return msg


def test_pairing_handles_out_of_order_callbacks_and_uses_each_stamp():
    """A valid older pair is retained even when the newest cloud has no matching image."""
    old_cloud = _stamp(terrain(), 10)
    new_cloud = _stamp(terrain(), 11)
    old_image = _stamp(VisionObstacle(), 10, 40_000_000)
    too_new_image = _stamp(VisionObstacle(), 11, 250_000_000)
    pair = find_synchronized_pair(
        [old_cloud, new_cloud], [old_image, too_new_image], 0.10
    )
    assert pair is not None
    assert pair[0] is old_cloud
    assert pair[1] is old_image
    assert abs(pair[2] - 0.04) < 1e-6


def test_pairing_rejects_zero_or_out_of_window_timestamps():
    """零时间戳和超出同步窗口的消息不能融合。"""
    assert find_synchronized_pair([terrain()], [VisionObstacle()], 0.10) is None
    cloud = _stamp(terrain(), 20)
    image = _stamp(VisionObstacle(), 21)
    assert find_synchronized_pair([cloud], [image], 0.10) is None

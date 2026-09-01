"""Tests for timestamped conservative camera/cloud fusion."""

from pathlib import Path

import yaml

from quadruped_interfaces.msg import FusedObstacle, TerrainFeatures, VisionObstacle
from quadruped_perception.perception_fusion import (
    find_synchronized_pair,
    fuse_observations,
    ros_clock_moved_backward,
    terrain_fallback_ready,
    terrain_observation_valid,
    vision_observation_valid,
    vision_overlaps_forward_corridor,
)


def terrain(obstacle_type=TerrainFeatures.STEP):
    """构造一条有效点云几何测试消息。"""
    msg = TerrainFeatures()
    msg.valid = True
    msg.obstacle_type = obstacle_type
    msg.confidence = 0.70
    msg.obstacle_height = 0.16
    msg.distance = 0.6
    msg.lateral_offset = 0.12
    msg.roughness = 0.02
    msg.width = 0.4
    msg.valid_points = 120
    return msg


def vision(obstacle_type=VisionObstacle.WALL, confidence=0.8):
    """构造带合法归一化框的视觉观测。"""
    return VisionObstacle(
        obstacle_type=obstacle_type,
        confidence=confidence,
        center_x=0.5,
        center_y=0.5,
        width=0.3,
        height=0.4,
    )


def test_matching_vision_boosts_confidence_but_not_geometry_requirement():
    """同类视觉提高置信度但不能替代几何有效位。"""
    cloud = terrain(TerrainFeatures.WALL)
    camera = vision()
    result = fuse_observations(cloud, camera, 0.03, 0.55)
    assert result.obstacle_type == FusedObstacle.WALL
    assert result.geometry_confirmed
    assert result.confidence > cloud.confidence
    assert result.vision_confirmed
    assert result.valid_points == cloud.valid_points
    assert result.roughness == cloud.roughness
    assert result.lateral_offset == cloud.lateral_offset


def test_uncalibrated_visual_bar_or_pole_cannot_refine_step_geometry():
    """No CameraInfo/projection means BAR/POLE pixels cannot rewrite a STEP.

    Clearance and width checks alone are deliberately insufficient: a synchronized frame may
    contain a forward step plus a different bar/pole inside the broad 2-D corridor. Metric
    terrain classification remains authoritative until calibrated point projection exists.
    """
    camera = vision(VisionObstacle.HEIGHT_BAR, 0.9)
    ordinary_step = terrain()
    ordinary_step.clearance_height = 0.20
    result = fuse_observations(ordinary_step, camera, 0.02, 0.55)
    assert result.obstacle_type == FusedObstacle.STEP
    assert result.confidence == ordinary_step.confidence
    assert not result.vision_confirmed

    pole_camera = vision(VisionObstacle.POLES, 0.9)
    narrow_step = terrain()
    narrow_step.width = 0.10
    result = fuse_observations(narrow_step, pole_camera, 0.02, 0.55)
    assert result.obstacle_type == FusedObstacle.STEP
    assert result.confidence == narrow_step.confidence
    assert not result.vision_confirmed


def test_same_class_bar_and_pole_visual_evidence_is_confirmed_normally():
    """Exact class agreement boosts confidence without changing metric geometry."""
    for terrain_type, vision_type, fused_type in (
        (TerrainFeatures.BAR, VisionObstacle.HEIGHT_BAR, FusedObstacle.BAR),
        (TerrainFeatures.POLE, VisionObstacle.POLES, FusedObstacle.POLE),
    ):
        cloud = terrain(terrain_type)
        result = fuse_observations(
            cloud, vision(vision_type, 0.9), 0.02, 0.55
        )
        assert result.obstacle_type == fused_type
        assert result.geometry_confirmed
        assert result.vision_confirmed
        assert result.confidence > cloud.confidence


def test_invalid_geometry_cannot_be_replaced_by_visual_classification():
    """Even a strong visual candidate cannot provide missing authoritative geometry."""
    camera = vision(VisionObstacle.HEIGHT_BAR, 0.9)
    invalid = TerrainFeatures(valid=False, obstacle_type=TerrainFeatures.UNKNOWN)
    result = fuse_observations(invalid, camera, 0.02, 0.55)
    assert result.obstacle_type == FusedObstacle.UNKNOWN
    assert not result.geometry_confirmed
    assert not result.vision_confirmed


def test_visual_target_without_compatible_geometry_is_not_confirmed():
    """同步视觉框不能把 CLEAR 点云冒充为已经完成类别一致性复核。"""
    cloud = terrain(TerrainFeatures.CLEAR)
    camera = vision(VisionObstacle.WALL, 0.9)
    result = fuse_observations(cloud, camera, 0.02, 0.55)
    assert result.obstacle_type == FusedObstacle.CLEAR
    assert result.geometry_confirmed
    assert not result.vision_confirmed
    assert result.confidence == cloud.confidence


def test_visual_conflict_cannot_invalidate_low_confidence_metric_geometry():
    """辅助视觉杂物不得把已达下游门限的点云置信度压回无效区。"""
    cloud = terrain(TerrainFeatures.STEP)
    cloud.confidence = 0.30
    camera = vision(VisionObstacle.WALL, 0.90)
    result = fuse_observations(cloud, camera, 0.02, 0.55)
    assert result.geometry_confirmed
    assert not result.vision_confirmed
    assert result.obstacle_type == FusedObstacle.STEP
    assert result.confidence == cloud.confidence


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


def test_ros_clock_rewind_detection_is_epoch_based_and_finite():
    """bag/Gazebo 回拨必须开始新融合 epoch，正常停钟或前进不应误清队列。"""
    assert not ros_clock_moved_backward(None, 10.0)
    assert not ros_clock_moved_backward(10.0, 10.0)
    assert not ros_clock_moved_backward(10.0, 10.1)
    assert ros_clock_moved_backward(10.0, 9.9)
    assert ros_clock_moved_backward(10.0, float("nan"))


def test_fusion_rejects_invalid_numeric_fields_and_visual_boxes():
    """生产者有效位不能掩盖 NaN、未知类别或退化视觉框。"""
    cloud = terrain()
    camera = vision()
    assert terrain_observation_valid(cloud)
    assert vision_observation_valid(camera, 0.55)

    cloud.slope_roll = float("nan")
    result = fuse_observations(cloud, camera, 0.01, 0.55)
    assert not result.geometry_confirmed
    assert result.obstacle_type == FusedObstacle.UNKNOWN
    assert result.slope_roll == 0.0

    cloud = terrain()
    camera.width = 0.0
    result = fuse_observations(cloud, camera, 0.01, 0.55)
    assert result.geometry_confirmed
    assert not result.vision_confirmed
    camera = vision()
    camera.center_x = 0.95
    assert not vision_observation_valid(camera, 0.55)
    cloud.obstacle_height = -0.1
    result = fuse_observations(cloud, camera, 0.01, 0.55)
    assert not result.geometry_confirmed
    assert result.obstacle_height == 0.0


def test_fusion_rejects_nan_visual_confidence():
    """NaN 比较不能意外绕过视觉置信度阈值。"""
    camera = vision(confidence=float("nan"))
    assert not vision_observation_valid(camera, 0.55)
    result = fuse_observations(terrain(), camera, 0.01, 0.55)
    assert not result.vision_confirmed


def test_fusion_rejects_visual_object_outside_forward_cloud_corridor():
    """同一时刻的画面边缘目标不能细分正前方的点云障碍。"""
    camera = vision(VisionObstacle.HEIGHT_BAR, 0.9)
    camera.center_x = 0.08
    camera.width = 0.10
    assert vision_observation_valid(camera, 0.55)
    assert not vision_overlaps_forward_corridor(camera, 0.15)
    cloud = terrain()
    cloud.clearance_height = 0.15
    result = fuse_observations(cloud, camera, 0.02, 0.55, 0.15)
    assert result.obstacle_type == FusedObstacle.STEP
    assert not result.vision_confirmed


def test_camera_dropout_falls_back_to_geometry_without_visual_confirmation():
    """相机断流不能冻结点云安全链，超时后应保留几何并明确无视觉确认。"""
    assert not terrain_fallback_ready(10.0, 10.20, 0.25)
    assert terrain_fallback_ready(10.0, 10.25, 0.25)
    assert not terrain_fallback_ready(10.0, 9.0, 0.25)
    cloud = terrain(TerrainFeatures.WALL)
    result = fuse_observations(cloud, None, 0.0, 0.55)
    assert result.geometry_confirmed
    assert not result.vision_confirmed
    assert result.obstacle_type == FusedObstacle.WALL
    assert result.confidence == cloud.confidence


def test_fusion_dropout_policy_is_versioned_in_vision_config():
    """同步容差与纯点云降级等待必须随项目配置提交，不能只依赖源码默认值。"""
    config_path = Path(__file__).parents[1] / "config" / "vision.yaml"
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    parameters = config["perception_fusion"]["ros__parameters"]
    assert parameters["sync_slop"] > 0.0
    assert parameters["terrain_only_timeout"] >= parameters["sync_slop"]

"""Tests for timestamped conservative camera/cloud fusion."""

from quadruped_interfaces.msg import FusedObstacle, TerrainFeatures, VisionObstacle
from quadruped_perception.perception_fusion import fuse_observations


def terrain(obstacle_type=TerrainFeatures.STEP):
    msg = TerrainFeatures()
    msg.valid = True
    msg.obstacle_type = obstacle_type
    msg.confidence = 0.70
    msg.obstacle_height = 0.16
    msg.distance = 0.6
    return msg


def test_matching_vision_boosts_confidence_but_not_geometry_requirement():
    cloud = terrain(TerrainFeatures.WALL)
    camera = VisionObstacle(obstacle_type=VisionObstacle.WALL, confidence=0.8)
    result = fuse_observations(cloud, camera, 0.03, 0.55)
    assert result.obstacle_type == FusedObstacle.WALL
    assert result.geometry_confirmed
    assert result.confidence > cloud.confidence


def test_visual_bar_only_refines_existing_positive_geometry():
    camera = VisionObstacle(
        obstacle_type=VisionObstacle.HEIGHT_BAR,
        confidence=0.9,
    )
    result = fuse_observations(terrain(), camera, 0.02, 0.55)
    assert result.obstacle_type == FusedObstacle.BAR

    invalid = TerrainFeatures(valid=False, obstacle_type=TerrainFeatures.UNKNOWN)
    result = fuse_observations(invalid, camera, 0.02, 0.55)
    assert result.obstacle_type == FusedObstacle.UNKNOWN
    assert not result.geometry_confirmed

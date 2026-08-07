"""Tests for bounded OpenCV and NumPy terrain feature extraction."""

import cv2
import numpy as np

from quadruped_perception.terrain_analyzer import (
    compute_terrain_features,
    DEFAULT_POINT_CLOUD_TOPICS,
)
from quadruped_perception.topic_selection import should_accept_source
from quadruped_perception.vision_obstacle_detector import (
    DEFAULT_IMAGE_TOPICS,
    detect_obstacle_evidence,
    largest_color_feature,
    ObstacleEvidence,
    stabilize_evidence,
)


def test_detects_simple_poles_and_height_bar():
    """Color and shape cues recognize two poles and one horizontal bar."""
    orange = np.zeros((240, 320), dtype=np.uint8)
    blue = np.zeros_like(orange)
    edges = np.zeros_like(orange)
    cv2.rectangle(orange, (60, 60), (80, 210), 255, -1)
    cv2.rectangle(orange, (180, 50), (202, 210), 255, -1)
    evidence = detect_obstacle_evidence(orange, blue, edges, 100.0)
    assert evidence.hint == "poles"
    assert evidence.confidence > 0.7

    orange.fill(0)
    cv2.rectangle(blue, (40, 80), (280, 110), 255, -1)
    evidence = detect_obstacle_evidence(orange, blue, edges, 100.0)
    assert evidence.hint == "height_bar"


def test_grayscale_geometry_and_temporal_confirmation():
    """Edges work without target colors, while one-frame noise is rejected."""
    mask = np.zeros((240, 320), dtype=np.uint8)
    cv2.rectangle(mask, (60, 40), (72, 220), 255, -1)
    cv2.rectangle(mask, (220, 45), (232, 220), 255, -1)
    evidence = detect_obstacle_evidence(
        np.zeros_like(mask), np.zeros_like(mask), mask, 100.0
    )
    assert evidence.hint == "poles"

    history = [evidence, evidence, evidence, ObstacleEvidence(), ObstacleEvidence()]
    stable = stabilize_evidence(history, 3)
    assert stable.hint == "poles"
    assert stable.confidence >= 0.55
    assert stabilize_evidence(history[:2], 3).hint == "none"


def test_color_feature_is_normalized():
    """Bounding-box outputs remain independent of camera resolution."""
    mask = np.zeros((100, 200), dtype=np.uint8)
    cv2.rectangle(mask, (50, 20), (149, 79), 255, -1)
    area, center_x, center_y, width, height = largest_color_feature(mask, 100.0)
    assert area > 5000
    assert abs(center_x - 0.5) < 0.01
    assert abs(center_y - 0.5) < 0.01
    assert 0.49 <= width <= 0.51
    assert 0.59 <= height <= 0.61


def test_numpy_terrain_features_and_minimum_points():
    """Flat sloped ground is measured and sparse clouds are rejected."""
    x_values = np.linspace(0.1, 1.5, 1000, dtype=np.float32)
    y_values = np.linspace(-0.3, 0.3, 1000, dtype=np.float32)
    z_values = 0.1 * x_values
    points = np.column_stack((x_values, y_values, z_values))
    result = compute_terrain_features(
        points, 0.1, 1.5, 0.45, 30000, 0.1, 0.28, 0.45, 0.06, 30
    )
    assert result is not None
    features, count = result
    assert count == 1000
    assert abs(features[4] - 0.1) < 1e-3
    assert features[5] < 1e-4

    sparse = compute_terrain_features(
        points[:10], 0.1, 1.5, 0.45, 30000, 0.1, 0.28, 0.45, 0.06, 30
    )
    assert sparse is None


def test_sensor_topic_selection_locks_and_fails_over():
    """A second default topic is ignored until the active source is stale."""
    assert should_accept_source(None, "/camera/image_raw", 0.0, 2.0)
    assert should_accept_source(
        "/camera/image_raw", "/camera/image_raw", 0.1, 2.0
    )
    assert not should_accept_source(
        "/camera/image_raw", "/image_raw", 0.5, 2.0
    )
    assert should_accept_source(
        "/camera/image_raw", "/image_raw", 2.1, 2.0
    )


def test_common_camera_and_3d_lidar_topics_are_supported():
    """Default candidates cover ROS conventions and common vendor namespaces."""
    assert "/image_raw" in DEFAULT_IMAGE_TOPICS
    assert "/camera/camera/color/image_raw" in DEFAULT_IMAGE_TOPICS
    assert "/camera/depth/points" in DEFAULT_POINT_CLOUD_TOPICS
    assert "/velodyne_points" in DEFAULT_POINT_CLOUD_TOPICS
    assert "/ouster/points" in DEFAULT_POINT_CLOUD_TOPICS
    assert "/livox/lidar" in DEFAULT_POINT_CLOUD_TOPICS

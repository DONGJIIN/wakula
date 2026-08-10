"""Tests for bounded OpenCV and NumPy terrain feature extraction."""

import cv2
import numpy as np
import pytest

from quadruped_perception.terrain_analyzer import (
    compute_terrain_features,
    transform_xyz,
)
from quadruped_perception.terrain_geometry import (
    CLEAR,
    PIT,
    WALL,
    analyze_terrain_geometry,
)
from quadruped_perception.topic_selection import should_accept_source
from quadruped_perception.vision_obstacle_detector import (
    ObstacleEvidence,
    adaptive_canny_thresholds,
    apply_image_quality,
    apply_detection_roi,
    detect_obstacle_evidence,
    enhance_illumination,
    evidence_iou,
    image_quality_score,
    largest_color_feature,
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


def test_pole_pair_rejects_unaligned_vertical_clutter():
    """Two unrelated vertical color regions must not masquerade as a gate."""
    orange = np.zeros((240, 320), dtype=np.uint8)
    empty = np.zeros_like(orange)
    cv2.rectangle(orange, (50, 10), (70, 100), 255, -1)
    cv2.rectangle(orange, (220, 145), (240, 235), 255, -1)
    evidence = detect_obstacle_evidence(orange, empty, empty, 100.0)
    assert evidence.hint != "poles"


def test_temporal_confirmation_rejects_spatially_inconsistent_boxes():
    """Repeated labels at jumping positions are treated as separate objects."""
    history = [
        ObstacleEvidence("poles", 0.8, center_x, 0.5, 0.2, 0.5)
        for center_x in (0.15, 0.50, 0.85)
    ]
    assert stabilize_evidence(history, 3).hint == "none"


def test_temporal_confirmation_requires_majority_and_box_overlap():
    """Old sparse hits and differently sized regions cannot form one stable target."""
    target = ObstacleEvidence("wall", 0.8, 0.5, 0.5, 0.20, 0.40)
    sparse_history = [target, target, target] + [ObstacleEvidence()] * 3
    assert stabilize_evidence(sparse_history, 3, minimum_match_ratio=0.6).hint == "none"

    inconsistent_sizes = [
        ObstacleEvidence("wall", 0.8, 0.5, 0.5, width, 0.40)
        for width in (0.05, 0.20, 0.29)
    ]
    assert evidence_iou(inconsistent_sizes[0], inconsistent_sizes[1]) < 0.3
    assert (
        stabilize_evidence(
            inconsistent_sizes,
            3,
            max_size_jitter=0.25,
            minimum_iou=0.40,
        ).hint
        == "none"
    )


def test_illumination_roi_and_adaptive_edge_helpers():
    """Preprocessing preserves shape, masks borders and returns valid thresholds."""
    image = np.full((80, 120, 3), 25, dtype=np.uint8)
    cv2.rectangle(image, (30, 20), (90, 65), (0, 70, 150), -1)
    enhanced = enhance_illumination(image, 2.0, 4)
    assert enhanced.shape == image.shape
    assert not np.array_equal(enhanced, image)

    mask = np.full((100, 200), 255, dtype=np.uint8)
    roi = apply_detection_roi(mask, 0.10, 0.90, 0.10)
    assert cv2.countNonZero(roi[:10]) == 0
    assert cv2.countNonZero(roi[:, :20]) == 0
    assert roi[50, 100] == 255

    gray = cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY)
    low, high = adaptive_canny_thresholds(gray, 60, 160, 0.33)
    assert 0 <= low < high <= 255


def test_image_quality_rejects_unusable_frames_and_scales_evidence():
    """Black/defocused frames do not contribute a false temporal vote."""
    black = np.zeros((120, 160), dtype=np.uint8)
    checker = 40 + np.indices((120, 160)).sum(axis=0) % 2 * 170
    assert image_quality_score(black) < 0.35
    assert image_quality_score(checker.astype(np.uint8)) > 0.7
    evidence = ObstacleEvidence("wall", 0.8, 0.5, 0.5, 0.4, 0.4)
    assert apply_image_quality(evidence, image_quality_score(black), 0.35).hint == "none"
    accepted = apply_image_quality(evidence, 0.8, 0.35)
    assert accepted.hint == "wall"
    assert 0.7 < accepted.confidence < evidence.confidence


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


def test_ground_envelope_separates_step_from_ground_slope():
    """A raised block must not turn a flat floor into a steep fitted slope."""
    rng = np.random.default_rng(7)
    ground = np.column_stack(
        (
            rng.uniform(0.1, 1.5, 3000),
            rng.uniform(-0.4, 0.4, 3000),
            rng.normal(0.0, 0.002, 3000),
        )
    )
    step = np.column_stack(
        (
            rng.uniform(0.65, 0.85, 500),
            rng.uniform(-0.15, 0.15, 500),
            rng.normal(0.12, 0.002, 500),
        )
    )
    result = compute_terrain_features(
        np.vstack((ground, step)),
        0.1,
        1.5,
        0.45,
        30000,
        0.1,
        0.28,
        0.45,
        0.06,
        30,
    )
    features, _ = result
    assert abs(features[4]) < 0.02
    assert 0.10 <= features[2] <= 0.14
    assert 0.60 <= features[7] <= 0.90


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


def test_xyz_transform_handles_rotation_translation_and_invalid_quaternion():
    """XYZ 变换正确应用刚体位姿并拒绝退化四元数。"""
    points = np.asarray([[1.0, 0.0, 0.0], [0.0, 1.0, 2.0]], dtype=np.float32)
    half = np.sqrt(0.5)
    transformed = transform_xyz(points, (1.0, 2.0, 3.0), (0.0, 0.0, half, half))
    np.testing.assert_allclose(
        transformed,
        [[1.0, 3.0, 3.0], [0.0, 2.0, 5.0]],
        atol=1e-6,
    )
    with pytest.raises(ValueError, match="degenerate"):
        transform_xyz(points, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))


def _dense_floor(z=0.0):
    """每个栅格给多个回波，模拟有组织深度点云。"""
    rows = []
    for x in np.arange(0.1, 1.31, 0.05):
        for y in np.arange(-0.4, 0.41, 0.05):
            rows.extend(((x, y, z - 0.001), (x, y, z + 0.001)))
    return np.asarray(rows, dtype=np.float64)


def test_grid_ground_segmentation_detects_wall_without_biasing_plane():
    """墙面不能把稳健地面平面拉成斜坡。"""
    floor = _dense_floor()
    wall = np.asarray(
        [
            (x, y, z)
            for x in (0.59, 0.61)
            for y in np.arange(-0.30, 0.31, 0.04)
            for z in np.arange(0.0, 0.36, 0.03)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, wall)))
    assert result.valid
    assert result.obstacle_type == WALL
    # 稳健 98% 分位不会追随最高单点，允许少量保守低估。
    assert result.obstacle_height >= 0.28
    assert abs(result.slope_pitch) < 0.03


def test_grid_ground_segmentation_requires_real_low_returns_for_pit():
    """坑洞分类必须由真实低处回波支持。"""
    floor = _dense_floor()
    pit = np.asarray(
        [
            (x, y, -0.16 + noise)
            for x in np.arange(0.55, 0.76, 0.04)
            for y in np.arange(-0.15, 0.16, 0.04)
            for noise in (-0.002, 0.002)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, pit)))
    assert result.valid
    assert result.obstacle_type == PIT
    assert result.pit_depth >= 0.12


def test_disconnected_depth_speckles_do_not_form_an_obstacle():
    """Several isolated flying pixels are not a spatially coherent hazard surface."""
    floor = _dense_floor()
    speckles = np.asarray(
        [
            point
            for x, y in ((0.25, -0.30), (0.75, 0.30), (1.20, -0.20))
            for point in ((x, y, 0.34), (x, y, 0.36))
        ],
        dtype=np.float64,
    )
    result = analyze_terrain_geometry(np.vstack((floor, speckles)))
    assert result.valid
    assert result.obstacle_type == CLEAR


def test_robust_ground_fit_preserves_long_slope_with_high_outliers():
    """A long ramp remains a ramp when a small elevated cluster is present."""
    floor = _dense_floor()
    floor[:, 2] += 0.15 * floor[:, 0]
    outliers = np.asarray(
        [
            (x, y, 0.15 * x + 0.30)
            for x in (0.60, 0.65)
            for y in (-0.05, 0.0, 0.05)
            for _ in range(2)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, outliers)))
    assert result.valid
    assert abs(result.slope_pitch - np.arctan(0.15)) < 0.03

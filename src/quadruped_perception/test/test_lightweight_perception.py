"""Tests for bounded OpenCV and NumPy terrain feature extraction."""

from pathlib import Path

import cv2
import numpy as np
import pytest

from quadruped_perception.terrain_analyzer import (
    bounded_point_sample,
    compute_terrain_features,
    filter_roi_points,
    transform_xyz,
)
from quadruped_perception.terrain_geometry import (
    _grid_samples,
    _largest_connected_region,
    BAR,
    CLEAR,
    PIT,
    POLE,
    STEP,
    UNKNOWN,
    WALL,
    analyze_terrain_geometry,
    navigation_obstacle_points,
    obstacle_front_heading,
)
from quadruped_perception.topic_selection import should_accept_source
from quadruped_perception.vision_obstacle_detector import (
    ObstacleEvidence,
    adaptive_canny_thresholds,
    annotate_detection_frame,
    apply_image_quality,
    apply_detection_roi,
    combined_hsv_mask,
    detect_obstacle_evidence,
    enhance_illumination,
    evidence_iou,
    image_quality_score,
    hsv_range_mask,
    largest_color_feature,
    stabilize_evidence,
    suppress_specular_edges,
    temporal_history_requires_reset,
)


def test_annotated_image_visualizes_roi_candidate_and_confirmed_result():
    """RViz 调试图必须保留尺寸，并把候选/确认信息画到原始图像副本上。"""
    source = np.zeros((120, 200, 3), dtype=np.uint8)
    candidate = ObstacleEvidence("wall", 0.7, 0.5, 0.5, 0.4, 0.5)
    confirmed = ObstacleEvidence("wall", 0.65, 0.5, 0.5, 0.4, 0.5)
    annotated = annotate_detection_frame(
        source, candidate, confirmed, 0.8, 0.05, 0.95, 0.02, "PIT"
    )
    assert annotated.shape == source.shape
    assert np.count_nonzero(annotated) > 0
    assert np.count_nonzero(source) == 0


def test_annotated_image_qos_matches_default_rviz_reliability():
    """标注图发布端必须兼容 RViz Image 默认的 RELIABLE 订阅策略。"""
    source = (
        Path(__file__).parents[1]
        / "quadruped_perception"
        / "vision_obstacle_detector.py"
    ).read_text(encoding="utf-8")
    assert "annotated_qos = QoSProfile" in source
    assert "reliability=ReliabilityPolicy.RELIABLE" in source


def test_plain_colored_region_is_not_promoted_to_an_obstacle_class():
    """无结构单色块不能再被误报成含义不明的 COLORED OBSTACLE。"""
    orange = np.zeros((240, 320), dtype=np.uint8)
    empty = np.zeros_like(orange)
    cv2.rectangle(orange, (60, 90), (260, 220), 255, -1)
    evidence = detect_obstacle_evidence(orange, empty, empty, 100.0)
    assert evidence.hint == "none"


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


def test_detects_blue_white_segmented_competition_height_bar():
    """蓝白交替横杆在 HSV 中断成多段，仍应合并成一个限高杆目标。"""
    empty = np.zeros((240, 320), dtype=np.uint8)
    blue = np.zeros_like(empty)
    # 每个蓝段故意小于通用 300 px 面积门，复现远距离规则横杆。
    # 横杆低于相机安装高度，接近时会落在画面下部，不能沿用“地平线以上”假设。
    for left in (62, 104, 146, 188, 230):
        cv2.rectangle(blue, (left, 198), (left + 20, 205), 255, -1)
    evidence = detect_obstacle_evidence(empty, blue, empty, 300.0)
    assert evidence.hint == "height_bar"
    assert evidence.confidence >= 0.75
    assert 0.48 <= evidence.center_x <= 0.51
    assert evidence.center_y >= 0.80


def test_segmented_bar_rejects_irregular_aligned_blue_clutter():
    """Three blue objects on one row are not a bar without repeated spacing."""
    empty = np.zeros((240, 320), dtype=np.uint8)
    blue = np.zeros_like(empty)
    # Similar blue pieces but gaps of 8 px then 105 px: this models signs/clothing
    # that previously passed horizontal alignment and could stabilize across frames.
    for left in (40, 69, 195, 224):
        cv2.rectangle(blue, (left, 105), (left + 20, 112), 255, -1)
    evidence = detect_obstacle_evidence(empty, blue, empty, 300.0)
    assert evidence.hint != "height_bar"


def test_height_bar_rejects_scene_spanning_floor_or_horizon_region():
    """贴近左右边界的宽高色块是场地/地平线，不应长期触发横杆限速。"""
    empty = np.zeros((240, 320), dtype=np.uint8)
    blue = np.zeros_like(empty)
    # 复现 Gazebo 联调中稳定误报的约 96% 宽、34% 高蓝色外接框。
    cv2.rectangle(blue, (6, 60), (313, 141), 255, -1)
    evidence = detect_obstacle_evidence(empty, blue, empty, 100.0)
    assert evidence.hint != "height_bar"

    # 即使框较薄，横跨近乎整幅画面的地平线也应交给点云而不是单目横杆分支。
    blue.fill(0)
    cv2.rectangle(blue, (3, 70), (316, 100), 255, -1)
    evidence = detect_obstacle_evidence(empty, blue, empty, 100.0)
    assert evidence.hint != "height_bar"


def test_uncolored_horizontal_edges_cannot_claim_height_bar():
    """台阶顶边和地平线仅有 Canny 轮廓时，必须等待颜色或点云确认。"""
    empty = np.zeros((240, 320), dtype=np.uint8)
    edges = np.zeros_like(empty)
    # 复现联调标注图中约 57% 宽、8% 高且跨多帧稳定的远处障碍顶边。
    cv2.rectangle(edges, (38, 83), (220, 103), 255, 3)
    evidence = detect_obstacle_evidence(empty, empty, edges, 100.0)
    assert evidence.hint != "height_bar"


def test_grayscale_geometry_and_temporal_confirmation():
    """Edges work without target colors, while one-frame noise is rejected."""
    mask = np.zeros((240, 320), dtype=np.uint8)
    cv2.rectangle(mask, (60, 40), (72, 220), 255, -1)
    cv2.rectangle(mask, (220, 45), (232, 220), 255, -1)
    evidence = detect_obstacle_evidence(
        np.zeros_like(mask), np.zeros_like(mask), mask, 100.0
    )
    assert evidence.hint == "poles"

    # 确认结果必须属于当前帧；旧命中可保留在窗口里，但不能在当前
    # NONE 帧上伪造一个带新 Header 的旧框。
    history = [ObstacleEvidence(), ObstacleEvidence(), evidence, evidence, evidence]
    stable = stabilize_evidence(history, 3)
    assert stable.hint == "poles"
    assert stable.confidence >= 0.55
    assert stabilize_evidence(history[:2], 3).hint == "none"

    disappeared = [evidence, evidence, evidence, ObstacleEvidence()]
    assert stabilize_evidence(disappeared, 3).hint == "none"


def test_edge_only_rectangles_cannot_claim_wall_semantics():
    """单目边缘框无法区分墙、台阶和场地边界，墙类别必须等待点云确认。"""
    mask = np.zeros((240, 320), dtype=np.uint8)
    cv2.rectangle(mask, (20, 45), (300, 220), 255, 4)
    evidence = detect_obstacle_evidence(
        np.zeros_like(mask), np.zeros_like(mask), mask, 100.0
    )
    assert evidence.hint != "wall"

    # 中等大小的闭合框仍然没有米制高度，不能只因尺寸看似像墙就输出 WALL。
    mask.fill(0)
    cv2.rectangle(mask, (80, 100), (240, 210), 255, 4)
    evidence = detect_obstacle_evidence(
        np.zeros_like(mask), np.zeros_like(mask), mask, 100.0
    )
    assert evidence.hint != "wall"


def test_pole_pair_rejects_unaligned_vertical_clutter():
    """Two unrelated vertical color regions must not masquerade as a gate."""
    orange = np.zeros((240, 320), dtype=np.uint8)
    empty = np.zeros_like(orange)
    cv2.rectangle(orange, (50, 10), (70, 100), 255, -1)
    cv2.rectangle(orange, (220, 145), (240, 235), 255, -1)
    evidence = detect_obstacle_evidence(orange, empty, empty, 100.0)
    assert evidence.hint != "poles"


def test_single_centered_colored_pole_is_not_mislabeled_as_wall():
    """另一根杆在视野外时，前向单根细长色柱仍应优先于歧义墙轮廓。"""
    orange = np.zeros((240, 320), dtype=np.uint8)
    edges = np.zeros_like(orange)
    cv2.rectangle(orange, (150, 65), (170, 220), 255, -1)
    cv2.rectangle(edges, (149, 64), (171, 221), 255, 2)
    evidence = detect_obstacle_evidence(orange, np.zeros_like(orange), edges, 100.0)
    assert evidence.hint == "poles"
    assert evidence.confidence >= 0.70

    # 画面边缘的竖直场地边框不属于正前方立柱，保持无提示或交给其他几何分支。
    orange.fill(0)
    edges.fill(0)
    cv2.rectangle(orange, (2, 65), (20, 220), 255, -1)
    evidence = detect_obstacle_evidence(orange, np.zeros_like(orange), edges, 100.0)
    assert evidence.hint != "poles"


def test_temporal_confirmation_rejects_spatially_inconsistent_boxes():
    """Repeated labels at jumping positions are treated as separate objects."""
    history = [
        ObstacleEvidence("poles", 0.8, center_x, 0.5, 0.2, 0.5)
        for center_x in (0.15, 0.50, 0.85)
    ]
    assert stabilize_evidence(history, 3).hint == "none"


def test_temporal_confirmation_tracks_smooth_approach_and_uses_current_box():
    """同一障碍平滑放大/下移时应继续确认，输出框必须属于最新帧。"""
    history = [
        ObstacleEvidence("poles", 0.82, 0.50, center_y, width, height)
        for center_y, width, height in (
            (0.40, 0.10, 0.24),
            (0.44, 0.13, 0.31),
            (0.49, 0.17, 0.40),
            (0.55, 0.22, 0.51),
            (0.62, 0.29, 0.65),
        )
    ]
    stable = stabilize_evidence(history, 3)
    assert stable.hint == "poles"
    # 节点给结果填当前图像 Header，因此不得返回历史中位框。
    assert stable.center_y == history[-1].center_y
    assert stable.width == history[-1].width
    assert stable.height == history[-1].height


def test_temporal_confirmation_tracks_smooth_turn_but_rejects_one_frame_jump():
    """连续转向允许累计大位移，单帧瞬移仍不能被多数标签掩盖。"""
    smooth_turn = [
        ObstacleEvidence("height_bar", 0.84, center_x, 0.48, 0.42, 0.16)
        for center_x in (0.18, 0.28, 0.38, 0.48, 0.58)
    ]
    stable = stabilize_evidence(smooth_turn, 3)
    assert stable.hint == "height_bar"
    assert stable.center_x == smooth_turn[-1].center_x

    teleported = smooth_turn[:4] + [
        ObstacleEvidence("height_bar", 0.84, 0.88, 0.48, 0.42, 0.16)
    ]
    assert stabilize_evidence(teleported, 3).hint == "none"


def test_temporal_confirmation_allows_one_quality_dropout_during_smooth_motion():
    """一帧模糊被质量门拒绝后，平滑运动应按真实帧间隔继续关联。"""
    history = [
        ObstacleEvidence("poles", 0.82, 0.20, 0.50, 0.40, 0.45),
        ObstacleEvidence(),
        ObstacleEvidence("poles", 0.82, 0.40, 0.50, 0.40, 0.45),
        ObstacleEvidence("poles", 0.82, 0.50, 0.50, 0.40, 0.45),
        ObstacleEvidence("poles", 0.82, 0.60, 0.50, 0.40, 0.45),
    ]
    stable = stabilize_evidence(history, 3)
    assert stable.hint == "poles"
    assert stable.center_x == history[-1].center_x


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


def test_visual_history_resets_after_dropout_or_clock_rewind():
    """相机长间隔和 rosbag 时钟回拨都不能沿用旧的多帧票数。"""
    assert not temporal_history_requires_reset(10.0, 10.1, 0.75)
    assert temporal_history_requires_reset(10.0, 10.8, 0.75)
    assert temporal_history_requires_reset(10.0, 9.0, 0.75)
    assert temporal_history_requires_reset(float("nan"), 10.0, 0.75)


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


def test_image_quality_rejects_clipped_light_and_motion_blur():
    """全黑、全白和运动模糊帧均不能靠亮度增强绕过质量门。"""
    black = np.zeros((120, 160), dtype=np.uint8)
    white = np.full_like(black, 255)
    indices = np.indices(black.shape)
    sharp = np.where(
        (indices[0] // 8 + indices[1] // 8) % 2 == 0, 50, 200
    ).astype(np.uint8)
    blurred = cv2.GaussianBlur(sharp, (31, 31), 0)
    assert image_quality_score(black) < 0.10
    assert image_quality_score(white) < 0.10
    assert image_quality_score(blurred) < image_quality_score(sharp)


def _full_visual_candidate(image):
    """用在线节点同一预处理顺序生成单帧候选，供极端光照回归复用。"""
    raw_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    quality = image_quality_score(raw_gray)
    enhanced = enhance_illumination(image, 2.0, 8)
    orange = combined_hsv_mask(
        image,
        enhanced,
        np.asarray((5, 80, 70), dtype=np.uint8),
        np.asarray((25, 255, 255), dtype=np.uint8),
    )
    blue = combined_hsv_mask(
        image,
        enhanced,
        np.asarray((90, 70, 50), dtype=np.uint8),
        np.asarray((135, 255, 255), dtype=np.uint8),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel)
    orange = cv2.morphologyEx(orange, cv2.MORPH_CLOSE, kernel)
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, kernel)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, kernel)
    gray = cv2.GaussianBlur(cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    low, high = adaptive_canny_thresholds(gray, 60, 160, 0.33)
    edges = cv2.dilate(
        cv2.morphologyEx(cv2.Canny(gray, low, high), cv2.MORPH_CLOSE, kernel),
        kernel,
        iterations=1,
    )
    edges = suppress_specular_edges(edges, image, 25, 245, 7)
    masks = [
        apply_detection_roi(mask, 0.05, 0.95, 0.02)
        for mask in (orange, blue, edges)
    ]
    evidence = detect_obstacle_evidence(*masks, max(300.0, image.size / 3 * 0.0008))
    return apply_image_quality(evidence, quality, 0.35), quality


def _textured_pole_scene():
    """合成带真实纹理的橙色双杆，避免把纯色测试误当成真实清晰图像。"""
    rows, columns = np.indices((240, 320))
    background = (55 + ((rows // 8 + columns // 8) % 2) * 35).astype(np.uint8)
    image = cv2.merge((background, background, background))
    cv2.rectangle(image, (62, 55), (82, 215), (0, 117, 223), -1)
    cv2.rectangle(image, (220, 50), (242, 215), (0, 117, 223), -1)
    # 给实体添加细纹理，使运动模糊质量测试与真实相机边缘更接近。
    for y in range(60, 210, 12):
        cv2.line(image, (62, y), (82, y), (0, 90, 180), 1)
        cv2.line(image, (220, y), (242, y), (0, 90, 180), 1)
    return image


def test_full_visual_pipeline_handles_brightness_and_local_shadow():
    """正常、整体变暗和半幅阴影下，比赛色双杆仍应产生同类候选。"""
    nominal = _textured_pole_scene()
    dim = cv2.convertScaleAbs(nominal, alpha=0.62, beta=8)
    shadow = nominal.copy()
    shadow[:, :160] = cv2.convertScaleAbs(shadow[:, :160], alpha=0.55, beta=5)
    for frame in (nominal, dim, shadow):
        evidence, quality = _full_visual_candidate(frame)
        assert quality >= 0.35
        assert evidence.hint == "poles"


def test_full_visual_pipeline_rejects_overexposure_and_downweights_motion_blur():
    """过曝必须拒绝；仍保留颜色的运动模糊帧必须显著降低置信度。"""
    overexposed = _textured_pole_scene()
    overexposed[:, :] = 245
    cv2.rectangle(overexposed, (70, 80), (250, 125), (255, 255, 255), -1)
    evidence, quality = _full_visual_candidate(overexposed)
    assert quality < 0.35
    assert evidence.hint == "none"

    nominal_evidence, nominal_quality = _full_visual_candidate(_textured_pole_scene())
    kernel = np.zeros((1, 35), dtype=np.float32)
    kernel[0, :] = 1.0 / kernel.shape[1]
    blurred = cv2.filter2D(_textured_pole_scene(), -1, kernel)
    evidence, quality = _full_visual_candidate(blurred)
    assert quality < nominal_quality
    # 若色块仍足够完整可以保留 poles 候选，但质量权重必须让它比清晰帧更难通过时序门。
    assert evidence.hint in ("none", "poles")
    if evidence.hint == "poles":
        assert evidence.confidence < nominal_evidence.confidence


def test_dual_illumination_mask_recovers_shadow_color_without_losing_original():
    """阴影色块可由增强分支补回，正常原图颜色仍保留。"""
    original = np.zeros((80, 120, 3), dtype=np.uint8)
    enhanced = np.zeros_like(original)
    # 暗橙色原图低于 V 下限；增强图恢复亮度但保持色相。
    cv2.rectangle(original, (10, 20), (50, 60), (0, 35, 70), -1)
    cv2.rectangle(enhanced, (10, 20), (50, 60), (0, 90, 180), -1)
    # 第二个正常橙色块只存在于原图，验证掩膜确实取并集。
    cv2.rectangle(original, (70, 20), (105, 60), (0, 90, 180), -1)
    lower = np.asarray((5, 80, 70), dtype=np.uint8)
    upper = np.asarray((25, 255, 255), dtype=np.uint8)
    mask = combined_hsv_mask(original, enhanced, lower, upper)
    assert mask[40, 30] == 255
    assert mask[40, 85] == 255


def test_hsv_mask_supports_hue_wrap_at_opencv_boundary():
    """红橙色标定跨越 H=179/0 时，两端色相都必须保留。"""
    hsv = np.asarray([[[178, 180, 180], [4, 180, 180], [80, 180, 180]]], dtype=np.uint8)
    mask = hsv_range_mask(
        hsv,
        np.asarray((170, 80, 70), dtype=np.uint8),
        np.asarray((12, 255, 255), dtype=np.uint8),
    )
    assert mask.tolist() == [[255, 255, 0]]


def test_specular_glare_edges_are_removed_but_colored_edges_remain():
    """白色过曝反光不应形成横杆轮廓，邻近蓝色结构仍可检测。"""
    image = np.zeros((100, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (10, 20), (70, 40), (255, 255, 255), -1)
    cv2.rectangle(image, (90, 20), (150, 40), (180, 60, 20), -1)
    edges = cv2.Canny(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 30, 80)
    filtered = suppress_specular_edges(edges, image, 25, 245, 7)
    assert cv2.countNonZero(filtered[:, 5:76]) == 0
    assert cv2.countNonZero(filtered[:, 85:156]) > 0


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


def test_vertical_roi_rejects_finite_depth_sentinel_values():
    """Drivers may encode no-return pixels as huge finite numbers, not NaN/Inf."""
    points = np.asarray(
        [
            [0.5, 0.0, 0.0],
            [0.6, 0.1, -0.4],
            [0.7, 0.0, 87.7],
            [0.8, 0.0, -50.0],
        ],
        dtype=np.float32,
    )
    filtered = filter_roi_points(
        points, 0.1, 1.5, 0.45, 100, z_min=-1.0, z_max=2.0
    )
    assert filtered.shape == (2, 3)
    assert np.max(filtered[:, 2]) <= 2.0
    assert np.min(filtered[:, 2]) >= -1.0


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


def test_near_vertical_surface_is_not_accepted_as_ground_plane():
    """A wall-only startup must fail closed instead of extrapolating huge heights."""
    rng = np.random.default_rng(23)
    x_values = rng.uniform(0.2, 1.0, 3000)
    points = np.column_stack(
        (
            x_values,
            rng.uniform(-0.4, 0.4, 3000),
            2.0 * x_values + rng.normal(0.0, 0.002, 3000),
        )
    )
    estimate = analyze_terrain_geometry(points, min_cells=8)
    assert not estimate.valid
    assert estimate.obstacle_height == 0.0


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


def test_pre_transform_sampling_is_bounded_deterministic_and_full_span():
    """高分辨率云必须覆盖首尾等距采样；禁用上限时不得复制或截断数据。"""
    points = np.arange(300, dtype=np.float32).reshape(100, 3)
    first = bounded_point_sample(points, 12)
    second = bounded_point_sample(points, 12)
    assert first.shape == (12, 3)
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(first[0], points[0])
    np.testing.assert_array_equal(first[-1], points[-1])
    np.testing.assert_array_equal(bounded_point_sample(points, 0), points)


def _dense_floor(z=0.0):
    """每个栅格给多个回波，模拟有组织深度点云。"""
    rows = []
    for x in np.arange(0.1, 1.31, 0.05):
        for y in np.arange(-0.4, 0.41, 0.05):
            rows.extend(((x, y, z - 0.001), (x, y, z + 0.001)))
    return np.asarray(rows, dtype=np.float64)


def test_clear_requires_normal_continuous_ground_through_body_corridor():
    """密集平地应通过中央通道可见性门，并保留高 CLEAR 置信度。"""
    result = analyze_terrain_geometry(_dense_floor())
    assert result.valid
    assert result.obstacle_type == CLEAR
    assert result.confidence > 0.95


def test_clear_fails_closed_for_complete_no_return_band_in_central_corridor():
    """两侧和远处虽有点，横贯落脚通道的无回波带仍必须是 UNKNOWN，而不是 CLEAR/PIT。"""
    floor = _dense_floor()
    missing_band = (
        (floor[:, 0] >= 0.55)
        & (floor[:, 0] <= 0.90)
        & (np.abs(floor[:, 1]) <= 0.25)
    )
    result = analyze_terrain_geometry(floor[~missing_band])
    assert not result.valid
    assert result.obstacle_type == UNKNOWN
    assert result.pit_depth == 0.0


def test_clear_fails_closed_when_all_ground_ahead_of_near_patch_is_missing():
    """只看到脚尖前一小片地面不能批准继续前进到尚未观测的区域。"""
    near_only = _dense_floor()
    near_only = near_only[near_only[:, 0] <= 0.45]
    result = analyze_terrain_geometry(near_only)
    assert not result.valid
    assert result.obstacle_type == UNKNOWN


def test_clear_confidence_includes_tolerated_ground_coverage_dropout():
    """小于 max_gap 的单行丢点可继续 CLEAR，但置信度必须低于完整平地。"""
    floor = _dense_floor()
    complete = analyze_terrain_geometry(floor)
    one_missing_row = floor[np.abs(floor[:, 0] - 0.40) > 0.015]
    degraded = analyze_terrain_geometry(one_missing_row)
    assert degraded.valid
    assert degraded.obstacle_type == CLEAR
    assert 0.0 < degraded.confidence < complete.confidence


def test_whole_floor_height_translation_conflicts_with_recent_ground_prior():
    """机身高度整体变化必须先 UNKNOWN，不能被旧先验钉成一整片假 PIT。"""
    shifted_floor = _dense_floor(z=0.20)
    result = analyze_terrain_geometry(
        shifted_floor,
        ground_height_prior=0.0,
        ground_prior_max_height_shift=0.10,
    )
    assert not result.valid
    assert result.obstacle_type == UNKNOWN
    assert result.pit_depth == 0.0
    assert result.ground_reference_conflict


def test_front_edge_heading_recovers_oblique_crossing_normal():
    """斜置台阶前缘应给出法线方向，避免越障 Action 沿边缘斜穿。"""
    expected = np.deg2rad(32.0)
    normal = np.asarray((np.cos(expected), np.sin(expected)))
    tangent = np.asarray((-normal[1], normal[0]))
    centre = np.asarray((0.75, 0.0))
    cells = []
    for along in np.linspace(-0.45, 0.45, 19):
        for depth in (0.00, 0.04, 0.08):
            xy = centre + tangent * along + normal * depth
            cells.append((xy[0], xy[1], 0.0, 0.0, 0.12, 4.0))
    heading, confidence = obstacle_front_heading(
        np.asarray(cells), float(np.min(np.asarray(cells)[:, 0])), 0.05
    )
    assert abs(heading - expected) < np.deg2rad(6.0)
    assert confidence >= 0.45


def test_front_edge_heading_rejects_isotropic_blob():
    """近圆形小斑块没有唯一入口方向，必须回退到中心对正。"""
    cells = np.asarray(
        [
            (0.70 + dx, dy, 0.0, 0.0, 0.12, 3.0)
            for dx in (-0.05, 0.0, 0.05)
            for dy in (-0.05, 0.0, 0.05)
        ]
    )
    heading, confidence = obstacle_front_heading(cells, 0.65, 0.05)
    assert heading == 0.0
    assert confidence == 0.0


def test_vectorized_grid_statistics_match_linear_quantile_reference():
    """RK3588 快速路径必须保持旧版格内低/中/高分位数含义。"""
    rng = np.random.default_rng(23)
    points = np.column_stack(
        (
            rng.uniform(0.1, 1.3, 4000),
            rng.uniform(-0.4, 0.4, 4000),
            rng.normal(-0.4, 0.03, 4000),
        )
    )
    cells = _grid_samples(points, 0.05)
    assert len(cells) > 100
    # 抽查每个快速结果的原格；XY 用均值，Z 应与 NumPy linear quantile 完全一致。
    for row in cells[:: max(1, len(cells) // 20)]:
        coordinate = np.floor(row[:2] / 0.05).astype(np.int32)
        point_coordinates = np.floor(points[:, :2] / 0.05).astype(np.int32)
        selected = points[np.all(point_coordinates == coordinate, axis=1)]
        assert len(selected) == int(row[5])
        np.testing.assert_allclose(row[:2], np.mean(selected[:, :2], axis=0))
        np.testing.assert_allclose(
            row[2:5], np.quantile(selected[:, 2], (0.15, 0.50, 0.90))
        )


def test_competition_threshold_sweep_low_step_pit_slope_and_bar():
    """按比赛尺寸和保守测量误差验证低台阶、坑、10/14°坡与限高杆阈值。"""
    rng = np.random.default_rng(29)
    floor = _dense_floor(z=-0.44)

    low_step = np.asarray(
        [
            (x, y, -0.36 + noise)
            for x in np.arange(0.55, 0.86, 0.035)
            for y in np.arange(-0.20, 0.21, 0.035)
            for noise in rng.normal(0.0, 0.003, 3)
        ]
    )
    estimate = analyze_terrain_geometry(
        np.vstack((floor, low_step)),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        min_region_cells=4,
        min_region_points=16,
    )
    assert estimate.obstacle_type == STEP
    assert 0.065 <= estimate.obstacle_height <= 0.10
    assert abs(estimate.lateral_offset) <= 0.03

    pit = np.asarray(
        [
            (x, y, -0.53 + noise)
            for x in np.arange(0.55, 0.86, 0.035)
            for y in np.arange(-0.20, 0.21, 0.035)
            for noise in rng.normal(0.0, 0.003, 3)
        ]
    )
    estimate = analyze_terrain_geometry(
        np.vstack((floor, pit)),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        min_region_cells=4,
        min_region_points=16,
    )
    assert estimate.obstacle_type == PIT
    assert estimate.pit_depth >= 0.075

    for angle in (10.0, 14.0):
        ramp = _dense_floor(z=-0.44)
        ramp[:, 2] += np.tan(np.deg2rad(angle)) * ramp[:, 0]
        estimate = analyze_terrain_geometry(
            ramp,
            step_height=0.07,
            pit_depth=0.07,
            wall_height=0.23,
            min_region_cells=4,
            min_region_points=16,
        )
        assert estimate.obstacle_type == CLEAR
        assert abs(np.rad2deg(estimate.slope_pitch) - angle) < 1.5

    bar = np.asarray(
        [
            (x, y, -0.14 + noise)
            for x in (0.59, 0.61)
            for y in np.arange(-0.40, 0.41, 0.03)
            for noise in (-0.012, 0.0, 0.012)
        ]
    )
    estimate = analyze_terrain_geometry(
        np.vstack((floor, bar)),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        bar_min_clearance=0.18,
        min_region_cells=4,
        min_region_points=16,
    )
    assert estimate.obstacle_type == BAR
    assert 0.27 <= estimate.clearance_height <= 0.31


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


def test_thin_wall_top_returns_are_not_downgraded_to_step():
    """正视薄墙只返回顶边时，XY 厚度仍足以区别墙和深踏面。"""
    floor = _dense_floor()
    # 实体墙遮挡脚下地面；横杆测试则保留同一 XY 格中的地面回波。
    floor = floor[
        ~(
            (floor[:, 0] >= 0.55)
            & (floor[:, 0] <= 0.65)
            & (np.abs(floor[:, 1]) <= 0.32)
        )
    ]
    wall_top = np.asarray(
        [
            (x, y, 0.30 + noise)
            for x in (0.59, 0.61)
            for y in np.arange(-0.30, 0.31, 0.035)
            for noise in (-0.002, 0.0, 0.002)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, wall_top)))
    assert result.valid
    assert result.obstacle_type == WALL
    assert result.obstacle_height >= 0.28


def test_oblique_thin_wall_uses_orientation_independent_thickness():
    """斜视 0.10 m 薄墙时，PCA 短轴厚度应防止它被错误当成深台面。"""
    floor = _dense_floor()
    tangent = np.asarray((1.0, 0.75), dtype=np.float64)
    tangent /= np.linalg.norm(tangent)
    normal = np.asarray((-tangent[1], tangent[0]), dtype=np.float64)
    center = np.asarray((0.85, 0.0), dtype=np.float64)
    wall_xy = []
    for along in np.arange(-0.45, 0.46, 0.035):
        for thick in (-0.04, 0.0, 0.04):
            wall_xy.append(center + tangent * along + normal * thick)
    wall_xy = np.asarray(wall_xy)
    # 实体墙遮住自身投影下的地面，否则 ground_coexistence 会正确把它视为悬空结构。
    relative_floor = floor[:, :2] - center
    along_floor = relative_floor @ tangent
    normal_floor = relative_floor @ normal
    floor = floor[
        ~((np.abs(along_floor) <= 0.48) & (np.abs(normal_floor) <= 0.07))
    ]
    wall_top = np.asarray(
        [
            (xy[0], xy[1], 0.30 + noise)
            for xy in wall_xy
            for noise in (-0.002, 0.0, 0.002)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, wall_top)))
    assert result.valid
    assert result.obstacle_type == WALL

    # 同高度但具有二维纵深的踏面必须仍是 STEP，不能被薄墙分支吞掉。
    platform = np.asarray(
        [
            (x, y, 0.30 + noise)
            for x in np.arange(0.60, 1.41, 0.04)
            for y in np.arange(-0.35, 0.36, 0.04)
            for noise in (-0.002, 0.0, 0.002)
        ]
    )
    platform_result = analyze_terrain_geometry(np.vstack((_dense_floor(), platform)))
    assert platform_result.valid
    assert platform_result.obstacle_type == STEP


def test_occluding_wall_top_cannot_become_ground_and_invert_into_pit():
    """墙顶栅格多于可见地面时，最低受支持近场层仍应锚定真实地面。"""
    floor = np.asarray(
        [
            (x, y, z)
            for x in np.arange(0.10, 0.41, 0.05)
            for y in np.arange(-0.20, 0.21, 0.05)
            for z in (-0.002, 0.0, 0.002)
        ]
    )
    # 两排墙顶横跨更宽视场，因此旧“最大高度箱”算法会错误选择 z=0.30 m。
    wall_top = np.asarray(
        [
            (x, y, 0.30 + noise)
            for x in (0.59, 0.61)
            for y in np.arange(-0.45, 0.46, 0.035)
            for noise in (-0.002, 0.0, 0.002)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, wall_top)))
    assert result.valid
    assert result.obstacle_type == WALL
    assert result.ground_height < 0.03
    assert result.pit_depth < 0.03

    # 最坏情况下墙顶完全遮住地面；连续运行时上一 CLEAR 帧的地面高度仍能恢复墙。
    prior_only = analyze_terrain_geometry(
        wall_top,
        ground_height_prior=0.0,
        min_cells=8,
        min_region_cells=3,
        min_region_points=12,
    )
    assert prior_only.valid
    assert prior_only.obstacle_type == WALL
    assert prior_only.ground_height < 0.03


def test_grid_ground_segmentation_detects_competition_height_bar_clearance():
    """离地约 0.30 m 的细横杆不能被同一切片中的地面点误判为墙。"""
    floor = _dense_floor()
    bar = np.asarray(
        [
            (x, y, z)
            for x in (0.59, 0.61)
            for y in np.arange(-0.40, 0.41, 0.04)
            for z in (0.29, 0.31, 0.33, 0.35)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, bar)))
    assert result.valid
    assert result.obstacle_type == BAR
    assert 0.25 <= result.clearance_height <= 0.33


def test_extended_stair_top_is_not_mistaken_for_a_suspended_bar():
    """只看到 0.30 m 台面时仍可用纵深排除限高杆，覆盖整场联调的 T 台样本。"""
    floor = _dense_floor()
    platform = np.asarray(
        [
            (x, y, 0.30 + noise)
            for x in np.arange(0.60, 1.41, 0.04)
            for y in np.arange(-0.35, 0.36, 0.04)
            for noise in (-0.006, 0.0, 0.006)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, platform)))
    assert result.valid
    assert result.obstacle_type == STEP
    assert result.clearance_height == 0.0


def test_near_field_ground_anchor_keeps_stairs_out_of_pit_class():
    """台阶占据多数栅格时，入口近场平地仍必须作为地面，不能反报成坑。"""
    points = []
    for x in np.arange(0.10, 0.71, 0.04):
        for y in np.arange(-0.45, 0.46, 0.04):
            points.extend(((x, y, -0.001), (x, y, 0.001)))
    for x in np.arange(0.75, 1.91, 0.04):
        level = min(4, int((x - 0.75) / 0.28) + 1)
        height = 0.10 * level
        for y in np.arange(-0.45, 0.46, 0.04):
            points.extend(((x, y, height - 0.002), (x, y, height + 0.002)))
    result = analyze_terrain_geometry(
        np.asarray(points),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        min_region_cells=4,
        min_region_points=16,
    )
    assert result.valid
    assert result.obstacle_type == STEP
    assert result.obstacle_height >= 0.35
    assert result.pit_depth < 0.02


def test_near_field_ground_anchor_recognizes_bridge_approach_as_ramp():
    """平地后的 14 度木桥引坡应输出坡度，而不是把平地误报为坑。"""
    points = []
    for x in np.arange(0.10, 0.71, 0.04):
        for y in np.arange(-0.45, 0.46, 0.04):
            points.extend(((x, y, -0.001), (x, y, 0.001)))
    tangent = np.tan(np.deg2rad(14.0))
    for x in np.arange(0.75, 1.91, 0.04):
        height = tangent * (x - 0.75)
        for y in np.arange(-0.45, 0.46, 0.04):
            points.extend(((x, y, height - 0.002), (x, y, height + 0.002)))
    result = analyze_terrain_geometry(
        np.asarray(points),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        min_region_cells=4,
        min_region_points=16,
    )
    assert result.valid
    assert result.obstacle_type == CLEAR
    assert abs(np.rad2deg(result.slope_pitch) - 14.0) < 1.0
    assert result.pit_depth == 0.0
    # 连通正高度区从坡面超过 7 cm 门限处开始，因此入口距离会比几何坡脚稍远，
    # 但必须显著小于 ROI 的固定 2.5 m 远端，才能随接近过程进入交接范围。
    assert 0.70 <= result.distance <= 1.60


def test_height_bar_with_grounded_supports_uses_crossbar_clearance():
    """限高杆落地支柱不得把横杆净空拉到零并误分类为墙。"""
    floor = _dense_floor()
    supports = np.asarray(
        [
            (x, y, z)
            for x in (0.59, 0.61)
            for y in (-0.32, 0.32)
            for z in np.arange(0.02, 0.35, 0.02)
        ]
    )
    crossbar = np.asarray(
        [
            (x, y, z)
            for x in (0.59, 0.61)
            for y in np.arange(-0.40, 0.41, 0.025)
            for z in (0.30, 0.32, 0.34)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, supports, crossbar)))
    assert result.valid
    assert result.obstacle_type == BAR
    assert 0.27 <= result.clearance_height <= 0.32


def test_dense_narrow_competition_pole_survives_grid_cell_gate():
    """约 70 mm 立柱只占一至两格，但有足够三维回波时仍应识别为立柱。"""
    floor = _dense_floor()
    pole = np.asarray(
        [
            (x, y, z)
            for x in (0.59, 0.61)
            for y in (-0.02, 0.00, 0.02)
            for z in np.arange(0.02, 0.35, 0.015)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, pole)))
    assert result.valid
    assert result.obstacle_type == POLE
    assert result.width <= 0.12


def test_nearest_supported_positive_region_wins_over_larger_far_wall():
    """近处细杆不得被同帧格数更多的远墙覆盖。"""
    floor = []
    for x in np.arange(0.10, 2.31, 0.05):
        for y in np.arange(-0.40, 0.41, 0.05):
            floor.extend(((x, y, -0.001), (x, y, 0.001)))
    near_pole = np.asarray(
        [
            (x, y, z)
            for x in (0.54, 0.56)
            for y in (-0.02, 0.0, 0.02)
            for z in np.arange(0.02, 0.35, 0.015)
        ]
    )
    far_wall = np.asarray(
        [
            (x, y, z)
            for x in (1.49, 1.51)
            for y in np.arange(-0.35, 0.36, 0.035)
            for z in np.arange(0.02, 0.35, 0.02)
        ]
    )
    result = analyze_terrain_geometry(
        np.vstack((np.asarray(floor), near_pole, far_wall)),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        min_region_cells=4,
        min_region_points=16,
    )
    assert result.valid
    assert result.obstacle_type == POLE
    assert 0.45 <= result.distance <= 0.65


def test_selected_region_geometry_is_not_polluted_at_the_same_distance():
    """同距离但横向分离的侧墙不得把中央细杆的前缘跨度扩大。"""
    floor = _dense_floor()
    center_pole = np.asarray(
        [
            (x, y, z)
            for x in (0.59, 0.61)
            for y in (-0.02, 0.0, 0.02)
            for z in np.arange(0.02, 0.35, 0.015)
        ]
    )
    disconnected_side_wall = np.asarray(
        [
            (x, y, z)
            for x in (0.59, 0.61)
            for y in np.arange(0.25, 0.41, 0.03)
            for z in np.arange(0.02, 0.35, 0.02)
        ]
    )
    result = analyze_terrain_geometry(
        np.vstack((floor, center_pole, disconnected_side_wall))
    )
    assert result.valid
    assert result.obstacle_type == POLE
    assert abs(result.lateral_offset) <= 0.05
    assert result.width <= 0.12


def test_nearest_step_wins_over_larger_far_pit_region():
    """近台阶应先于远处坑底回波，坑类别不再无条件抢占优先级。"""
    floor = []
    for x in np.arange(0.10, 2.31, 0.05):
        for y in np.arange(-0.40, 0.41, 0.05):
            floor.extend(((x, y, -0.001), (x, y, 0.001)))
    near_step = np.asarray(
        [
            (x, y, 0.10 + noise)
            for x in np.arange(0.52, 0.77, 0.04)
            for y in np.arange(-0.18, 0.19, 0.04)
            for noise in (-0.002, 0.0, 0.002)
        ]
    )
    far_pit = np.asarray(
        [
            (x, y, -0.13 + noise)
            for x in np.arange(1.35, 1.86, 0.04)
            for y in np.arange(-0.30, 0.31, 0.04)
            for noise in (-0.002, 0.0, 0.002)
        ]
    )
    result = analyze_terrain_geometry(
        np.vstack((np.asarray(floor), near_step, far_pit)),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        min_region_cells=4,
        min_region_points=16,
    )
    assert result.valid
    assert result.obstacle_type == STEP
    assert 0.45 <= result.distance <= 0.80
    assert result.pit_depth == 0.0


def test_nearest_pit_wins_without_inheriting_larger_far_wall_height():
    """近坑洞与远墙同帧时输出 PIT，且不得把远墙高度混入坑几何。"""
    floor = []
    for x in np.arange(0.10, 2.31, 0.05):
        for y in np.arange(-0.40, 0.41, 0.05):
            floor.extend(((x, y, -0.001), (x, y, 0.001)))
    near_pit = np.asarray(
        [
            (x, y, -0.13 + noise)
            # 与既有坑洞回归使用同等尺寸，保证近场仍有足够地面可锚定。
            for x in np.arange(0.55, 0.76, 0.04)
            for y in np.arange(-0.15, 0.16, 0.04)
            for noise in (-0.002, 0.0, 0.002)
        ]
    )
    far_wall = np.asarray(
        [
            (x, y, z)
            for x in (1.49, 1.51)
            for y in np.arange(-0.35, 0.36, 0.035)
            for z in np.arange(0.02, 0.35, 0.02)
        ]
    )
    result = analyze_terrain_geometry(
        np.vstack((np.asarray(floor), near_pit, far_wall)),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        min_region_cells=4,
        min_region_points=16,
    )
    assert result.valid
    assert result.obstacle_type == PIT
    assert 0.45 <= result.distance <= 0.85
    assert result.obstacle_height == 0.0


def test_near_sparse_height_noise_cannot_hide_far_supported_wall():
    """近处相邻高飞点不能借格内地面回波抢占远处真实墙面。"""
    floor = []
    for x in np.arange(0.10, 2.31, 0.05):
        for y in np.arange(-0.40, 0.41, 0.05):
            floor.extend(((x, y, -0.001), (x, y, 0.001)))
    # 四个相邻格各有两个高飞点。旧逻辑把同格地面点也计入 region points，
    # 总数恰好达到 16，于是最近候选会错误遮挡后方墙；真实异常回波只有 8 个。
    near_noise = np.asarray(
        [
            (x, y, z)
            for x in (0.51, 0.56)
            for y in (-0.025, 0.025)
            for z in (0.30, 0.31)
        ]
    )
    far_wall = np.asarray(
        [
            (x, y, z)
            for x in (1.49, 1.51)
            for y in np.arange(-0.35, 0.36, 0.035)
            for z in np.arange(0.02, 0.35, 0.02)
        ]
    )
    result = analyze_terrain_geometry(
        np.vstack((np.asarray(floor), near_noise, far_wall)),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        min_region_cells=4,
        min_region_points=16,
    )
    assert result.valid
    assert result.obstacle_type == WALL
    assert 1.35 <= result.distance <= 1.65


def test_near_sparse_depth_noise_cannot_hide_far_supported_pit():
    """负异常使用独立原始回波计数，近低飞点不能抢占远处真实坑洞。"""
    floor = []
    for x in np.arange(0.10, 2.31, 0.05):
        for y in np.arange(-0.40, 0.41, 0.05):
            floor.extend(((x, y, -0.001), (x, y, 0.001)))
    near_noise = np.asarray(
        [
            (x, y, z)
            for x in (0.51, 0.56)
            for y in (-0.025, 0.025)
            for z in (-0.31, -0.30)
        ]
    )
    far_pit = np.asarray(
        [
            (x, y, -0.13 + noise)
            for x in np.arange(1.35, 1.86, 0.04)
            for y in np.arange(-0.30, 0.31, 0.04)
            for noise in (-0.002, 0.0, 0.002)
        ]
    )
    result = analyze_terrain_geometry(
        np.vstack((np.asarray(floor), near_noise, far_pit)),
        step_height=0.07,
        pit_depth=0.07,
        wall_height=0.23,
        min_region_cells=4,
        min_region_points=16,
    )
    assert result.valid
    assert result.obstacle_type == PIT
    assert 1.25 <= result.distance <= 1.90


def test_region_confidence_counts_anomaly_echoes_not_coincident_ground():
    """地面回波不虚增正障碍置信度，更多真实坑底回波应提高 PIT 置信度。"""
    floor = _dense_floor()
    coordinates = tuple(
        (x, y) for x in (0.51, 0.56) for y in (-0.025, 0.025)
    )
    high_echoes = np.asarray(
        [(x, y, 0.10) for x, y in coordinates for _ in range(2)]
    )
    coincident_ground = np.asarray([(x, y, 0.0) for x, y in coordinates])
    parameters = dict(
        step_height=0.07,
        pit_depth=0.07,
        min_region_cells=4,
        min_region_points=8,
    )
    baseline = analyze_terrain_geometry(
        np.vstack((floor, high_echoes)), **parameters
    )
    extra_ground = analyze_terrain_geometry(
        np.vstack((floor, high_echoes, coincident_ground)), **parameters
    )
    assert baseline.obstacle_type == STEP
    assert extra_ground.obstacle_type == STEP
    # 地面平面重拟合可能产生约 1e-5 的浮点差，但旧总点数公式会增加 0.008。
    assert abs(extra_ground.confidence - baseline.confidence) < 0.001

    weak_pit = np.asarray(
        [(x, y, -0.13) for x, y in coordinates for _ in range(2)]
    )
    dense_pit = np.asarray(
        [(x, y, -0.13) for x, y in coordinates for _ in range(4)]
    )
    weak_result = analyze_terrain_geometry(
        np.vstack((floor, weak_pit)), **parameters
    )
    dense_result = analyze_terrain_geometry(
        np.vstack((floor, dense_pit)), **parameters
    )
    assert weak_result.obstacle_type == PIT
    assert dense_result.obstacle_type == PIT
    assert dense_result.confidence > weak_result.confidence + 0.02


def test_dense_single_cell_flat_blob_cannot_use_narrow_pole_exception():
    """窄立柱召回门不能让单格密集飞点被降级误报成台阶。"""
    floor = _dense_floor()
    blob = np.asarray(
        [
            (0.625 + dx, 0.025 + dy, 0.30 + dz)
            for dx in (-0.008, -0.004, 0.0, 0.004, 0.008)
            for dy in (-0.008, 0.0, 0.008)
            for dz in (-0.002, 0.002)
        ]
    )
    result = analyze_terrain_geometry(np.vstack((floor, blob)))
    assert result.valid
    assert result.obstacle_type == CLEAR


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
    virtual = navigation_obstacle_points(
        np.vstack((floor, pit)), result, minimum_height_above_ground=0.05
    )
    assert len(virtual) > 0
    # 虚拟点位于拟合地面上方，能通过 Nav2 的绝对 z 过滤，不会把低回波静默漏掉。
    assert np.quantile(virtual[:, 2], 0.5) > result.ground_height


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


def test_connected_but_weak_depth_speckles_do_not_form_an_obstacle():
    """相邻栅格中各两个飞点仍不足以证明存在连续障碍表面。"""
    floor = _dense_floor()
    weak_cluster = np.asarray(
        [
            point
            for x, y in ((0.60, 0.00), (0.65, 0.00), (0.70, 0.00))
            for point in ((x, y, 0.30), (x, y, 0.31))
        ],
        dtype=np.float64,
    )
    result = analyze_terrain_geometry(np.vstack((floor, weak_cluster)))
    assert result.valid
    assert result.obstacle_type == CLEAR


def test_connected_region_preserves_cells_across_rounding_boundary():
    """接近栅格右边界的中值坐标不能与下一格四舍五入到同一索引。"""
    # x=0.099 属于 floor 索引 1，x=0.101 属于索引 2；旧 rint 实现会把二者都变成 2，
    # 字典覆盖后只剩两个格，从而漏掉窄而连续的障碍。
    cells = np.asarray(
        [
            (0.099, 0.0, 0.2, 0.2, 0.2, 4),
            (0.101, 0.0, 0.2, 0.2, 0.2, 4),
            (0.151, 0.0, 0.2, 0.2, 0.2, 4),
        ],
        dtype=np.float64,
    )
    region = _largest_connected_region(
        cells, np.ones(3, dtype=bool), cell_size=0.05
    )
    assert len(region) == 3


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


def test_competition_fourteen_degree_ramp_remains_ground_not_wall():
    """规则中的 11.3°/14° 坡面应拟合为地面坡度，而不是高墙或台阶。"""
    floor = _dense_floor()
    slope = np.tan(np.deg2rad(14.0))
    floor[:, 2] += slope * floor[:, 0]
    result = analyze_terrain_geometry(floor)
    assert result.valid
    assert result.obstacle_type == CLEAR
    assert abs(result.slope_pitch - np.deg2rad(14.0)) < 0.03
    # 合法坡面不能直接进入 Nav2 标障点云，否则局部代价地图会在坡顶封路。
    obstacle_points = navigation_obstacle_points(floor, result)
    assert len(obstacle_points) == 0


def test_nav2_cloud_keeps_low_step_relative_to_ground_plane():
    """base_link 下方的低台阶仍应按相对地面高度进入代价地图。"""
    floor = _dense_floor(z=-0.44)
    step = np.asarray(
        [
            (x, y, -0.32 + noise)
            for x in np.arange(0.60, 0.81, 0.04)
            for y in np.arange(-0.16, 0.17, 0.04)
            for noise in (-0.002, 0.002)
        ]
    )
    points = np.vstack((floor, step))
    result = analyze_terrain_geometry(points)
    assert result.valid
    selected = navigation_obstacle_points(points, result, minimum_height_above_ground=0.05)
    assert len(selected) > 0
    # 点在 base_link 下方仍合法；Nav2 YAML 的绝对 z 下限不能把它们漏掉。
    assert np.max(selected[:, 2]) < 0.0

"""Regression tests for fail-fast perception YAML validation."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from quadruped_perception.parameter_validation import (
    validate_fusion_parameters,
    validate_terrain_parameters,
    validate_vision_parameters,
)


CONFIG_ROOT = Path(__file__).parents[1] / "config"


def _parameters(file_name: str, node_name: str) -> dict:
    """Load the same raw mapping that ROS passes to a node from its shipped YAML."""
    config = yaml.safe_load((CONFIG_ROOT / file_name).read_text(encoding="utf-8"))
    return config[node_name]["ros__parameters"]


def test_shipped_perception_configuration_is_valid():
    """Every committed online profile must satisfy the executable startup contract."""
    terrain = _parameters("terrain.yaml", "terrain_analyzer")
    vision_file = "vision.yaml"
    vision = _parameters(vision_file, "vision_obstacle_detector")
    fusion = _parameters(vision_file, "perception_fusion")
    validate_terrain_parameters(terrain)
    validate_vision_parameters(vision)
    validate_fusion_parameters(fusion)


def test_vision_validation_reports_all_related_mistakes_at_once():
    """A launch error should identify the whole broken group, not one typo per restart."""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values.update(
        processing_hz=0.0,
        roi_top_ratio=0.9,
        roi_bottom_ratio=0.2,
        history_size=2,
        confirmation_frames=3,
        segmented_bar_max_gap_ratio=0.0,
        segmented_bar_max_gap_cv=3.0,
    )
    with pytest.raises(ValueError) as error:
        validate_vision_parameters(values)
    message = str(error.value)
    assert "processing_hz" in message
    assert "roi_top_ratio" in message
    assert "confirmation_frames" in message
    assert "segmented_bar_max_gap_ratio" in message
    assert "segmented_bar_max_gap_cv" in message


@pytest.mark.parametrize(
    ("name", "value"),
    (
        ("processing_hz", 30.01),
        ("processing_hz", 0.49),
        ("min_area_px", 0.5),
        ("adaptive_canny_sigma", 0.01),
        ("roi_top_ratio", 0.96),
        ("roi_bottom_ratio", 0.04),
        ("roi_side_margin_ratio", 0.46),
        ("max_bar_width_ratio", 0.09),
        ("max_bar_height_ratio", 0.01),
        ("segmented_bar_max_gap_ratio", 0.09),
        ("max_temporal_center_jitter", 0.009),
        ("max_temporal_center_jitter", 1.001),
        ("max_temporal_size_jitter", 0.009),
        ("max_temporal_size_jitter", 1.001),
        ("history_reset_timeout", 0.09),
        ("source_switch_timeout", 0.09),
        ("source_failure_cooldown", 0.09),
        ("source_failure_cooldown", 30.01),
    ),
)
def test_vision_validation_rejects_values_the_node_would_silently_clip(
    name, value
):
    """YAML 参数要么原样生效，要么启动失败，不得悄悄改成运行边界。"""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values[name] = value
    with pytest.raises(ValueError, match=name):
        validate_vision_parameters(values)


@pytest.mark.parametrize(
    ("name", "value"),
    (
        ("max_temporal_center_jitter", 0.01),
        ("max_temporal_center_jitter", 1.0),
        ("max_temporal_size_jitter", 0.01),
        ("max_temporal_size_jitter", 1.0),
    ),
)
def test_vision_validation_accepts_normalized_temporal_jitter_boundaries(
    name, value
):
    """归一化框的时序跳变阈值必须接受运行合同的两个闭区间端点。"""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values[name] = value
    validate_vision_parameters(values)


@pytest.mark.parametrize("name", ("morphology_size", "glare_dilation_size"))
def test_vision_validation_rejects_even_kernel_size(name):
    """形态学核必须显式配成奇数，不能在运行时自动加一。"""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values[name] = 4
    with pytest.raises(ValueError, match=name):
        validate_vision_parameters(values)


def test_vision_validation_rejects_glare_kernel_above_runtime_limit():
    """高光膘胀核大于 31 时必须启动失败，不得静默缩到 31。"""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values["glare_dilation_size"] = 33
    with pytest.raises(ValueError, match="glare_dilation_size"):
        validate_vision_parameters(values)


@pytest.mark.parametrize(
    "name",
    (
        "publish_debug_mask",
        "publish_annotated_image",
        "adaptive_canny",
        "illumination_normalization",
        "dual_illumination_color_mask",
        "suppress_specular_glare",
    ),
)
def test_vision_validation_rejects_quoted_boolean(name):
    """字符串 ``false`` 在 Python 中为真值，因此必须在参数合同边界拒绝。"""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values[name] = "false"
    with pytest.raises(ValueError, match=name):
        validate_vision_parameters(values)


def test_vision_validation_rejects_invalid_hsv_and_topics():
    """Reject non-portable topic names and inverted linear HSV components."""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values["image_topic_candidates"] = ["camera/image_raw"]
    values["orange_hsv_lower"] = [10, 200, 70]
    values["orange_hsv_upper"] = [20, 100, 255]
    with pytest.raises(ValueError) as error:
        validate_vision_parameters(values)
    assert "absolute ROS topic" in str(error.value)
    assert "orange_hsv_lower" in str(error.value)


def test_vision_validation_accepts_wrapped_hue_range():
    """Hue lower > upper denotes 179-to-0 wraparound, not an inverted range."""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values["orange_hsv_lower"] = [175, 80, 70]
    values["orange_hsv_upper"] = [12, 255, 255]
    validate_vision_parameters(values)


def test_vision_validation_rejects_inverted_value_range():
    """Hue may wrap, but the linear saturation/value bounds may never wrap."""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values["blue_hsv_lower"] = [175, 80, 240]
    values["blue_hsv_upper"] = [12, 255, 100]
    with pytest.raises(ValueError, match="blue_hsv_lower S/V"):
        validate_vision_parameters(values)


def test_terrain_validation_rejects_inverted_roi_and_thresholds():
    """Reject impossible 3-D windows and unordered height decisions together."""
    values = deepcopy(_parameters("terrain.yaml", "terrain_analyzer"))
    values.update(
        front_x_min=1.0,
        front_x_max=0.5,
        front_z_min=0.5,
        front_z_max=-0.5,
        warning_height=0.2,
        critical_height=0.1,
    )
    with pytest.raises(ValueError) as error:
        validate_terrain_parameters(values)
    message = str(error.value)
    assert "front ROI" in message
    assert "front_z_max" in message
    assert "critical_height" in message


@pytest.mark.parametrize(
    ("name", "value"),
    (
        ("processing_hz", 30.01),
        ("processing_hz", 0.49),
        ("lateral_half_width", -0.001),
        ("ground_percentile", 0.01),
        ("ground_percentile", 0.41),
        ("source_switch_timeout", 0.09),
        ("source_failure_cooldown", 0.09),
        ("source_geometry_failure_frames", 1),
        ("source_geometry_failure_frames", 31),
        ("ground_prior_max_age", 0.19),
        ("ground_prior_max_consecutive_conflicts", 0),
        ("ground_prior_max_height_shift", 0.02),
        ("grid_cell_size", 0.019),
        ("ground_height_bin", 0.009),
        ("pit_depth_threshold", 0.029),
        ("wall_height_threshold", 0.099),
        ("min_connected_region_cells", 1),
        ("min_connected_region_points", 3),
    ),
)
def test_terrain_validation_rejects_values_the_node_would_silently_clip(
    name, value
):
    """点云 YAML 的可接受边界必须与 TerrainAnalyzer 的实际运行边界相同。"""
    values = deepcopy(_parameters("terrain.yaml", "terrain_analyzer"))
    values[name] = value
    with pytest.raises(ValueError, match=name):
        validate_terrain_parameters(values)


def test_terrain_validation_rejects_roi_narrower_than_clear_ground_corridor():
    """实时 CLEAR 合同必须看到有物理宽度的落脚通道，零宽诊断 ROI 不可用于导航。"""
    values = deepcopy(_parameters("terrain.yaml", "terrain_analyzer"))
    values["lateral_half_width"] = 0.0
    with pytest.raises(ValueError, match="clear_ground_corridor_half_width"):
        validate_terrain_parameters(values)


def test_terrain_validation_rejects_nonportable_frame_and_impossible_point_gates():
    """TF frame syntax and point-count gates must describe an executable sensor pipeline."""
    values = deepcopy(_parameters("terrain.yaml", "terrain_analyzer"))
    values["target_frame"] = "/base_link"
    values["transform_max_points"] = 20
    values["min_valid_points"] = 30
    values["max_points"] = 40
    values["min_connected_region_points"] = 50
    values["nav2_obstacle_min_height_above_ground"] = 0.08
    with pytest.raises(ValueError) as error:
        validate_terrain_parameters(values)
    message = str(error.value)
    assert "target_frame" in message
    assert "transform_max_points" in message
    assert "min_connected_region_points" in message
    assert "nav2_obstacle_min_height_above_ground" in message


def test_fusion_validation_preserves_synchronization_window_contract():
    """Require a bounded queue and a fallback delay no shorter than sync slop."""
    values = deepcopy(_parameters("vision.yaml", "perception_fusion"))
    values["sync_slop"] = 0.3
    values["terrain_only_timeout"] = 0.1
    values["queue_size"] = 1
    with pytest.raises(ValueError) as error:
        validate_fusion_parameters(values)
    message = str(error.value)
    assert "queue_size" in message
    assert "terrain_only_timeout" in message


def test_fusion_validation_rejects_sync_window_below_runtime_minimum():
    """同步窗不得在校验后再被节点静默放大到 1 ms。"""
    values = deepcopy(_parameters("vision.yaml", "perception_fusion"))
    values["sync_slop"] = 0.0005
    with pytest.raises(ValueError, match="sync_slop"):
        validate_fusion_parameters(values)


@pytest.mark.parametrize(
    ("name", "value"),
    (("sync_slop", 0.501), ("queue_size", 101), ("terrain_only_timeout", 5.01)),
)
def test_fusion_validation_enforces_latency_and_memory_upper_bounds(name, value):
    """融合窗口和二次队列搜索必须保持适合运动机器人及 RK3588 的有界成本。"""
    values = deepcopy(_parameters("vision.yaml", "perception_fusion"))
    values[name] = value
    with pytest.raises(ValueError, match=name):
        validate_fusion_parameters(values)

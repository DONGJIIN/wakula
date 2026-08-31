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


def test_vision_validation_rejects_invalid_hsv_and_topics():
    """Reject non-portable topic names and inverted OpenCV color bounds."""
    values = deepcopy(_parameters("vision.yaml", "vision_obstacle_detector"))
    values["image_topic_candidates"] = ["camera/image_raw"]
    values["orange_hsv_lower"] = [30, 80, 70]
    values["orange_hsv_upper"] = [20, 255, 255]
    with pytest.raises(ValueError) as error:
        validate_vision_parameters(values)
    assert "absolute ROS topic" in str(error.value)
    assert "orange_hsv_lower" in str(error.value)


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

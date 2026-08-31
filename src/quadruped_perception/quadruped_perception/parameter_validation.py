"""Fail-fast validation for hardware-independent perception parameters.

ROS parameters often come from hand-edited YAML files.  Silently clipping an invalid ROI,
history length, frequency, or HSV range makes the node appear to run while producing results
that are almost impossible to diagnose.  These pure helpers validate the raw values before
the online nodes allocate subscriptions and timers.  They intentionally have no ROS imports,
so the same contract is exercised by CI and future offline calibration tools.
"""

from __future__ import annotations

from math import isfinite
from numbers import Real
from typing import Mapping, Sequence


VISION_PARAMETER_NAMES = (
    "image_topic",
    "image_topic_candidates",
    "debug_mask_topic",
    "annotated_image_topic",
    "processing_hz",
    "resize_width",
    "min_area_px",
    "min_area_ratio",
    "morphology_size",
    "edge_low_threshold",
    "edge_high_threshold",
    "adaptive_canny_sigma",
    "clahe_clip_limit",
    "clahe_grid_size",
    "glare_saturation_max",
    "glare_value_min",
    "glare_dilation_size",
    "roi_top_ratio",
    "roi_bottom_ratio",
    "roi_side_margin_ratio",
    "min_color_fill_ratio",
    "min_bar_aspect_ratio",
    "max_bar_width_ratio",
    "max_bar_height_ratio",
    "segmented_bar_max_gap_ratio",
    "segmented_bar_max_gap_cv",
    "min_image_quality",
    "history_size",
    "confirmation_frames",
    "max_temporal_center_jitter",
    "max_temporal_size_jitter",
    "temporal_match_ratio",
    "min_temporal_iou",
    "history_reset_timeout",
    "source_switch_timeout",
    "orange_hsv_lower",
    "orange_hsv_upper",
    "blue_hsv_lower",
    "blue_hsv_upper",
)

TERRAIN_PARAMETER_NAMES = (
    "input_topic",
    "input_topic_candidates",
    "target_frame",
    "processing_hz",
    "transform_timeout",
    "transform_max_points",
    "max_points",
    "nav2_cloud_max_points",
    "nav2_obstacle_min_height_above_ground",
    "front_x_min",
    "front_x_max",
    "lateral_half_width",
    "front_z_min",
    "front_z_max",
    "ground_percentile",
    "warning_height",
    "critical_height",
    "max_slope",
    "max_roughness",
    "min_valid_points",
    "source_switch_timeout",
    "grid_cell_size",
    "ground_height_bin",
    "pit_depth_threshold",
    "wall_height_threshold",
    "bar_min_clearance",
    "min_connected_region_cells",
    "min_connected_region_points",
)

FUSION_PARAMETER_NAMES = (
    "sync_slop",
    "queue_size",
    "vision_min_confidence",
    "vision_center_margin",
    "terrain_only_timeout",
)


def _number(values: Mapping[str, object], name: str, errors: list[str]) -> float:
    """Return a finite numeric parameter or append one precise validation error."""
    value = values.get(name)
    if isinstance(value, bool) or not isinstance(value, Real):
        errors.append(f"{name} must be a finite number")
        return 0.0
    result = float(value)
    if not isfinite(result):
        errors.append(f"{name} must be finite")
        return 0.0
    return result


def _integer(values: Mapping[str, object], name: str, errors: list[str]) -> int:
    """Return an exact integer; reject floats such as 3.5 instead of truncating them."""
    value = _number(values, name, errors)
    if not float(value).is_integer():
        errors.append(f"{name} must be an integer")
    return int(value)


def _range(
    values: Mapping[str, object],
    name: str,
    lower: float,
    upper: float,
    errors: list[str],
) -> float:
    value = _number(values, name, errors)
    if not lower <= value <= upper:
        errors.append(f"{name} must be in [{lower}, {upper}]")
    return value


def _positive(values: Mapping[str, object], name: str, errors: list[str]) -> float:
    value = _number(values, name, errors)
    if value <= 0.0:
        errors.append(f"{name} must be > 0")
    return value


def _topic(name: str, value: object, errors: list[str], *, allow_empty: bool) -> None:
    """Validate the absolute topic convention used by the portable launch profiles."""
    if not isinstance(value, str):
        errors.append(f"{name} must be a string")
        return
    if not value and allow_empty:
        return
    if not value.startswith("/") or value.endswith("/") or "//" in value:
        errors.append(f"{name} must be an absolute ROS topic without a trailing slash")
    if any(character.isspace() for character in value):
        errors.append(f"{name} must not contain whitespace")


def _topic_source(
    values: Mapping[str, object], override_name: str, candidates_name: str, errors: list[str]
) -> None:
    override = values.get(override_name)
    _topic(override_name, override, errors, allow_empty=True)
    candidates = values.get(candidates_name)
    if isinstance(candidates, (str, bytes)) or not isinstance(candidates, Sequence):
        errors.append(f"{candidates_name} must be a non-empty sequence")
        return
    if not candidates and not override:
        errors.append(f"{candidates_name} must not be empty when {override_name} is empty")
    for index, topic in enumerate(candidates):
        _topic(f"{candidates_name}[{index}]", topic, errors, allow_empty=False)


def _hsv_triplet(
    values: Mapping[str, object], name: str, errors: list[str]
) -> tuple[int, int, int]:
    raw = values.get(name)
    if isinstance(raw, (str, bytes)) or not isinstance(raw, Sequence) or len(raw) != 3:
        errors.append(f"{name} must contain exactly [H, S, V]")
        return (0, 0, 0)
    result: list[int] = []
    for index, (item, maximum) in enumerate(zip(raw, (179, 255, 255))):
        if isinstance(item, bool) or not isinstance(item, Real) or not float(item).is_integer():
            errors.append(f"{name}[{index}] must be an integer")
            result.append(0)
            continue
        integer = int(item)
        if not 0 <= integer <= maximum:
            errors.append(f"{name}[{index}] must be in [0, {maximum}]")
        result.append(integer)
    return tuple(result)


def _raise(group: str, errors: list[str]) -> None:
    """Raise one stable message so launch logs show all related mistakes at once."""
    if errors:
        raise ValueError(f"invalid {group} parameters: " + "; ".join(dict.fromkeys(errors)))


def validate_vision_parameters(values: Mapping[str, object]) -> None:
    """Validate camera processing, ROI, quality, temporal, and HSV parameters."""
    errors: list[str] = []
    _topic_source(values, "image_topic", "image_topic_candidates", errors)
    _topic("debug_mask_topic", values.get("debug_mask_topic"), errors, allow_empty=False)
    _topic("annotated_image_topic", values.get("annotated_image_topic"), errors, allow_empty=False)
    _positive(values, "processing_hz", errors)
    if _integer(values, "resize_width", errors) < 0:
        errors.append("resize_width must be >= 0 (0 disables resizing)")
    _positive(values, "min_area_px", errors)
    _range(values, "min_area_ratio", 0.0, 0.10, errors)
    if _integer(values, "morphology_size", errors) < 1:
        errors.append("morphology_size must be >= 1")
    edge_low = _integer(values, "edge_low_threshold", errors)
    edge_high = _integer(values, "edge_high_threshold", errors)
    if not 0 <= edge_low < edge_high <= 255:
        errors.append("edge thresholds must satisfy 0 <= low < high <= 255")
    _range(values, "adaptive_canny_sigma", 0.01, 0.99, errors)
    _positive(values, "clahe_clip_limit", errors)
    if _integer(values, "clahe_grid_size", errors) < 2:
        errors.append("clahe_grid_size must be >= 2")
    _range(values, "glare_saturation_max", 0, 255, errors)
    _range(values, "glare_value_min", 0, 255, errors)
    if _integer(values, "glare_dilation_size", errors) < 1:
        errors.append("glare_dilation_size must be >= 1")
    roi_top = _range(values, "roi_top_ratio", 0.0, 1.0, errors)
    roi_bottom = _range(values, "roi_bottom_ratio", 0.0, 1.0, errors)
    if roi_top >= roi_bottom:
        errors.append("roi_top_ratio must be smaller than roi_bottom_ratio")
    _range(values, "roi_side_margin_ratio", 0.0, 0.49, errors)
    _range(values, "min_color_fill_ratio", 0.0, 1.0, errors)
    if _number(values, "min_bar_aspect_ratio", errors) < 1.0:
        errors.append("min_bar_aspect_ratio must be >= 1")
    _range(values, "max_bar_width_ratio", 0.0, 1.0, errors)
    _range(values, "max_bar_height_ratio", 0.0, 1.0, errors)
    _positive(values, "segmented_bar_max_gap_ratio", errors)
    _range(values, "segmented_bar_max_gap_cv", 0.0, 2.0, errors)
    _range(values, "min_image_quality", 0.0, 1.0, errors)
    history_size = _integer(values, "history_size", errors)
    confirmation_frames = _integer(values, "confirmation_frames", errors)
    if history_size < 1:
        errors.append("history_size must be >= 1")
    if not 1 <= confirmation_frames <= max(1, history_size):
        errors.append("confirmation_frames must be in [1, history_size]")
    _positive(values, "max_temporal_center_jitter", errors)
    _positive(values, "max_temporal_size_jitter", errors)
    _range(values, "temporal_match_ratio", 0.0, 1.0, errors)
    _range(values, "min_temporal_iou", 0.0, 1.0, errors)
    _positive(values, "history_reset_timeout", errors)
    _positive(values, "source_switch_timeout", errors)
    for prefix in ("orange", "blue"):
        lower = _hsv_triplet(values, f"{prefix}_hsv_lower", errors)
        upper = _hsv_triplet(values, f"{prefix}_hsv_upper", errors)
        if any(low > high for low, high in zip(lower, upper)):
            errors.append(f"{prefix}_hsv_lower must not exceed {prefix}_hsv_upper")
    _raise("vision", errors)


def validate_terrain_parameters(values: Mapping[str, object]) -> None:
    """Validate point-cloud resource limits, ROI geometry, and classification thresholds."""
    errors: list[str] = []
    _topic_source(values, "input_topic", "input_topic_candidates", errors)
    target_frame = values.get("target_frame")
    if (
        not isinstance(target_frame, str)
        or not target_frame
        or any(c.isspace() for c in target_frame)
    ):
        errors.append("target_frame must be a non-empty frame without whitespace")
    _positive(values, "processing_hz", errors)
    if _number(values, "transform_timeout", errors) < 0.0:
        errors.append("transform_timeout must be >= 0")
    if _integer(values, "transform_max_points", errors) < 0:
        errors.append("transform_max_points must be >= 0 (0 disables the limit)")
    for name in ("max_points", "nav2_cloud_max_points", "min_valid_points"):
        if _integer(values, name, errors) < 1:
            errors.append(f"{name} must be >= 1")
    if _number(values, "nav2_obstacle_min_height_above_ground", errors) < 0.0:
        errors.append("nav2_obstacle_min_height_above_ground must be >= 0")
    x_min = _number(values, "front_x_min", errors)
    x_max = _number(values, "front_x_max", errors)
    if x_min < 0.0 or x_max <= x_min:
        errors.append("front ROI must satisfy 0 <= front_x_min < front_x_max")
    _positive(values, "lateral_half_width", errors)
    z_min = _number(values, "front_z_min", errors)
    z_max = _number(values, "front_z_max", errors)
    if z_max <= z_min:
        errors.append("front_z_max must exceed front_z_min")
    _range(values, "ground_percentile", 0.0, 1.0, errors)
    warning = _positive(values, "warning_height", errors)
    critical = _positive(values, "critical_height", errors)
    if critical <= warning:
        errors.append("critical_height must exceed warning_height")
    for name in (
        "max_slope",
        "max_roughness",
        "source_switch_timeout",
        "grid_cell_size",
        "ground_height_bin",
        "pit_depth_threshold",
        "wall_height_threshold",
    ):
        _positive(values, name, errors)
    if _number(values, "bar_min_clearance", errors) < warning:
        errors.append("bar_min_clearance must be >= warning_height")
    for name in ("min_connected_region_cells", "min_connected_region_points"):
        if _integer(values, name, errors) < 1:
            errors.append(f"{name} must be >= 1")
    _raise("terrain", errors)


def validate_fusion_parameters(values: Mapping[str, object]) -> None:
    """Validate bounded time synchronization and visual association settings."""
    errors: list[str] = []
    sync_slop = _positive(values, "sync_slop", errors)
    if _integer(values, "queue_size", errors) < 2:
        errors.append("queue_size must be >= 2")
    _range(values, "vision_min_confidence", 0.0, 1.0, errors)
    _range(values, "vision_center_margin", 0.0, 0.49, errors)
    fallback = _positive(values, "terrain_only_timeout", errors)
    if fallback < sync_slop:
        errors.append("terrain_only_timeout must be >= sync_slop")
    _raise("fusion", errors)

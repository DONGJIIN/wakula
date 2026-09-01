"""Pure startup validation for navigation-safety and autonomous-task parameters."""

from __future__ import annotations

from math import isfinite, pi
from numbers import Real
from typing import Mapping, Sequence


SAFETY_PARAMETER_NAMES = (
    "step_threshold", "climb_threshold", "stop_threshold", "max_slope",
    "max_roughness", "sensor_timeout", "fused_min_confidence", "vision_timeout",
    "vision_min_confidence", "vision_center_margin", "vision_speed_scale",
    "hard_stop_distance", "hazard_approach_speed", "future_stamp_tolerance",
    "status_log_period", "min_points", "clear_confirmation_frames",
    "hazard_confirmation_frames", "name_confirmation_frames", "name_clear_frames",
    "output_frame", "legacy_features_enabled",
)

SPEED_GATE_PARAMETER_NAMES = (
    "input_topic", "output_topic", "command_timeout", "assessment_timeout",
    "navigation_health_timeout", "default_speed_limit", "scan_topic", "scan_timeout",
    "emergency_stop_distance", "emergency_sector_half_angle",
    "emergency_scan_min_valid_ratio", "emergency_scan_max_invalid_gap_angle",
    "alignment_guidance_timeout", "alignment_max_angular_speed",
    "stopped_rotation_linear_tolerance", "rotation_recovery_timeout",
    "autonomy_lease_timeout",
)


def _number(values: Mapping[str, object], name: str, errors: list[str]) -> float:
    value = values.get(name)
    if isinstance(value, bool) or not isinstance(value, Real):
        errors.append(f"{name} must be a finite number")
        return 0.0
    result = float(value)
    if not isfinite(result):
        errors.append(f"{name} must be finite")
        return 0.0
    return result


def _positive(values: Mapping[str, object], name: str, errors: list[str]) -> float:
    value = _number(values, name, errors)
    if value <= 0.0:
        errors.append(f"{name} must be > 0")
    return value


def _unit(values: Mapping[str, object], name: str, errors: list[str]) -> float:
    value = _number(values, name, errors)
    if not 0.0 <= value <= 1.0:
        errors.append(f"{name} must be in [0, 1]")
    return value


def _integer(values: Mapping[str, object], name: str, errors: list[str], minimum: int = 1) -> int:
    value = _number(values, name, errors)
    if not value.is_integer():
        errors.append(f"{name} must be an integer")
    result = int(value)
    if result < minimum:
        errors.append(f"{name} must be >= {minimum}")
    return result


def _topic(values: Mapping[str, object], name: str, errors: list[str]) -> str:
    value = values.get(name)
    if not isinstance(value, str):
        errors.append(f"{name} must be a string")
        return ""
    if not value.startswith("/") or value.endswith("/") or "//" in value:
        errors.append(f"{name} must be an absolute ROS topic without a trailing slash")
    if any(character.isspace() for character in value):
        errors.append(f"{name} must not contain whitespace")
    return value


def _raise(group: str, errors: list[str]) -> None:
    if errors:
        raise ValueError(f"invalid {group} parameters: " + "; ".join(dict.fromkeys(errors)))


def validate_safety_parameters(values: Mapping[str, object]) -> None:
    """Validate fail-closed terrain classification and temporal filtering settings."""
    errors: list[str] = []
    step = _positive(values, "step_threshold", errors)
    climb = _positive(values, "climb_threshold", errors)
    stop = _positive(values, "stop_threshold", errors)
    if not step < climb < stop:
        errors.append("height thresholds must satisfy step < climb < stop")
    for name in (
        "max_slope", "max_roughness", "sensor_timeout", "vision_timeout",
        "hard_stop_distance", "future_stamp_tolerance", "status_log_period",
    ):
        _positive(values, name, errors)
    for name in (
        "fused_min_confidence", "vision_min_confidence", "vision_center_margin",
        "vision_speed_scale", "hazard_approach_speed",
    ):
        _unit(values, name, errors)
    for name in (
        "min_points", "clear_confirmation_frames", "hazard_confirmation_frames",
        "name_confirmation_frames", "name_clear_frames",
    ):
        _integer(values, name, errors)
    output_frame = values.get("output_frame")
    if (
        not isinstance(output_frame, str)
        or not output_frame
        or any(c.isspace() for c in output_frame)
    ):
        errors.append("output_frame must be a non-empty frame without whitespace")
    if not isinstance(values.get("legacy_features_enabled"), bool):
        errors.append("legacy_features_enabled must be boolean")
    _raise("terrain safety", errors)


def validate_guidance_parameters(values: Mapping[str, object]) -> None:
    """Validate approach/align/handoff ordering before creating the guidance timer."""
    errors: list[str] = []
    input_timeout = _positive(values, "input_timeout", errors)
    approach = _positive(values, "approach_start_distance", errors)
    handoff = _positive(values, "handoff_distance", errors)
    if approach <= handoff:
        errors.append("approach_start_distance must exceed handoff_distance")
    alignment = _positive(values, "alignment_tolerance", errors)
    if alignment > pi:
        errors.append("alignment_tolerance must be <= pi")
    for name in ("max_lateral_target", "minimum_slope_for_handoff"):
        _positive(values, name, errors)
    for name in ("approach_speed_limit", "alignment_speed_limit", "target_smoothing_alpha"):
        _unit(values, name, errors)
    for name in ("distance_hysteresis", "angle_hysteresis"):
        if _number(values, name, errors) < 0.0:
            errors.append(f"{name} must be >= 0")
    _integer(values, "ready_confirmation_frames", errors)
    _integer(values, "type_confirmation_frames", errors)
    if input_timeout <= 0.0:
        errors.append("input_timeout must permit watchdog operation")
    _raise("traversal guidance", errors)


def validate_speed_gate_parameters(values: Mapping[str, object]) -> None:
    """Validate the final Twist ownership, heartbeat, and emergency-scan contract."""
    errors: list[str] = []
    input_topic = _topic(values, "input_topic", errors)
    output_topic = _topic(values, "output_topic", errors)
    _topic(values, "scan_topic", errors)
    if input_topic and input_topic == output_topic:
        errors.append(
            "input_topic and output_topic must differ to prevent a velocity feedback loop"
        )
    for name in (
        "command_timeout", "assessment_timeout", "navigation_health_timeout",
        "scan_timeout", "emergency_stop_distance", "alignment_guidance_timeout",
        "alignment_max_angular_speed", "rotation_recovery_timeout",
        "autonomy_lease_timeout",
    ):
        _positive(values, name, errors)
    _unit(values, "default_speed_limit", errors)
    sector = _positive(values, "emergency_sector_half_angle", errors)
    if sector > pi:
        errors.append("emergency_sector_half_angle must be <= pi")
    valid_ratio = _unit(values, "emergency_scan_min_valid_ratio", errors)
    if valid_ratio <= 0.0:
        errors.append("emergency_scan_min_valid_ratio must be > 0")
    invalid_gap = _positive(
        values, "emergency_scan_max_invalid_gap_angle", errors
    )
    # A larger blind arc would no longer be "near 360 degree" protection for a
    # rotating footprint.  Supporting such hardware requires a different sensor or a
    # footprint/trajectory-aware contract, not a permissive parameter override.
    if invalid_gap > 0.35:
        errors.append("emergency_scan_max_invalid_gap_angle must be <= 0.35")
    if _number(values, "stopped_rotation_linear_tolerance", errors) < 0.0:
        errors.append("stopped_rotation_linear_tolerance must be >= 0")
    _raise("navigation speed gate", errors)


def validate_mission_parameters(values: Mapping[str, object]) -> None:
    """Validate autonomous state-machine invariants that are independent of robot hardware."""
    errors: list[str] = []
    for name, value in values.items():
        if name in ("autostart", "expected_obstacle_ids", "observation_frame"):
            continue
        if isinstance(value, bool) or not isinstance(value, Real) or not isfinite(float(value)):
            errors.append(f"{name} must be a finite number")
        elif float(value) < 0.0:
            errors.append(f"{name} must be >= 0")
    if not isinstance(values.get("autostart"), bool):
        errors.append("autostart must be boolean")
    observation_frame = values.get("observation_frame")
    if (
        not isinstance(observation_frame, str)
        or not observation_frame
        or observation_frame != observation_frame.strip()
        or observation_frame.startswith("/")
        or observation_frame.endswith("/")
        or any(character.isspace() for character in observation_frame)
    ):
        errors.append(
            "observation_frame must be canonical without surrounding slash or whitespace"
        )
    integer_names = (
        "frontier_minimum_cells", "approach_stall_handoff_count",
        "obstacle_confirmation_frames", "semantic_confirmation_votes",
        "semantic_recent_window", "semantic_verification_max_attempts",
        "empty_frontier_confirmations", "maximum_search_turns",
    )
    for name in integer_names:
        _integer(values, name, errors)
    minimum_distance = _number(values, "frontier_minimum_distance", errors)
    maximum_distance = _number(values, "frontier_maximum_distance", errors)
    if maximum_distance <= minimum_distance:
        errors.append("frontier_maximum_distance must exceed frontier_minimum_distance")
    observation_distance = _number(values, "semantic_observation_distance", errors)
    confirmation_distance = _number(values, "semantic_confirmation_distance", errors)
    if observation_distance < confirmation_distance:
        errors.append("semantic_observation_distance must be >= semantic_confirmation_distance")
    votes = _integer(values, "semantic_confirmation_votes", errors)
    window = _integer(values, "semantic_recent_window", errors)
    if votes > window:
        errors.append("semantic_confirmation_votes must not exceed semantic_recent_window")
    _unit(values, "minimum_obstacle_confidence", errors)
    stable_duration = _number(values, "post_traversal_stable_duration", errors)
    verification_timeout = _positive(values, "post_traversal_verification_timeout", errors)
    if stable_duration >= verification_timeout:
        errors.append("post_traversal_stable_duration must be below verification timeout")
    for name in (
        "map_timeout", "guidance_timeout", "startup_sensor_settle_time",
        "goal_timeout", "traversal_timeout", "traversal_progress_timeout",
        "action_response_timeout", "action_cancel_timeout",
        "controller_wait_timeout", "safety_geometry_stale_seconds",
        "safety_guidance_spatial_tolerance",
        "navigation_health_timeout",
        "navigation_health_recovery_duration",
        "nav_stall_timeout", "return_nav_stall_timeout",
        "odom_progress_timeout",
        "mission_timeout",
        "return_time_reserve", "inventory_log_period",
        "obstacle_revisit_max_cooldown",
        "handoff_fallback_viewpoint_tolerance",
        "handoff_fallback_view_heading_tolerance",
        "handoff_fallback_max_distance", "handoff_fallback_max_lateral",
        "handoff_fallback_spatial_tolerance", "handoff_alignment_tolerance",
        "direct_handoff_max_distance",
        "approach_stall_handoff_max_distance",
        "approach_stall_handoff_max_lateral",
        "approach_stall_handoff_max_heading_error",
        "failed_entry_turn_angle", "failed_entry_settle_time",
        "failed_entry_memory_duration", "failed_entry_station_tolerance",
        "failed_entry_heading_tolerance", "failed_entry_escape_distance",
        "traversal_ready_max_distance", "traversal_ready_max_lateral",
        "traversal_ready_max_heading_error",
    ):
        _positive(values, name, errors)
    mission_timeout = _number(values, "mission_timeout", errors)
    return_reserve = _number(values, "return_time_reserve", errors)
    if return_reserve >= mission_timeout:
        errors.append("return_time_reserve must be below mission_timeout")
    revisit_cooldown = _number(values, "obstacle_revisit_cooldown", errors)
    revisit_maximum = _number(values, "obstacle_revisit_max_cooldown", errors)
    if revisit_maximum < revisit_cooldown:
        errors.append(
            "obstacle_revisit_max_cooldown must be >= obstacle_revisit_cooldown"
        )
    # These quantities are divisors, motion-evidence thresholds or spatial identity
    # radii.  Zero silently disables their invariant (for example, localization noise
    # would become perpetual Nav2 progress), so they are stricter than parameters where
    # zero intentionally means "disabled".
    for name in (
        "nav_progress_translation", "nav_progress_rotation",
        "post_traversal_minimum_displacement", "post_traversal_beyond_margin",
        "post_traversal_stable_translation", "post_traversal_stable_rotation",
        "return_home_tolerance", "frontier_exclusion_radius",
        "coverage_visit_radius", "obstacle_failure_radius",
        "obstacle_lock_radius", "semantic_verification_lock_radius",
        "completed_obstacle_radius", "obstacle_revisit_position_tolerance",
    ):
        _positive(values, name, errors)
    traversal_timeout = _number(values, "traversal_timeout", errors)
    progress_timeout = _number(values, "traversal_progress_timeout", errors)
    if progress_timeout >= traversal_timeout:
        errors.append("traversal_progress_timeout must be below traversal_timeout")
    minimum_displacement = _number(
        values, "post_traversal_minimum_displacement", errors
    )
    beyond_margin = _number(values, "post_traversal_beyond_margin", errors)
    if beyond_margin >= minimum_displacement:
        errors.append(
            "post_traversal_beyond_margin must be below "
            "post_traversal_minimum_displacement"
        )
    fallback_distance = _number(
        values, "handoff_fallback_max_distance", errors
    )
    fallback_lateral = _number(
        values, "handoff_fallback_max_lateral", errors
    )
    direct_distance = _number(values, "direct_handoff_max_distance", errors)
    stall_distance = _number(
        values, "approach_stall_handoff_max_distance", errors
    )
    stall_lateral = _number(
        values, "approach_stall_handoff_max_lateral", errors
    )
    handoff_heading = _number(
        values, "handoff_alignment_tolerance", errors
    )
    stall_heading = _number(
        values, "approach_stall_handoff_max_heading_error", errors
    )
    if direct_distance > stall_distance:
        errors.append(
            "direct_handoff_max_distance must not exceed "
            "approach_stall_handoff_max_distance"
        )
    if fallback_distance > stall_distance:
        errors.append(
            "handoff_fallback_max_distance must not exceed "
            "approach_stall_handoff_max_distance"
        )
    if fallback_lateral > stall_lateral:
        errors.append(
            "handoff_fallback_max_lateral must not exceed "
            "approach_stall_handoff_max_lateral"
        )
    if handoff_heading > stall_heading:
        errors.append(
            "handoff_alignment_tolerance must not exceed "
            "approach_stall_handoff_max_heading_error"
        )
    ready_distance = _number(values, "traversal_ready_max_distance", errors)
    ready_lateral = _number(values, "traversal_ready_max_lateral", errors)
    ready_heading = _number(
        values, "traversal_ready_max_heading_error", errors
    )
    if ready_distance > fallback_distance:
        errors.append(
            "traversal_ready_max_distance must not exceed "
            "handoff_fallback_max_distance"
        )
    if ready_distance > stall_distance:
        errors.append(
            "traversal_ready_max_distance must not exceed the PREPARING "
            "distance envelope"
        )
    if ready_lateral > fallback_lateral:
        errors.append(
            "traversal_ready_max_lateral must not exceed "
            "handoff_fallback_max_lateral"
        )
    if ready_lateral > stall_lateral:
        errors.append(
            "traversal_ready_max_lateral must not exceed the PREPARING "
            "lateral envelope"
        )
    if ready_heading > handoff_heading:
        errors.append(
            "traversal_ready_max_heading_error must not exceed "
            "handoff_alignment_tolerance"
        )
    if ready_heading > stall_heading:
        errors.append(
            "traversal_ready_max_heading_error must not exceed the PREPARING "
            "heading envelope"
        )
    for name in (
        "minimum_alignment_command_angle", "pre_alignment_trigger_angle",
        "pre_alignment_max_step", "handoff_alignment_tolerance",
        "approach_stall_handoff_max_heading_error", "semantic_verification_turn_angle",
        "traversal_ready_max_heading_error",
        "search_turn_angle", "post_traversal_stable_rotation",
        "handoff_fallback_view_heading_tolerance",
        "failed_entry_turn_angle",
        "failed_entry_heading_tolerance",
    ):
        if _number(values, name, errors) > pi:
            errors.append(f"{name} must be <= pi")
    obstacle_ids = values.get("expected_obstacle_ids")
    if (
        isinstance(obstacle_ids, (str, bytes))
        or not isinstance(obstacle_ids, Sequence)
        or not obstacle_ids
    ):
        errors.append("expected_obstacle_ids must be a non-empty sequence")
    elif any(not isinstance(item, str) or not item for item in obstacle_ids):
        errors.append("expected_obstacle_ids must contain non-empty strings")
    elif len(set(obstacle_ids)) != len(obstacle_ids):
        errors.append("expected_obstacle_ids must not contain duplicates")
    _raise("autonomous mission", errors)

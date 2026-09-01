"""SLAM/Nav2 health-node startup parameter validation.

The health and readiness monitors decide whether autonomous velocity is allowed and whether
Nav2 may leave its inactive lifecycle state.  A malformed topic, frame, scan contract, or
timeout therefore must stop the monitor at startup; silently clipping it would turn a clear
configuration error into an ambiguous runtime wait or an unsafe healthy result.

This module intentionally has no ROS imports.  CI, launch-time nodes, and future deployment
checkers can consequently share exactly the same validation rules.
"""

from __future__ import annotations

from math import isfinite, pi
from numbers import Real
from typing import Mapping


HEALTH_PARAMETER_NAMES = (
    "global_frame",
    "base_frame",
    "sensor_timeout",
    "minimum_scan_valid_ratio",
    "minimum_scan_samples",
    "minimum_scan_field_of_view",
    "expected_odom_frame",
    "max_xy_covariance",
    "max_yaw_covariance",
    "max_odom_jump",
    "max_odom_yaw_jump",
    "odom_jump_recovery_samples",
    "future_stamp_tolerance",
)

READINESS_PARAMETER_NAMES = (
    "global_frame",
    "base_frame",
    "scan_topic",
    "odom_topic",
    "sensor_timeout",
    "future_stamp_tolerance",
    "minimum_scan_valid_ratio",
    "minimum_scan_samples",
    "minimum_scan_field_of_view",
    "max_xy_covariance",
    "max_yaw_covariance",
    "max_odom_jump",
    "max_odom_yaw_jump",
    "odom_jump_recovery_samples",
    "expected_odom_frame",
    "lifecycle_service",
    "recover_slam_toolbox",
    "slam_lifecycle_node",
    "slam_recovery_period",
    "slam_recovery_startup_grace",
    "service_request_timeout",
)


def _number(values: Mapping[str, object], name: str, errors: list[str]) -> float:
    """Return a finite number without accepting booleans as Python integers."""
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


def _integer(
    values: Mapping[str, object], name: str, errors: list[str], minimum: int
) -> int:
    value = _number(values, name, errors)
    if not value.is_integer():
        errors.append(f"{name} must be an integer")
    result = int(value)
    if result < minimum:
        errors.append(f"{name} must be >= {minimum}")
    return result


def _frame(values: Mapping[str, object], name: str, errors: list[str]) -> str:
    """Validate a TF frame ID; unlike a topic, it must not begin with a slash."""
    value = values.get(name)
    if not isinstance(value, str) or not value:
        errors.append(f"{name} must be a non-empty frame")
        return ""
    if value.startswith("/") or value.endswith("/") or any(c.isspace() for c in value):
        errors.append(f"{name} must be a frame without leading/trailing slash or whitespace")
    return value


def _absolute_name(values: Mapping[str, object], name: str, errors: list[str]) -> str:
    """Validate absolute ROS topic, service, and fully-qualified node names."""
    value = values.get(name)
    if not isinstance(value, str):
        errors.append(f"{name} must be a string")
        return ""
    if not value.startswith("/") or value == "/" or value.endswith("/") or "//" in value:
        errors.append(f"{name} must be an absolute ROS name without a trailing slash")
    if any(c.isspace() for c in value):
        errors.append(f"{name} must not contain whitespace")
    return value


def _common_scan_and_odom_contract(
    values: Mapping[str, object], errors: list[str]
) -> None:
    """Validate invariants deliberately shared by startup and runtime monitors."""
    _frame(values, "global_frame", errors)
    _frame(values, "base_frame", errors)
    _frame(values, "expected_odom_frame", errors)
    _positive(values, "sensor_timeout", errors)

    valid_ratio = _number(values, "minimum_scan_valid_ratio", errors)
    if not 0.0 < valid_ratio <= 1.0:
        errors.append("minimum_scan_valid_ratio must be in (0, 1]")
    _integer(values, "minimum_scan_samples", errors, 2)

    field_of_view = _positive(values, "minimum_scan_field_of_view", errors)
    if field_of_view > 2.0 * pi + 0.05:
        errors.append("minimum_scan_field_of_view must not exceed 2*pi")

    if _number(values, "max_xy_covariance", errors) < 0.0:
        errors.append("max_xy_covariance must be >= 0")
    if _number(values, "max_yaw_covariance", errors) < 0.0:
        errors.append("max_yaw_covariance must be >= 0")
    future_tolerance = _number(values, "future_stamp_tolerance", errors)
    if future_tolerance < 0.0:
        errors.append("future_stamp_tolerance must be >= 0")


def _raise(group: str, errors: list[str]) -> None:
    """Aggregate related mistakes so one launch attempt reports the whole YAML group."""
    if errors:
        raise ValueError(f"invalid {group} parameters: " + "; ".join(dict.fromkeys(errors)))


def validate_navigation_health_parameters(values: Mapping[str, object]) -> None:
    """Validate runtime health thresholds before creating the safety publisher."""
    errors: list[str] = []
    _common_scan_and_odom_contract(values, errors)
    _positive(values, "max_odom_jump", errors)
    yaw_jump = _positive(values, "max_odom_yaw_jump", errors)
    if yaw_jump > pi:
        errors.append("max_odom_yaw_jump must be <= pi")
    _integer(values, "odom_jump_recovery_samples", errors, 1)
    _raise("navigation health", errors)


def validate_nav2_readiness_parameters(values: Mapping[str, object]) -> None:
    """Validate Nav2 activation and optional SLAM lifecycle recovery configuration."""
    errors: list[str] = []
    _common_scan_and_odom_contract(values, errors)
    _positive(values, "max_odom_jump", errors)
    yaw_jump = _positive(values, "max_odom_yaw_jump", errors)
    if yaw_jump > pi:
        errors.append("max_odom_yaw_jump must be <= pi")
    _integer(values, "odom_jump_recovery_samples", errors, 1)
    for name in ("scan_topic", "odom_topic", "lifecycle_service", "slam_lifecycle_node"):
        _absolute_name(values, name, errors)
    if not isinstance(values.get("recover_slam_toolbox"), bool):
        errors.append("recover_slam_toolbox must be boolean")
    _positive(values, "slam_recovery_period", errors)
    if _number(values, "slam_recovery_startup_grace", errors) < 0.0:
        errors.append("slam_recovery_startup_grace must be >= 0")
    _positive(values, "service_request_timeout", errors)
    _raise("Nav2 readiness", errors)

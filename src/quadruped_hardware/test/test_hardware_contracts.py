"""Unit tests for safety and mock vendor-SDK contracts."""

import math

import rclpy
from std_srvs.srv import SetBool

from quadruped_hardware.mock_sdk_adapter import mock_traversal_state
from quadruped_hardware.system_safety_supervisor import (
    SafetyLimits,
    SystemSafetySupervisor,
    evaluate_safety,
    joint_state_is_valid,
    joint_positions_within_limit,
    quaternion_to_roll_pitch,
)
from quadruped_interfaces.msg import CrossingStatus


def test_quaternion_conversion_rejects_invalid_and_tracks_pitch():
    assert quaternion_to_roll_pitch(0.0, 0.0, 0.0, 0.0) is None
    orientation = quaternion_to_roll_pitch(
        0.0, math.sin(0.2), 0.0, math.cos(0.2)
    )
    assert abs(orientation[0]) < 1e-6
    assert abs(orientation[1] - 0.4) < 1e-6


def test_optional_sources_start_clear_but_seen_stale_source_stops():
    limits = SafetyLimits(sensor_timeout=1.0)
    assert evaluate_safety(
        False, None, None, None, (None, None, None), limits
    ) == (False, ())
    stop, reasons = evaluate_safety(
        False, (0.0, 0.0), True, 24.0, (1.2, 0.1, 0.1), limits
    )
    assert stop and "stale_imu" in reasons


def test_estop_attitude_joint_and_battery_fail_closed():
    limits = SafetyLimits(minimum_battery_voltage=18.0)
    stop, reasons = evaluate_safety(
        True,
        (0.8, 0.0),
        False,
        17.0,
        (0.0, 0.0, 0.0),
        limits,
    )
    assert stop
    assert {"emergency_stop", "attitude_limit", "invalid_joint_state", "low_battery"} <= set(reasons)


def test_required_missing_sources_stop_before_hardware_arrives():
    limits = SafetyLimits(
        require_imu=True,
        require_joint_states=True,
        require_battery=True,
    )
    stop, reasons = evaluate_safety(
        False, None, None, None, (None, None, None), limits
    )
    assert stop
    assert {"missing_imu", "missing_joint_states", "missing_battery"} <= set(reasons)


def test_mock_backend_success_and_fault_modes():
    running = mock_traversal_state(0.5, 2.0)
    assert running[0] == CrossingStatus.RUNNING
    success = mock_traversal_state(2.0, 2.0)
    assert success == (
        CrossingStatus.SUCCEEDED,
        CrossingStatus.VERIFYING_CONTACT,
        1.0,
        True,
    )
    assert mock_traversal_state(0.5, 2.0, "silence") is None
    assert mock_traversal_state(1.0, 2.0, "fail")[0] == CrossingStatus.FAILED
    assert mock_traversal_state(0.2, 2.0, "invalid_progress")[2] > 1.0


def test_safety_service_can_transition_between_ok_and_stop():
    """Service-level test covers publishers, diagnostics and logger transitions."""
    rclpy.init()
    node = SystemSafetySupervisor()
    try:
        response = node.estop_service(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert response.success
        assert node.last_state == "STOP:emergency_stop"
        node.estop_service(SetBool.Request(data=False), SetBool.Response())
        assert node.last_state == "OK"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_joint_state_accepts_unavailable_effort_but_rejects_partial_nan():
    names = ["a", "b"]
    assert joint_state_is_valid(names, [0.0, 0.1], [0.0, 0.0], [math.nan, math.nan])
    assert not joint_state_is_valid(names, [0.0, 0.1], [0.0, 0.0], [0.0, math.nan])
    assert joint_positions_within_limit([0.0, -1.0, 1.0], 1.1)
    assert not joint_positions_within_limit([0.0, 1.2], 1.1)

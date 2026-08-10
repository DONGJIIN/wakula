"""Regression tests for hardware-neutral kinematics and gait contracts."""

import math

import pytest

from quadruped_control.basic_motion_controller import (
    crossing_profile,
    quaternion_roll_pitch,
)
from quadruped_control.gait import JOINT_NAMES, GaitParameters, joint_targets
from quadruped_control.kinematics import (
    UnreachableFootTarget,
    forward_leg,
    inverse_leg,
    nominal_stance,
)
from quadruped_interfaces.msg import CrossingCommand


@pytest.mark.parametrize("side", (-1, 1))
def test_inverse_forward_round_trip_at_stance(side):
    target = nominal_stance(side)
    angles = inverse_leg(*target, side)
    actual = forward_leg(*angles, side)
    assert actual == pytest.approx(target, abs=1e-8)


def test_ik_rejects_singular_non_finite_and_overextended_targets():
    with pytest.raises(UnreachableFootTarget):
        inverse_leg(0.0, 0.0, 0.0, 1)
    with pytest.raises(UnreachableFootTarget):
        inverse_leg(float("nan"), 0.065, -0.39, 1)
    with pytest.raises(UnreachableFootTarget):
        inverse_leg(0.0, 0.065, -0.46, 1)


def test_gait_produces_all_finite_bounded_joint_targets():
    for phase in (0.0, 0.2, 0.5, 0.8, 0.999):
        targets = joint_targets(
            phase,
            0.35,
            0.12,
            0.6,
            GaitParameters(),
            roll=0.1,
            pitch=-0.1,
            attitude_gain=0.8,
        )
        assert tuple(targets) == JOINT_NAMES
        assert all(math.isfinite(value) for value in targets.values())
        for name, value in targets.items():
            if "_hip_" in name:
                assert -0.75 <= value <= 0.75
            elif "_thigh_" in name:
                assert -1.57 <= value <= 1.20
            else:
                assert -2.70 <= value <= -0.15


def test_invalid_twist_falls_back_to_safe_finite_stance():
    targets = joint_targets(0.3, float("inf"), 0.0, 0.0)
    assert len(targets) == 12
    assert all(math.isfinite(value) for value in targets.values())


def test_crossing_profile_validates_and_limits_clearance():
    assert crossing_profile(CrossingCommand.STEP, 0.1, 0.5) == pytest.approx(
        (0.08, 0.14)
    )
    assert crossing_profile(CrossingCommand.CLIMB, 0.4, 1.0)[1] == 0.20
    assert crossing_profile(99, 0.1, 0.5) is None
    assert crossing_profile(CrossingCommand.STEP, float("nan"), 0.5) is None


def test_quaternion_helper_normalizes_and_rejects_invalid():
    assert quaternion_roll_pitch(0.0, 0.0, 0.0, 0.0) is None
    result = quaternion_roll_pitch(0.0, math.sin(0.1), 0.0, math.cos(0.1))
    assert result == pytest.approx((0.0, 0.2), abs=1e-8)

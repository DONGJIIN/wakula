"""Analytic 3-DOF leg kinematics matching ``quadruped.urdf.xacro``.

Coordinates follow REP-103: ``x`` forward, ``y`` left and ``z`` upward.  Each
target is expressed from that leg's hip-roll joint.  The solver deliberately
chooses the knee-bent branch used by the nominal standing pose; callers must
never silently send unreachable or non-finite targets to a real actuator.
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class LegGeometry:
    hip_offset: float = 0.065
    upper_length: float = 0.22
    lower_length: float = 0.23


@dataclass(frozen=True)
class JointLimits:
    hip: Tuple[float, float] = (-0.75, 0.75)
    thigh: Tuple[float, float] = (-1.57, 1.20)
    calf: Tuple[float, float] = (-2.70, -0.15)


class UnreachableFootTarget(ValueError):
    """Raised when a requested foot pose has no safe IK solution."""


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def forward_leg(
    hip: float,
    thigh: float,
    calf: float,
    side: int,
    geometry: LegGeometry = LegGeometry(),
) -> Tuple[float, float, float]:
    """Return foot ``(x, y, z)`` from three joint angles.

    ``side`` is ``+1`` for left legs and ``-1`` for right legs.  This function
    is kept ROS-free so CAD dimensions and recorded joint data can be checked
    in unit tests or offline calibration tools.
    """
    if side not in (-1, 1) or not _finite((hip, thigh, calf)):
        raise ValueError("side must be +/-1 and angles must be finite")
    plane_x = -geometry.upper_length * math.sin(thigh)
    plane_x -= geometry.lower_length * math.sin(thigh + calf)
    plane_z = -geometry.upper_length * math.cos(thigh)
    plane_z -= geometry.lower_length * math.cos(thigh + calf)
    lateral = side * geometry.hip_offset
    y = lateral * math.cos(hip) - plane_z * math.sin(hip)
    z = lateral * math.sin(hip) + plane_z * math.cos(hip)
    return plane_x, y, z


def inverse_leg(
    x: float,
    y: float,
    z: float,
    side: int,
    geometry: LegGeometry = LegGeometry(),
    limits: JointLimits = JointLimits(),
    workspace_margin: float = 1e-5,
) -> Tuple[float, float, float]:
    """Solve the knee-bent IK branch and enforce workspace and joint limits."""
    if side not in (-1, 1) or not _finite((x, y, z)):
        raise UnreachableFootTarget("target must be finite and side must be +/-1")
    lateral_sq = y * y + z * z - geometry.hip_offset**2
    if lateral_sq <= workspace_margin**2:
        raise UnreachableFootTarget("target crosses the hip-offset singularity")
    plane_z = -math.sqrt(lateral_sq)
    hip = math.atan2(z, y) - math.atan2(
        plane_z, side * geometry.hip_offset
    )
    hip = math.atan2(math.sin(hip), math.cos(hip))

    reach_sq = x * x + plane_z * plane_z
    minimum = abs(geometry.upper_length - geometry.lower_length)
    maximum = geometry.upper_length + geometry.lower_length
    reach = math.sqrt(reach_sq)
    if reach <= minimum + workspace_margin or reach >= maximum - workspace_margin:
        raise UnreachableFootTarget(
            f"planar reach {reach:.4f} m outside ({minimum:.4f}, {maximum:.4f})"
        )
    cosine = (
        reach_sq - geometry.upper_length**2 - geometry.lower_length**2
    ) / (2.0 * geometry.upper_length * geometry.lower_length)
    if cosine < -1.0 - 1e-7 or cosine > 1.0 + 1e-7:
        raise UnreachableFootTarget("target has no real knee solution")
    calf = -math.acos(max(-1.0, min(1.0, cosine)))
    down = -plane_z
    forward_opposite = -x
    thigh = math.atan2(forward_opposite, down) - math.atan2(
        geometry.lower_length * math.sin(calf),
        geometry.upper_length + geometry.lower_length * math.cos(calf),
    )
    angles = (hip, thigh, calf)
    ranges = (limits.hip, limits.thigh, limits.calf)
    if any(angle < low or angle > high for angle, (low, high) in zip(angles, ranges)):
        raise UnreachableFootTarget(
            "IK solution violates configured hip/thigh/calf joint limits"
        )
    return angles


def nominal_stance(
    side: int,
    height: float = 0.39,
    geometry: LegGeometry = LegGeometry(),
) -> Tuple[float, float, float]:
    """Return a symmetric, comfortably bent standing foot target."""
    if not math.isfinite(height) or not 0.20 <= height <= 0.43:
        raise ValueError("standing height must be finite and within 0.20..0.43 m")
    return 0.0, side * geometry.hip_offset, -height

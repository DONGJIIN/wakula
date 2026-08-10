"""Deterministic open-loop foothold trajectories for pre-hardware testing.

This is intentionally a small gait generator, not an MPC or whole-body
controller.  It converts a bounded body Twist into smooth body-frame foot
targets; analytic IK then maps those targets to the existing twelve joint
names.  Keeping this layer pure makes it reusable with Gazebo, rosbag tests and
a future SDK adapter while the real dynamics backend is replaced independently.
"""

import math
from dataclasses import dataclass
from typing import Dict, Tuple

from .kinematics import LegGeometry, UnreachableFootTarget, inverse_leg


LEG_ORDER = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for leg in LEG_ORDER
    for joint in ("hip", "thigh", "calf")
)
LEG_LAYOUT = {
    "front_left": (0.20, 0.12, 1, 0.00),
    "front_right": (0.20, -0.12, -1, 0.50),
    "rear_left": (-0.20, 0.12, 1, 0.50),
    "rear_right": (-0.20, -0.12, -1, 0.00),
}
CRAWL_OFFSETS = {
    "front_left": 0.00,
    "rear_right": 0.25,
    "front_right": 0.50,
    "rear_left": 0.75,
}


@dataclass(frozen=True)
class GaitParameters:
    standing_height: float = 0.39
    cadence: float = 1.5
    duty_factor: float = 0.62
    swing_height: float = 0.06
    maximum_stride: float = 0.16
    maximum_lateral_stride: float = 0.06


def clamp_twist(vx: float, vy: float, wz: float):
    """Apply conservative software limits and reject NaN/Inf as zero."""
    values = (vx, vy, wz)
    if not all(math.isfinite(float(value)) for value in values):
        return 0.0, 0.0, 0.0
    return (
        max(-0.45, min(0.45, float(vx))),
        max(-0.18, min(0.18, float(vy))),
        max(-0.90, min(0.90, float(wz))),
    )


def _foot_cycle(
    phase: float,
    stride_x: float,
    stride_y: float,
    swing_height: float,
    duty_factor: float,
) -> Tuple[float, float, float]:
    """Return smooth ``x/y/lift`` offsets for one normalized gait cycle."""
    phase %= 1.0
    if phase < duty_factor:
        progress = phase / duty_factor
        blend = 0.5 * (1.0 - math.cos(math.pi * progress))
        return (
            0.5 * stride_x * (1.0 - 2.0 * blend),
            0.5 * stride_y * (1.0 - 2.0 * blend),
            0.0,
        )
    progress = (phase - duty_factor) / (1.0 - duty_factor)
    blend = 0.5 * (1.0 - math.cos(math.pi * progress))
    return (
        0.5 * stride_x * (-1.0 + 2.0 * blend),
        0.5 * stride_y * (-1.0 + 2.0 * blend),
        swing_height * math.sin(math.pi * progress),
    )


def joint_targets(
    phase: float,
    vx: float,
    vy: float,
    wz: float,
    parameters: GaitParameters = GaitParameters(),
    swing_height_override: float = 0.0,
    crawl: bool = False,
    roll: float = 0.0,
    pitch: float = 0.0,
    attitude_gain: float = 0.0,
) -> Dict[str, float]:
    """Generate all twelve joint targets for a trot or four-beat crawl.

    Small roll/pitch feedback changes virtual leg length only; it is clamped to
    25 mm and is therefore a posture aid, not a substitute for force control.
    """
    vx, vy, wz = clamp_twist(vx, vy, wz)
    cadence = max(0.2, float(parameters.cadence))
    stance_time = max(0.05, parameters.duty_factor / cadence)
    output = {}
    for index, leg in enumerate(LEG_ORDER):
        hip_x, hip_y, side, trot_offset = LEG_LAYOUT[leg]
        offset = CRAWL_OFFSETS[leg] if crawl else trot_offset
        local_vx = vx - wz * hip_y
        local_vy = vy + wz * hip_x
        stride_x = max(
            -parameters.maximum_stride,
            min(parameters.maximum_stride, local_vx * stance_time),
        )
        stride_y = max(
            -parameters.maximum_lateral_stride,
            min(parameters.maximum_lateral_stride, local_vy * stance_time),
        )
        lift_height = max(parameters.swing_height, swing_height_override)
        dx, dy, lift = _foot_cycle(
            phase + offset,
            stride_x,
            stride_y,
            lift_height,
            parameters.duty_factor if not crawl else 0.76,
        )
        posture = max(
            -0.025,
            min(0.025, attitude_gain * (-roll * hip_y + pitch * hip_x)),
        )
        target = (
            dx,
            side * LegGeometry().hip_offset + dy,
            -parameters.standing_height + lift + posture,
        )
        try:
            angles = inverse_leg(*target, side)
        except UnreachableFootTarget:
            # A neutral stance is always safer than clipping an invalid IK
            # target independently into an unknown linkage configuration.
            angles = inverse_leg(
                0.0,
                side * LegGeometry().hip_offset,
                -parameters.standing_height,
                side,
            )
        for joint, value in zip(("hip", "thigh", "calf"), angles):
            output[f"{leg}_{joint}_joint"] = value
    return output

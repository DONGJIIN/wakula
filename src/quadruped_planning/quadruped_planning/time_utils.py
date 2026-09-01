"""Small, testable helpers for ROS-clock watchdogs.

Simulation reset and rosbag looping can move ``/clock`` backwards.  A negative age is
therefore never evidence that cached data is fresh: callers must clear state tied to
the old clock epoch and wait for a new message.
"""

from __future__ import annotations

from math import isfinite


def ros_age_seconds(now, stamp) -> float:
    """Return ``now - stamp`` in seconds, or infinity when no stamp exists."""
    if stamp is None:
        return float("inf")
    try:
        return float((now - stamp).nanoseconds) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return float("inf")


def ros_age_is_fresh(now, stamp, timeout: float) -> bool:
    """Require a finite age in ``[0, timeout]``.

    The lower bound is as important as the timeout: without it, a timestamp retained
    across a simulator/rosbag rewind can remain authorized for an arbitrarily long
    portion of the new clock epoch.
    """
    age = ros_age_seconds(now, stamp)
    return bool(
        isfinite(age)
        and isfinite(float(timeout))
        and float(timeout) > 0.0
        and 0.0 <= age <= float(timeout)
    )


def ros_clock_moved_backward(now, previous_now) -> bool:
    """Return true when two local ROS-clock samples belong to different epochs."""
    if previous_now is None:
        return False
    try:
        return int(now.nanoseconds) < int(previous_now.nanoseconds)
    except (AttributeError, TypeError, ValueError):
        return True

"""Bounded runtime-mode probes shared by the public launch entries.

The ROS graph lists a topic when either a publisher *or* a subscriber exists.  A
plain ``ros2 topic list`` check therefore cannot prove that Gazebo or rosbag is
actually providing ``/clock``.  This module parses the publisher count reported
by ``ros2 topic info`` and deliberately uses a daemon-free, time-bounded query so
an old daemon cache cannot switch hardware nodes to simulation time.

The probe does not require the clock to advance: a deliberately paused simulator
still owns simulation time.  Playback workflows that start paused should continue
to pass ``use_sim_time:=true`` explicitly, as documented by the replay script.
"""

from __future__ import annotations

import re
import subprocess
import time
from typing import Optional


_PUBLISHER_COUNT = re.compile(r"^Publisher count:\s*(\d+)\s*$", re.MULTILINE)


def topic_publisher_count(output: str) -> Optional[int]:
    """Return the CLI publisher count, or ``None`` for unknown/malformed output."""
    match = _PUBLISHER_COUNT.search(output or "")
    return int(match.group(1)) if match else None


def clock_publisher_is_available() -> bool:
    """Confirm a ``/clock`` publisher with two short, daemon-free graph probes.

    Each subprocess has a hard wall-time bound.  DDS discovery can miss a bridge
    during its first few hundred milliseconds, so a single bounded retry is used.
    Failures and malformed CLI output conservatively select hardware time; callers
    can always override auto detection with an explicit launch argument.
    """
    attempts = 2
    for attempt in range(attempts):
        try:
            result = subprocess.run(
                [
                    "ros2",
                    "topic",
                    "info",
                    "/clock",
                    "--no-daemon",
                    "--spin-time",
                    "0.50",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.2,
            )
        except (OSError, subprocess.TimeoutExpired):
            result = None
        if result is not None and getattr(result, "returncode", 0) == 0:
            publisher_count = topic_publisher_count(result.stdout)
            if publisher_count is not None and publisher_count > 0:
                return True
        if attempt + 1 < attempts:
            time.sleep(0.10)
    return False

"""Tests for the reusable Gazebo/SLAM/Nav2 regression reporter."""

import math
from pathlib import Path
import time

from quadruped_tools.stack_regression import ResourceStats, StreamStats, _angle_error, parse_args


def test_angle_error_wraps_at_pi_boundary():
    """回环偏航误差不能把 +179°/-179° 错报为 358°。"""
    assert _angle_error(math.radians(179), math.radians(-179)) < math.radians(3)


def test_stream_stats_tracks_largest_gap_without_exposing_internal_clock():
    stats = StreamStats()
    stats.update()
    time.sleep(0.002)
    stats.update()
    public = stats.public()
    assert public["count"] == 2
    assert public["max_gap_seconds"] > 0.0
    assert "_last_time" not in public


def test_regression_requires_explicit_motion_permission_and_safe_defaults():
    options = parse_args([])
    assert not options.allow_motion
    assert options.command_topic == "/cmd_vel_teleop"
    assert options.cycles >= 3
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "stack_regression = quadruped_tools.stack_regression:main" in setup


def test_resource_sampler_has_no_optional_python_dependency():
    stats = ResourceStats()
    stats.sample()
    report = stats.public()
    assert report["samples"] == 1
    assert report["peak_rss_mib"] >= 0.0


def test_nav2_regression_keeps_narrow_passage_and_cancel_handshake():
    """防止后续精简工具时误删 1 m 绕杆通道或 Action 取消确认。"""
    source = (
        Path(__file__).parents[1] / "quadruped_tools" / "stack_regression.py"
    ).read_text(encoding="utf-8")
    assert '"narrow_pole_passage"' in source
    assert "cancel_goal_async" in source
    assert "cancel_acknowledged" in source

"""Tests for the reusable Gazebo/SLAM/Nav2 regression reporter."""

import math
from pathlib import Path
import time

from quadruped_interfaces.msg import TraversalGuidance
from quadruped_tools.stack_regression import (
    ResourceStats,
    StackRegression,
    StreamStats,
    _angle_error,
    parse_args,
)


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


def test_guidance_monitor_detects_only_inconsistent_ready_contract():
    """回归工具必须捕获“无有效感知却请求控制交接”的危险消息。"""
    node = object.__new__(StackRegression)
    node.streams = {"traversal_guidance": StreamStats()}
    node.guidance_phase_samples = {}
    node.guidance_phase_transitions = 0
    node.guidance_ready_boundary_transitions = 0
    node.guidance_rapid_ready_boundary_transitions = 0
    node.guidance_contract_violations = 0
    node._last_guidance_phase = None
    node._last_guidance_transition_time = None

    valid = TraversalGuidance()
    valid.phase = TraversalGuidance.PHASE_READY
    valid.perception_valid = True
    valid.traversal_required = True
    valid.ready_for_handoff = True
    node._guidance_callback(valid)
    assert node.guidance_contract_violations == 0

    invalid = TraversalGuidance()
    invalid.phase = TraversalGuidance.PHASE_READY
    invalid.ready_for_handoff = True
    node._guidance_callback(invalid)
    assert node.guidance_contract_violations == 1
    assert node.streams["traversal_guidance"].count == 2

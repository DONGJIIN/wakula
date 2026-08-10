"""Unit tests for terrain and OpenCV evidence fusion."""

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from quadruped_planning.cmd_vel_gate import gated_twist
from quadruped_planning.competition_obstacle_manager import CompetitionObstacleManager
from quadruped_planning.crossing_action_server import (
    controller_success_is_valid,
    validate_controller_status,
    validate_goal_values,
)
from quadruped_planning.crossing_action_coordinator import (
    CrossingTriggerLatch,
    select_goal_mode,
    valid_terrain_observation,
)
from quadruped_planning.obstacle_crossing_manager import (
    ConservativeDecisionFilter,
    apply_geometry_classification,
    apply_visual_assist,
    select_terrain_decision,
    validate_height_thresholds,
    visual_evidence_in_path,
)


def decide(height=0.0, points=100.0, slope=0.0, roughness=0.0):
    return select_terrain_decision(
        height,
        points,
        slope,
        roughness,
        30,
        0.08,
        0.18,
        0.32,
        0.45,
        0.06,
    )


def test_invalid_terrain_stops():
    assert decide(points=10)[0] == "STOP"
    assert decide(height=float("nan"))[0] == "STOP"


def test_invalid_decision_limits_fail_closed():
    """Corrupt runtime limits must never make unknown terrain look passable."""
    decision = select_terrain_decision(
        0.0,
        100.0,
        0.0,
        0.0,
        30,
        0.08,
        0.18,
        0.32,
        float("nan"),
        0.06,
    )
    assert decision == ("STOP", "WAIT_FOR_TERRAIN", 0.0)


def test_geometry_owns_crossing_mode():
    assert decide(height=0.04)[0] == "WALK"
    assert decide(height=0.10)[0] == "STEP"
    assert decide(height=0.20)[0] == "CLIMB"
    assert decide(height=0.35)[0] == "STOP"
    assert decide(slope=-0.50)[0] == "CLIMB"
    assert apply_geometry_classification(decide(), 3, 0.12)[0] == "STOP"
    assert apply_geometry_classification(decide(), 5, 0.0)[1] == "CROSS_LOW_PROFILE"
    assert select_goal_mode("CLIMB", 5, "CROSS_LOW_PROFILE") == 3


def test_visual_target_requires_confidence_and_center():
    centered_poles = [1.0, 0.75, 0.50, 0.50, 0.20, 0.60]
    edge_poles = [1.0, 0.75, 0.05, 0.50, 0.20, 0.60]
    uncertain_bar = [2.0, 0.30, 0.50, 0.50, 0.70, 0.10]

    assert visual_evidence_in_path(centered_poles, 0.55, 0.20)
    assert not visual_evidence_in_path(edge_poles, 0.55, 0.20)
    assert not visual_evidence_in_path(uncertain_bar, 0.55, 0.20)
    assert not visual_evidence_in_path(
        [1.0, 1.2, 0.5, 0.5, 0.2, 0.6], 0.55, 0.20
    )
    assert not visual_evidence_in_path(
        [1.0, 0.8, 0.5, 1.2, 0.2, 0.6], 0.55, 0.20
    )


def test_invalid_visual_data_is_ignored():
    assert not visual_evidence_in_path([], 0.55, 0.20)
    assert not visual_evidence_in_path([float("nan")] * 6, 0.55, 0.20)
    assert not visual_evidence_in_path([9.0, 0.9, 0.5, 0.5, 0.2, 0.2], 0.55, 0.20)


def test_invalid_height_thresholds_use_safe_defaults():
    assert validate_height_thresholds(0.08, 0.18, 0.32) == (0.08, 0.18, 0.32)
    assert validate_height_thresholds(0.30, 0.10, 0.20) == (0.08, 0.18, 0.32)


def test_visual_assist_only_slows_clear_terrain():
    walk = ("WALK", "NAVIGATE", 1.0)
    step = ("STEP", "CROSS_STEP", 0.45)

    assert apply_visual_assist(walk, True, 0.35) == (
        "WALK",
        "VERIFY_VISUAL_OBSTACLE_WITH_DEPTH",
        0.35,
    )
    assert apply_visual_assist(walk, False, 0.35) == walk
    assert apply_visual_assist(step, True, 0.35) == step


def test_velocity_gate_requires_both_fresh_inputs():
    """A stale planner or decision heartbeat always produces a zero command."""
    command = Twist()
    command.linear.x = 1.0
    command.angular.z = 0.5
    output = gated_twist(command, 0.4, True, True)
    assert abs(output.linear.x - 0.4) < 1e-6
    assert abs(output.angular.z - 0.2) < 1e-6
    assert gated_twist(command, 1.0, False, True).linear.x == 0.0
    assert gated_twist(command, 1.0, True, False).linear.x == 0.0
    assert gated_twist(command, 0.0, True, True).linear.x == 0.0
    assert gated_twist(command, 1.0, True, True, True, False).linear.x == 0.0
    assert gated_twist(command, 1.0, True, True, False, True).linear.x == 0.0
    assert gated_twist(
        command, 1.0, True, True, False, False, False
    ).linear.x == 0.0


def test_crossing_trigger_latches_until_clear_and_limits_retries():
    """One obstacle cannot create an unbounded stream of Action goals."""
    latch = CrossingTriggerLatch(clear_frames=2, retry_limit=1)
    assert latch.observe("STEP", True, False)
    assert not latch.observe("STEP", True, False)
    latch.finish(False)
    assert latch.observe("STEP", True, False)
    latch.finish(False)
    assert not latch.observe("STEP", True, False)
    assert not latch.observe("WALK", False, False)
    assert not latch.observe("WALK", False, False)
    assert latch.observe("CLIMB", True, False)


def test_crossing_trigger_rejects_invalid_or_distant_terrain():
    data = [0.0] * 9
    data[6], data[7] = 0.12, 0.60
    assert valid_terrain_observation(data, 0.80) == (0.12, 0.60)
    data[7] = 1.20
    assert valid_terrain_observation(data, 0.80) is None
    data[7] = float("nan")
    assert valid_terrain_observation(data, 0.80) is None


def test_crossing_action_goal_validation():
    """The Action boundary rejects unsafe modes, values and timeouts."""
    assert validate_goal_values(1, 0.12, 0.50, 0.4, 10.0, 60.0)[0]
    assert not validate_goal_values(99, 0.12, 0.50, 0.4, 10.0, 60.0)[0]
    assert not validate_goal_values(1, -0.1, 0.50, 0.4, 10.0, 60.0)[0]
    assert not validate_goal_values(1, 0.1, 0.50, 0.0, 10.0, 60.0)[0]
    assert not validate_goal_values(1, 0.1, 0.50, 0.4, 90.0, 60.0)[0]
    assert not validate_goal_values(
        1, float("nan"), 0.50, 0.4, 10.0, 60.0
    )[0]


def test_controller_status_requires_monotonic_valid_progress():
    """Malformed, out-of-range and regressing feedback cannot refresh a goal."""
    assert validate_controller_status(0, 2, 0.5, 0.4)[0]
    assert not validate_controller_status(9, 2, 0.5, 0.4)[0]
    assert not validate_controller_status(0, 9, 0.5, 0.4)[0]
    assert not validate_controller_status(0, 2, 1.2, 0.4)[0]
    assert not validate_controller_status(0, 2, 0.3, 0.4)[0]
    # Failure/cancel must still be accepted even if the backend resets progress.
    assert validate_controller_status(2, 2, 0.0, 0.8)[0]


def test_controller_success_requires_progress_and_contact_proof():
    assert controller_success_is_valid(0.98, True, 0.95, True)
    assert not controller_success_is_valid(0.90, True, 0.95, True)
    assert not controller_success_is_valid(1.0, False, 0.95, True)
    assert controller_success_is_valid(1.0, False, 0.95, False)


def test_decision_filter_escalates_now_and_confirms_clearance():
    """STOP is immediate; motion actions and clearance need repeated evidence."""
    filter_ = ConservativeDecisionFilter(
        3, ("WALK", "NAVIGATE", 1.0), hazard_frames=2
    )
    assert filter_.update(("CLIMB", "CROSS_CLIMB", 0.2))[0] == "WALK"
    assert filter_.update(("CLIMB", "CROSS_CLIMB", 0.2))[0] == "CLIMB"
    assert filter_.update(("WALK", "NAVIGATE", 1.0))[0] == "CLIMB"
    assert filter_.update(("WALK", "NAVIGATE", 1.0))[0] == "CLIMB"
    assert filter_.update(("WALK", "NAVIGATE", 1.0))[0] == "WALK"
    assert filter_.update(("STOP", "REPLAN", 0.0))[0] == "STOP"


def test_competition_tracks_out_of_order_active_obstacle():
    """Completing a hinted obstacle must not score the first configured one."""
    rclpy.init()
    node = CompetitionObstacleManager()
    try:
        node.hint_callback(String(data="bridge_b"))
        assert node.current_obstacle == "bridge_b"
        node.complete_callback(Bool(data=True))
        assert "bridge_b" in node.completed
        assert "straight_poles" not in node.completed
        assert node.current_obstacle == "straight_poles"
    finally:
        node.destroy_node()
        rclpy.shutdown()

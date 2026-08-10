"""地形、OpenCV 辅助证据和速度门的纯逻辑测试。"""

from geometry_msgs.msg import Twist
from quadruped_interfaces.msg import FusedObstacle

from quadruped_planning.cmd_vel_gate import gated_twist
from quadruped_planning.obstacle_crossing_manager import (
    ConservativeDecisionFilter,
    apply_geometry_classification,
    apply_visual_assist,
    select_terrain_decision,
    select_fused_decision,
    validate_height_thresholds,
    visual_evidence_in_path,
)


def decide(height=0.0, points=100.0, slope=0.0, roughness=0.0):
    return select_terrain_decision(
        height, points, slope, roughness, 30, 0.08, 0.18, 0.32, 0.45, 0.06
    )


def test_invalid_terrain_stops():
    assert decide(points=10) == ("STOP", "WAIT_FOR_TERRAIN", 0.0)
    assert decide(height=float("nan"))[0] == "STOP"


def test_invalid_limits_fail_closed():
    result = select_terrain_decision(
        0.0, 100.0, 0.0, 0.0, 30, 0.08, 0.18, 0.32, float("nan"), 0.06
    )
    assert result == ("STOP", "WAIT_FOR_TERRAIN", 0.0)


def test_geometry_classifies_but_never_executes_crossing():
    assert decide(height=0.04) == ("WALK", "NAVIGATE", 1.0)
    assert decide(height=0.10) == ("STEP", "STOP_FOR_STEP", 0.0)
    assert decide(height=0.20) == ("CLIMB", "STOP_FOR_CLIMB_OR_REPLAN", 0.0)
    assert decide(height=0.35)[0] == "STOP"
    assert decide(slope=-0.50)[0] == "CLIMB"
    assert apply_geometry_classification(decide(), 3, 0.12)[0] == "STOP"
    assert apply_geometry_classification(decide(), 5, 0.0) == (
        "STOP", "STOP_FOR_LOW_BAR", 0.0
    )
    assert apply_geometry_classification(decide(height=0.10), 4, 0.0) == (
        "STOP", "REPLAN_AROUND_WALL", 0.0
    )


def test_visual_target_requires_valid_centered_evidence():
    assert visual_evidence_in_path([1.0, 0.75, 0.50, 0.50, 0.20, 0.60], 0.55, 0.20)
    assert not visual_evidence_in_path([1.0, 0.75, 0.05, 0.50, 0.20, 0.60], 0.55, 0.20)
    assert not visual_evidence_in_path([2.0, 0.30, 0.50, 0.50, 0.70, 0.10], 0.55, 0.20)
    assert not visual_evidence_in_path([], 0.55, 0.20)
    assert not visual_evidence_in_path([float("nan")] * 6, 0.55, 0.20)


def test_invalid_height_thresholds_use_safe_defaults():
    assert validate_height_thresholds(0.08, 0.18, 0.32) == (0.08, 0.18, 0.32)
    assert validate_height_thresholds(0.30, 0.10, 0.20) == (0.08, 0.18, 0.32)


def test_visual_assist_only_slows_clear_terrain():
    walk = ("WALK", "NAVIGATE", 1.0)
    stopped_step = ("STEP", "STOP_FOR_STEP", 0.0)
    assert apply_visual_assist(walk, True, 0.35) == (
        "WALK", "VERIFY_VISUAL_OBSTACLE_WITH_DEPTH", 0.35
    )
    assert apply_visual_assist(walk, False, 0.35) == walk
    assert apply_visual_assist(stopped_step, True, 0.35) == stopped_step


def test_fused_observation_is_atomic_and_fail_closed():
    msg = FusedObstacle()
    msg.geometry_confirmed = True
    msg.confidence = 0.8
    msg.obstacle_type = FusedObstacle.STEP
    msg.obstacle_height = 0.12
    msg.valid_points = 100
    result = select_fused_decision(
        msg, 0.25, 30, 0.08, 0.18, 0.32, 0.45, 0.06, 0.35
    )
    assert result == ("STEP", "STOP_FOR_STEP", 0.0)

    msg.geometry_confirmed = False
    assert select_fused_decision(
        msg, 0.25, 30, 0.08, 0.18, 0.32, 0.45, 0.06, 0.35
    ) == ("STOP", "WAIT_FOR_SYNCHRONIZED_PERCEPTION", 0.0)


def test_fused_visual_confirmation_only_slows_clear_geometry():
    msg = FusedObstacle()
    msg.geometry_confirmed = True
    msg.vision_confirmed = True
    msg.confidence = 0.8
    msg.obstacle_type = FusedObstacle.CLEAR
    msg.valid_points = 100
    assert select_fused_decision(
        msg, 0.25, 30, 0.08, 0.18, 0.32, 0.45, 0.06, 0.35
    ) == ("WALK", "VERIFY_VISUAL_OBSTACLE_WITH_DEPTH", 0.35)


def test_velocity_gate_requires_fresh_command_and_decision():
    command = Twist()
    command.linear.x = 1.0
    command.angular.z = 0.5
    output = gated_twist(command, 0.4, True, True)
    assert abs(output.linear.x - 0.4) < 1e-6
    assert abs(output.angular.z - 0.2) < 1e-6
    assert gated_twist(command, 1.0, False, True).linear.x == 0.0
    assert gated_twist(command, 1.0, True, False).linear.x == 0.0
    assert gated_twist(command, 0.0, True, True).linear.x == 0.0


def test_filter_confirms_hazard_and_clearance_but_stops_immediately():
    filter_ = ConservativeDecisionFilter(3, ("WALK", "NAVIGATE", 1.0), hazard_frames=2)
    climb = ("CLIMB", "STOP_FOR_CLIMB_OR_REPLAN", 0.0)
    assert filter_.update(climb)[0] == "WALK"
    assert filter_.update(climb)[0] == "CLIMB"
    assert filter_.update(("WALK", "NAVIGATE", 1.0))[0] == "CLIMB"
    assert filter_.update(("WALK", "NAVIGATE", 1.0))[0] == "CLIMB"
    assert filter_.update(("WALK", "NAVIGATE", 1.0))[0] == "WALK"
    assert filter_.update(("STOP", "REPLAN", 0.0))[0] == "STOP"

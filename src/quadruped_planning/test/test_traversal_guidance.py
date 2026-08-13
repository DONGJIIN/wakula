"""越障入口引导的纯逻辑回归测试。"""

from quadruped_interfaces.msg import NavigationSafety, TraversalGuidance
from quadruped_planning.traversal_guidance import compute_guidance


PARAMETERS = {
    "approach_start_distance": 1.5,
    "handoff_distance": 0.75,
    "alignment_tolerance": 0.10,
    "max_lateral_target": 0.45,
    "approach_speed_limit": 0.25,
    "alignment_speed_limit": 0.12,
    "minimum_slope_for_handoff": 0.12,
}


def safety(obstacle_type, distance=1.0, lateral=0.0):
    """构造一条最小有效的原子安全观测。"""
    msg = NavigationSafety()
    msg.perception_valid = True
    msg.obstacle_type = obstacle_type
    msg.confidence = 0.8
    msg.distance = distance
    msg.lateral_offset = lateral
    msg.speed_limit = 0.5
    return msg


def test_invalid_observation_never_creates_an_approach_target():
    msg = safety(NavigationSafety.OBSTACLE_STEP)
    msg.perception_valid = False
    decision = compute_guidance(msg, **PARAMETERS)
    assert decision.phase == TraversalGuidance.PHASE_INVALID
    assert not decision.traversal_required
    assert decision.speed_limit == 0.0


def test_step_transitions_from_approach_to_align_and_ready():
    far = compute_guidance(
        safety(NavigationSafety.OBSTACLE_STEP, 2.0, 0.20), **PARAMETERS
    )
    assert far.phase == TraversalGuidance.PHASE_APPROACH
    assert abs(far.approach_x - 1.25) < 1e-6
    assert far.speed_limit == 0.25

    middle = compute_guidance(
        safety(NavigationSafety.OBSTACLE_STEP, 1.0, 0.10), **PARAMETERS
    )
    assert middle.phase == TraversalGuidance.PHASE_ALIGN
    assert middle.approach_y == 0.10

    close = compute_guidance(
        safety(NavigationSafety.OBSTACLE_STEP, 0.70, 0.01), **PARAMETERS
    )
    assert close.phase == TraversalGuidance.PHASE_READY
    assert close.ready_for_handoff
    assert close.speed_limit == 0.0


def test_close_but_misaligned_target_is_not_ready():
    decision = compute_guidance(
        safety(NavigationSafety.OBSTACLE_BAR, 0.70, 0.20), **PARAMETERS
    )
    assert decision.phase == TraversalGuidance.PHASE_ALIGN
    assert not decision.ready_for_handoff


def test_pole_remains_nav2_navigation_object_not_motion_handoff():
    decision = compute_guidance(
        safety(NavigationSafety.OBSTACLE_POLE, 0.60), **PARAMETERS
    )
    assert decision.phase == TraversalGuidance.PHASE_CLEAR
    assert not decision.traversal_required


def test_confirmed_slope_can_request_handoff_without_fake_obstacle_type():
    msg = safety(NavigationSafety.OBSTACLE_CLEAR, 0.60)
    msg.slope_pitch = 0.18
    decision = compute_guidance(msg, **PARAMETERS)
    assert decision.phase == TraversalGuidance.PHASE_READY
    assert decision.obstacle_type == NavigationSafety.OBSTACLE_CLEAR
    assert decision.traversal_required

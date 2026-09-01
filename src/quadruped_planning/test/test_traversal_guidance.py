"""越障入口引导的纯逻辑回归测试。"""

import math

from quadruped_interfaces.msg import NavigationSafety, TraversalGuidance
from quadruped_planning.traversal_guidance import (
    GuidanceStabilizer,
    compute_guidance,
    surface_axis_heading,
)


PARAMETERS = {
    "approach_start_distance": 1.5,
    "handoff_distance": 0.75,
    "alignment_tolerance": 0.10,
    "max_lateral_target": 0.45,
    "approach_speed_limit": 0.25,
    "alignment_speed_limit": 0.12,
    "minimum_slope_for_handoff": 0.12,
}


def safety(obstacle_type, distance=1.0, lateral=0.0, semantic_id=""):
    """构造一条最小有效的原子安全观测。"""
    msg = NavigationSafety()
    msg.perception_valid = True
    msg.obstacle_type = obstacle_type
    msg.semantic_id = semantic_id
    msg.confidence = 0.8
    msg.distance = distance
    msg.lateral_offset = lateral
    msg.speed_limit = 0.5
    return msg


def stabilizer():
    """使用生产默认值构造不依赖 ROS 节点的时序稳定器。"""
    return GuidanceStabilizer(
        handoff_distance=PARAMETERS["handoff_distance"],
        alignment_tolerance=PARAMETERS["alignment_tolerance"],
        approach_start_distance=PARAMETERS["approach_start_distance"],
        target_smoothing_alpha=0.35,
        distance_hysteresis=0.05,
        angle_hysteresis=0.035,
        ready_confirmation_frames=3,
        type_confirmation_frames=3,
        approach_speed_limit=PARAMETERS["approach_speed_limit"],
        alignment_speed_limit=PARAMETERS["alignment_speed_limit"],
    )


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


def test_only_rule_height_pole_enters_competition_course_handoff():
    low_support = safety(NavigationSafety.OBSTACLE_POLE, 0.60)
    low_support.obstacle_height = 0.32
    decision = compute_guidance(low_support, **PARAMETERS)
    assert decision.phase == TraversalGuidance.PHASE_CLEAR
    assert not decision.traversal_required

    rule_pole = safety(NavigationSafety.OBSTACLE_POLE, 0.60)
    rule_pole.obstacle_height = 0.55
    decision = compute_guidance(rule_pole, **PARAMETERS)
    assert decision.phase == TraversalGuidance.PHASE_READY
    assert decision.traversal_required


def test_confirmed_slope_can_request_handoff_without_fake_obstacle_type():
    msg = safety(NavigationSafety.OBSTACLE_CLEAR, 0.60)
    msg.slope_pitch = 0.18
    decision = compute_guidance(msg, **PARAMETERS)
    assert decision.phase == TraversalGuidance.PHASE_READY
    assert decision.obstacle_type == NavigationSafety.OBSTACLE_CLEAR
    assert decision.traversal_required


def test_ready_requires_three_consistent_frames_and_holds_small_noise():
    """单帧低估距离不能触发交接，READY 后厘米级抖动也不应闪回 ALIGN。"""
    filter_ = stabilizer()
    close = compute_guidance(
        safety(NavigationSafety.OBSTACLE_STEP, 0.70, 0.01), **PARAMETERS
    )
    first = filter_.update(close)
    second = filter_.update(close)
    third = filter_.update(close)
    assert first.phase == TraversalGuidance.PHASE_ALIGN
    assert second.phase == TraversalGuidance.PHASE_ALIGN
    assert third.phase == TraversalGuidance.PHASE_READY
    assert third.ready_for_handoff

    # 0.77 m 超过进入阈值 0.75 m，但仍位于 READY 的 0.80 m 退出边界内。
    noisy = compute_guidance(
        safety(NavigationSafety.OBSTACLE_STEP, 0.77, 0.01), **PARAMETERS
    )
    held = filter_.update(noisy)
    assert held.phase == TraversalGuidance.PHASE_READY


def test_invalid_input_immediately_revokes_ready_and_clears_history():
    """感知断流优先于平滑，不能把历史 READY 粘到恢复后的新数据上。"""
    filter_ = stabilizer()
    close = compute_guidance(
        safety(NavigationSafety.OBSTACLE_BAR, 0.65), **PARAMETERS
    )
    for _ in range(3):
        output = filter_.update(close)
    assert output.ready_for_handoff

    invalid = filter_.update(compute_guidance(NavigationSafety(), **PARAMETERS))
    assert invalid.phase == TraversalGuidance.PHASE_INVALID
    assert not invalid.ready_for_handoff

    recovered = filter_.update(close)
    assert recovered.phase == TraversalGuidance.PHASE_ALIGN
    assert not recovered.ready_for_handoff


def test_target_low_pass_rejects_single_frame_lateral_jump():
    """障碍连通域边缘跳动时，建议入口不应跟随单帧横移。"""
    filter_ = stabilizer()
    base = compute_guidance(
        safety(NavigationSafety.OBSTACLE_WALL, 1.0, 0.0), **PARAMETERS
    )
    filter_.update(base)
    jump = compute_guidance(
        safety(NavigationSafety.OBSTACLE_WALL, 1.0, 0.40), **PARAMETERS
    )
    output = filter_.update(jump)
    assert 0.0 < output.approach_y < 0.40


def test_geometry_type_flicker_does_not_reset_same_entry():
    """同一障碍边缘的单帧类别跳变不得撤销入口连续性。"""
    filter_ = stabilizer()
    step = compute_guidance(
        safety(NavigationSafety.OBSTACLE_STEP, 0.70, 0.01), **PARAMETERS
    )
    pit = compute_guidance(
        safety(NavigationSafety.OBSTACLE_PIT, 0.71, 0.02), **PARAMETERS
    )
    first = filter_.update(step)
    second = filter_.update(pit)
    third = filter_.update(step)
    assert first.obstacle_type == NavigationSafety.OBSTACLE_STEP
    assert second.obstacle_type == NavigationSafety.OBSTACLE_STEP
    assert third.phase == TraversalGuidance.PHASE_READY


def test_semantic_id_is_forwarded_and_interleaved_obstacles_never_share_ready():
    """同粗类型邻近障碍必须以强类型 ID 形成新的确认窗口。"""
    filter_ = stabilizer()
    bridge = compute_guidance(
        safety(
            NavigationSafety.OBSTACLE_STEP,
            0.70,
            0.01,
            "wooden_bridge_a",
        ),
        **PARAMETERS,
    )
    stair = compute_guidance(
        safety(
            NavigationSafety.OBSTACLE_STEP,
            0.71,
            0.02,
            "t_shaped_stairs",
        ),
        **PARAMETERS,
    )
    assert bridge.semantic_id == "wooden_bridge_a"
    filter_.update(bridge)
    filter_.update(bridge)
    switched = filter_.update(stair)
    assert switched.semantic_id == "t_shaped_stairs"
    assert switched.phase == TraversalGuidance.PHASE_ALIGN
    assert not switched.ready_for_handoff


def test_surface_axis_aligns_diagonal_ramp_before_handoff():
    """斜看 11.3° 主坡时应先转到坡轴，不能沿可见轮廓中心斜穿场地。"""
    heading = surface_axis_heading(math.radians(7.1), math.radians(7.1))
    assert heading is not None
    assert math.radians(40.0) <= heading <= math.radians(50.0)

    msg = safety(NavigationSafety.OBSTACLE_STEP, 0.70, 0.0)
    msg.slope_pitch = math.radians(7.1)
    msg.slope_roll = math.radians(7.1)
    decision = compute_guidance(msg, **PARAMETERS)
    assert decision.phase == TraversalGuidance.PHASE_ALIGN
    assert decision.heading_error > math.radians(40.0)


def test_surface_axis_ignores_flat_plane_noise():
    assert surface_axis_heading(math.radians(1.0), math.radians(1.0)) is None


def test_flat_bridge_uses_confident_front_edge_heading():
    """平桥无坡度时仍应按点云前缘法线对正，而非斜穿桥面。"""
    msg = safety(NavigationSafety.OBSTACLE_STEP, 0.70, 0.02)
    msg.structure_heading = math.radians(-28.0)
    msg.structure_heading_confidence = 0.82
    decision = compute_guidance(msg, **PARAMETERS)
    assert decision.phase == TraversalGuidance.PHASE_ALIGN
    assert math.radians(-31.0) < decision.heading_error < math.radians(-24.0)


def test_uncertain_front_edge_keeps_centre_fallback():
    """低可信方向不能覆盖稳定的连通域中心对正。"""
    msg = safety(NavigationSafety.OBSTACLE_STEP, 0.70, 0.20)
    msg.structure_heading = math.radians(70.0)
    msg.structure_heading_confidence = 0.20
    decision = compute_guidance(msg, **PARAMETERS)
    assert abs(decision.heading_error) < math.radians(20.0)

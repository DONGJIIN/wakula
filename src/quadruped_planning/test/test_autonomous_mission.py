"""前沿探索和自动任务安全边界的确定性测试。"""

from math import hypot

from nav_msgs.msg import OccupancyGrid

from quadruped_planning.autonomous_mission import (
    Frontier,
    FailedEntry,
    ObservedObstacle,
    action_type_for_semantic,
    action_obstacle_type,
    bounded_alignment_delta,
    canonical_obstacle_id,
    choose_frontier,
    choose_pending_obstacle,
    close_handoff_is_safe,
    distance_outside_grid,
    distance_inside_grid_edge,
    distance_to_segment,
    dominant_planar_vote,
    extract_coverage_goals,
    extract_frontiers,
    failed_entry_matches,
    frontier_goal_in_known_free_space,
    inventory_display,
    is_actionable_semantic_id,
    mission_score,
    mission_inventory,
    navigation_purpose_allows_yaw_only_recovery,
    matching_pending_semantic,
    matching_pending_semantic_from_viewpoint,
    nav_status_allows_guarded_handoff,
    normalized_angle,
    obstacle_revisit_delay,
    obstacle_geometry_fits_candidate,
    obstacle_was_completed,
    resolve_completed_semantics,
    inventory_message,
    semantic_id_for_action,
    semantic_vote_is_confirmed,
    replacement_semantic_vote,
    recovery_station_in_known_free_space,
    select_full_semantic_vote,
    select_semantic_vote,
    semantic_after_approach_stall,
    semantic_task_is_complete,
    target_is_in_heading_cone,
    timeout_reached,
    traversal_crossing_evidence,
    traversal_geometry_evidence,
    verification_station_matches,
    world_to_cell,
)
from action_msgs.msg import GoalStatus
from quadruped_interfaces.action import TraverseObstacle
from quadruped_interfaces.msg import TraversalGuidance


def map_with_unknown_border():
    grid = OccupancyGrid()
    grid.info.width = 12
    grid.info.height = 10
    grid.info.resolution = 0.5
    grid.info.origin.position.x = -3.0
    grid.info.origin.position.y = -2.5
    grid.info.origin.orientation.w = 1.0
    data = [-1] * (grid.info.width * grid.info.height)
    # 中央 6×4 是已探索自由区；其外圈自由格就是一条闭合前沿。
    for row in range(3, 7):
        for col in range(3, 9):
            data[row * grid.info.width + col] = 0
    grid.data = data
    return grid


def test_frontier_goal_is_on_known_free_space_and_scored():
    grid = map_with_unknown_border()
    result = extract_frontiers(grid, (0.0, 0.0), minimum_cells=4, minimum_distance=0.0)
    assert result
    # ``cells`` 是该周向扇区的局部信息增益，不再是整圈前沿的总格数。
    assert result[0].cells >= 2
    assert result[0].score > 0.0
    assert -3.0 <= result[0].x <= 3.0
    assert -2.5 <= result[0].y <= 2.5
    # 闭合前沿不能被平均成地图中心；每个输出必须来自一枚真实自由 frontier cell。
    assert len(result) >= 4
    col = int((result[0].x - grid.info.origin.position.x) / grid.info.resolution)
    row = int((result[0].y - grid.info.origin.position.y) / grid.info.resolution)
    assert grid.data[row * grid.info.width + col] == 0
    assert hypot(result[0].x, result[0].y) > 0.5


def test_invalid_or_tiny_maps_have_no_frontier():
    assert extract_frontiers(OccupancyGrid(), (0.0, 0.0)) == []
    grid = map_with_unknown_border()
    assert extract_frontiers(grid, (0.0, 0.0), minimum_cells=100) == []


def test_frontier_navigation_goal_retreats_into_known_free_space():
    """生产探索目标应与未知边界留有净空，防止 Nav2 在终点膨胀区停滞。"""
    grid = map_with_unknown_border()
    raw = (1.25, 0.25)  # 自由区最东侧、紧邻未知格。
    goal = frontier_goal_in_known_free_space(
        grid, raw, (0.0, 0.0), standoff=0.50, clearance=0.0
    )
    assert goal is not None
    assert hypot(goal[0], goal[1]) < hypot(raw[0], raw[1])
    cell = world_to_cell(grid, *goal)
    assert cell is not None
    assert grid.data[cell[1] * grid.info.width + cell[0]] == 0


def test_frontier_goal_rejects_unknown_clearance_disk():
    grid = map_with_unknown_border()
    # 2 m 净空在这个小已知区域内无法满足，必须跳过而不是发布危险目标。
    assert frontier_goal_in_known_free_space(
        grid, (1.25, 0.25), (0.0, 0.0), standoff=0.5, clearance=2.0
    ) is None


def test_recovery_station_avoids_occupied_forward_corridor():
    """Recovery must choose a free side instead of blindly re-entering a step."""
    grid = OccupancyGrid()
    grid.info.width = 60
    grid.info.height = 60
    grid.info.resolution = 0.10
    grid.info.origin.position.x = -3.0
    grid.info.origin.position.y = -3.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (grid.info.width * grid.info.height)
    # Block a wide east-facing strip directly in front of the robot.
    for row in range(27, 34):
        for col in range(33, 48):
            grid.data[row * grid.info.width + col] = 100
    goal = recovery_station_in_known_free_space(
        grid, (0.0, 0.0, 0.0), 0.80, clearance=0.20
    )
    assert goal is not None
    assert abs(goal[1]) > 0.20 or goal[0] < 0.0


def test_recovery_station_fails_closed_without_body_clearance():
    grid = map_with_unknown_border()
    assert recovery_station_in_known_free_space(
        grid, (0.0, 0.0, 0.0), 0.80, clearance=2.0
    ) is None


def test_coverage_goals_visit_known_free_space_after_frontiers_disappear():
    """地图全 known 时仍须巡检未走近区域，而不是原地旋转并提前结束。"""
    grid = OccupancyGrid()
    grid.info.width = 20
    grid.info.height = 12
    grid.info.resolution = 0.25
    grid.info.origin.position.x = -2.5
    grid.info.origin.position.y = -1.5
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (grid.info.width * grid.info.height)
    goals = extract_coverage_goals(
        grid,
        (0.0, 0.0),
        [(0.0, 0.0)],
        spacing=0.5,
        clearance=0.25,
        visit_radius=0.75,
        minimum_distance=0.5,
        maximum_distance=4.0,
    )
    assert goals
    assert goals[0].distance > 0.75
    assert world_to_cell(grid, goals[0].x, goals[0].y) is not None
    # 把首选目标登记为已访问后，它不能再次成为覆盖候选。
    revisited = extract_coverage_goals(
        grid,
        (0.0, 0.0),
        [(0.0, 0.0), (goals[0].x, goals[0].y)],
        spacing=0.5,
        clearance=0.25,
        visit_radius=0.75,
        minimum_distance=0.5,
        maximum_distance=4.0,
    )
    assert all(hypot(item.x - goals[0].x, item.y - goals[0].y) >= 0.75 for item in revisited)


def test_consistent_near_field_geometry_replaces_stale_far_field_vote():
    """两帧近场结构可纠正远场历史多数票，但单帧跳变不能改类。"""
    votes = ["t_shaped_stairs"] * 7 + ["high_wall"] * 2
    assert select_semantic_vote(votes) == "high_wall"
    assert select_semantic_vote(
        ["t_shaped_stairs"] * 7 + ["high_wall"]
    ) == "t_shaped_stairs"


def test_action_semantic_requires_repeated_recent_evidence():
    votes = ["t_shaped_stairs"] * 6 + ["high_wall"] * 3
    assert semantic_vote_is_confirmed(
        votes, "high_wall", minimum_votes=3, recent_window=5
    )


def test_sustained_new_semantic_can_replace_but_noise_cannot():
    """A lock is sticky for noise, not permanent against a better viewpoint."""
    assert replacement_semantic_vote(
        ["high_wall", "high_wall", "t_shaped_stairs", "high_wall"],
        "high_wall",
        minimum_votes=3,
        recent_window=6,
    ) == ""
    assert replacement_semantic_vote(
        [
            "high_wall",
            "high_wall",
            "t_shaped_stairs",
            "t_shaped_stairs",
            "t_shaped_stairs",
        ],
        "high_wall",
        minimum_votes=3,
        recent_window=6,
    ) == "t_shaped_stairs"
    assert replacement_semantic_vote(
        ["t_shaped_stairs"] * 6,
        "t_shaped_stairs",
        minimum_votes=3,
        recent_window=6,
    ) == ""


def test_close_handoff_rejects_unknown_or_misaligned_obstacle():
    limits = dict(
        maximum_distance=1.20,
        maximum_lateral=0.50,
        maximum_heading_error=0.40,
    )
    assert close_handoff_is_safe(
        "main_slope", 0.80, 0.10, 0.05, **limits
    )
    assert not close_handoff_is_safe(
        "", 0.80, 0.10, 0.05, **limits
    )
    assert not close_handoff_is_safe(
        "main_slope", 1.21, 0.10, 0.05, **limits
    )
    assert not close_handoff_is_safe(
        "main_slope", 0.80, 0.10, 0.60, **limits
    )
    assert not semantic_vote_is_confirmed(
        ["high_wall", "t_shaped_stairs", "high_wall"],
        "high_wall",
        minimum_votes=3,
        recent_window=5,
    )


def test_confirmed_obstacle_pre_alignment_is_bounded_and_noise_safe():
    """远处障碍应先有限原地对正，小误差和非法值不得生成旋转目标。"""
    assert bounded_alignment_delta(0.12, 0.18, 0.52) == 0.0
    assert abs(bounded_alignment_delta(0.30, 0.18, 0.52) - 0.30) < 1e-6
    assert abs(bounded_alignment_delta(1.20, 0.18, 0.52) - 0.52) < 1e-6
    assert abs(bounded_alignment_delta(-1.20, 0.18, 0.52) + 0.52) < 1e-6
    assert bounded_alignment_delta(float("nan"), 0.18, 0.52) == 0.0


def test_world_to_cell_rejects_robot_beyond_latest_map_boundary():
    grid = map_with_unknown_border()
    assert world_to_cell(grid, -2.75, -2.25) == (0, 0)
    assert world_to_cell(grid, 2.99, 2.49) == (11, 9)
    assert world_to_cell(grid, 3.01, 0.0) is None
    assert world_to_cell(grid, -3.01, 0.0) is None
    assert distance_outside_grid(grid, 0.0, 0.0) == 0.0
    assert abs(distance_outside_grid(grid, 3.08, 0.0) - 0.08) < 1e-6
    assert abs(distance_outside_grid(grid, -3.20, -2.70) - hypot(0.20, 0.20)) < 1e-6
    assert abs(distance_inside_grid_edge(grid, 0.0, 0.0) - 2.5) < 1e-6
    assert abs(distance_inside_grid_edge(grid, 2.90, 0.0) - 0.10) < 1e-6
    assert distance_inside_grid_edge(grid, 3.10, 0.0) == 0.0


def test_boundary_guard_allows_targets_that_turn_away():
    robot = (0.0, 0.0, 0.0)
    assert target_is_in_heading_cone(robot, (2.0, 0.2))
    assert not target_is_in_heading_cone(robot, (0.0, 2.0))
    assert not target_is_in_heading_cone(robot, (-2.0, 0.0))
    assert not target_is_in_heading_cone(robot, None)


def test_completed_long_obstacle_corridor_suppresses_its_far_edge():
    assert distance_to_segment((5.3, 0.1), (0.8, 0.0), (6.2, 0.2)) < 0.2
    assert distance_to_segment((3.0, 1.0), (0.8, 0.0), (6.2, 0.2)) > 0.8


def test_traversal_completion_requires_reaching_far_side_of_entry():
    """只前进到入口或横向绕到旁边，都不能被误记为成功越障。"""
    limits = dict(minimum_displacement=0.45, beyond_obstacle_margin=0.12)
    assert traversal_crossing_evidence(
        (0.0, 0.0), (1.0, 0.0), (1.25, 0.08), **limits
    )
    assert not traversal_crossing_evidence(
        (0.0, 0.0), (1.0, 0.0), (0.95, 0.0), **limits
    )
    assert not traversal_crossing_evidence(
        (0.0, 0.0), (1.0, 0.0), (0.0, 1.5), **limits
    )


def test_right_angle_pole_completion_allows_lateral_finish_but_requires_motion():
    """The pole course turns; its valid finish need not lie beyond one entry plane."""
    limits = dict(minimum_displacement=0.45, beyond_obstacle_margin=0.12)
    assert traversal_geometry_evidence(
        "right_angle_poles", (0.0, 0.0), (1.0, 0.0), (0.2, 1.5), **limits
    )
    assert not traversal_geometry_evidence(
        "right_angle_poles", (0.0, 0.0), (1.0, 0.0), (0.1, 0.1), **limits
    )
    assert not traversal_geometry_evidence(
        "high_wall", (0.0, 0.0), (1.0, 0.0), (0.2, 1.5), **limits
    )


def test_traversal_crossing_evidence_rejects_invalid_or_degenerate_pose():
    limits = dict(minimum_displacement=0.45, beyond_obstacle_margin=0.12)
    assert not traversal_crossing_evidence(
        (0.0, 0.0), (0.05, 0.0), (1.0, 0.0), **limits
    )
    assert not traversal_crossing_evidence(
        (0.0, 0.0), (1.0, 0.0), (float("nan"), 0.0), **limits
    )


def test_slope_handoff_has_an_unambiguous_action_type():
    guidance = TraversalGuidance()
    guidance.obstacle_type = TraversalGuidance.OBSTACLE_CLEAR
    guidance.traversal_required = True
    assert action_obstacle_type(guidance) == TraverseObstacle.Goal.OBSTACLE_SLOPE
    guidance.obstacle_type = TraversalGuidance.OBSTACLE_STEP
    assert action_obstacle_type(guidance) == TraverseObstacle.Goal.OBSTACLE_STEP
    guidance.obstacle_type = TraversalGuidance.OBSTACLE_POLE
    assert action_obstacle_type(guidance) == TraverseObstacle.Goal.OBSTACLE_POLE


def test_semantic_name_is_cross_checked_against_action_geometry():
    # 上一帧名称不能把明确的坑 Action 记成木桥。
    assert semantic_id_for_action(
        "wooden_bridge_b", TraverseObstacle.Goal.OBSTACLE_PIT
    ) == "wooden_bridge_b"
    assert semantic_id_for_action(
        "gravel_wood_pit", TraverseObstacle.Goal.OBSTACLE_BAR
    ) == "gravel_wood_pit"
    # STEP 对应多项规则障碍，只有已由名称层确认的兼容语义才能保留，不能凭几何
    # 强猜 A/B/台阶；限高杆近裁剪为 STEP 是整场联调确认的兼容退化。
    assert semantic_id_for_action(
        "t_shaped_stairs", TraverseObstacle.Goal.OBSTACLE_STEP
    ) == "t_shaped_stairs"
    assert semantic_id_for_action(
        "height_bar", TraverseObstacle.Goal.OBSTACLE_STEP
    ) == "height_bar"
    assert semantic_id_for_action(
        "wooden_bridge_unknown", TraverseObstacle.Goal.OBSTACLE_SLOPE
    ) == "wooden_bridge_unknown"
    # Coarse geometry is not a unique competition identity. Pit rails and bridge
    # sides can look like WALL, while one height-bar post can look like POLE.
    assert semantic_id_for_action("", TraverseObstacle.Goal.OBSTACLE_WALL) == ""
    assert semantic_id_for_action("", TraverseObstacle.Goal.OBSTACLE_POLE) == ""
    assert semantic_id_for_action(
        "height_bar", TraverseObstacle.Goal.OBSTACLE_POLE
    ) == "height_bar"


def test_stable_semantic_controls_final_action_when_near_view_degrades():
    assert action_type_for_semantic(
        "high_wall", TraverseObstacle.Goal.OBSTACLE_STEP
    ) == TraverseObstacle.Goal.OBSTACLE_WALL
    assert action_type_for_semantic(
        "height_bar", TraverseObstacle.Goal.OBSTACLE_POLE
    ) == TraverseObstacle.Goal.OBSTACLE_BAR
    assert action_type_for_semantic(
        "wooden_bridge_b", TraverseObstacle.Goal.OBSTACLE_PIT
    ) == TraverseObstacle.Goal.OBSTACLE_STEP
    assert action_type_for_semantic("", TraverseObstacle.Goal.OBSTACLE_PIT) == (
        TraverseObstacle.Goal.OBSTACLE_PIT
    )


def test_competition_names_have_stable_ids_without_world_coordinates():
    assert canonical_obstacle_id("直角绕杆区（立柱）") == "right_angle_poles"
    assert canonical_obstacle_id("砂砾与碎木坑") == "gravel_wood_pit"
    assert canonical_obstacle_id("限高杆（支柱结构）") == "height_bar"
    assert canonical_obstacle_id("主斜坡（10°坡面）") == "main_slope"
    assert canonical_obstacle_id("木桥 B（桥板间隙）") == "wooden_bridge_b"
    assert canonical_obstacle_id("木桥引坡（14°，A/B 待结构确认）") == "wooden_bridge_unknown"
    assert canonical_obstacle_id("台阶或木桥踏板（待结构确认）") == ""
    assert canonical_obstacle_id("T 字形台阶") == "t_shaped_stairs"
    assert canonical_obstacle_id("高墙") == "high_wall"
    assert canonical_obstacle_id("场地边界（禁止越界）") == ""
    assert canonical_obstacle_id("视觉检测到有色障碍") == ""
    assert is_actionable_semantic_id("wooden_bridge_a")
    assert is_actionable_semantic_id("wooden_bridge_b")
    # “桥型待确认”与 T 台首级踏面仍有歧义，只能继续换视角，不能触发盲目前冲。
    assert not is_actionable_semantic_id("wooden_bridge_unknown")
    assert not is_actionable_semantic_id("")


def test_fine_semantics_survive_transient_coarse_geometry_types():
    """近场局部点云退化时仍保留已由多帧结构确认的比赛障碍语义。"""
    assert semantic_id_for_action(
        "t_shaped_stairs", TraverseObstacle.Goal.OBSTACLE_PIT
    ) == "t_shaped_stairs"
    assert semantic_id_for_action(
        "wooden_bridge_a", TraverseObstacle.Goal.OBSTACLE_PIT
    ) == "wooden_bridge_a"
    assert semantic_id_for_action(
        "high_wall", TraverseObstacle.Goal.OBSTACLE_BAR
    ) == "high_wall"
    assert semantic_id_for_action(
        "high_wall", TraverseObstacle.Goal.OBSTACLE_PIT
    ) == "high_wall"
    assert action_type_for_semantic(
        "t_shaped_stairs", TraverseObstacle.Goal.OBSTACLE_PIT
    ) == TraverseObstacle.Goal.OBSTACLE_STEP
    assert action_type_for_semantic(
        "high_wall", TraverseObstacle.Goal.OBSTACLE_BAR
    ) == TraverseObstacle.Goal.OBSTACLE_WALL


def test_two_independent_bridge_results_complete_a_and_b_without_guessing_first():
    completed = resolve_completed_semantics([], "wooden_bridge_unknown")
    assert "wooden_bridge_a" not in completed
    assert "wooden_bridge_b" not in completed
    completed = resolve_completed_semantics(completed, "wooden_bridge_b")
    assert "wooden_bridge_a" in completed
    assert "wooden_bridge_b" in completed
    assert not any(item.startswith("wooden_bridge_unknown") for item in completed)

    two_unknowns = resolve_completed_semantics([], "wooden_bridge_unknown")
    two_unknowns = resolve_completed_semantics(two_unknowns, "wooden_bridge_unknown")
    assert {"wooden_bridge_a", "wooden_bridge_b"}.issubset(two_unknowns)


def test_completed_unique_semantic_is_never_reexecuted_but_bridge_pair_can_finish():
    assert semantic_task_is_complete("gravel_wood_pit", ["gravel_wood_pit"])
    assert not semantic_task_is_complete("high_wall", ["gravel_wood_pit"])
    assert not semantic_task_is_complete(
        "wooden_bridge_unknown", ["wooden_bridge_unknown_1"]
    )
    assert semantic_task_is_complete(
        "wooden_bridge_unknown", ["wooden_bridge_a", "wooden_bridge_b"]
    )


def test_bridge_platform_alone_cannot_be_traversed_without_a_or_b_evidence():
    from quadruped_interfaces.msg import NavigationSafety

    deck = NavigationSafety()
    deck.perception_valid = True
    deck.obstacle_type = NavigationSafety.OBSTACLE_STEP
    deck.obstacle_height = 0.24
    deck.width = 1.0
    deck.roughness = 0.018
    deck.slope_pitch = 0.02
    deck.slope_roll = 0.01
    assert not obstacle_geometry_fits_candidate("wooden_bridge_unknown", deck)


def test_high_wall_geometry_rejects_pit_rail_but_accepts_full_face():
    from quadruped_interfaces.msg import NavigationSafety

    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_WALL
    safety.obstacle_height = 0.25
    safety.width = 1.0
    assert not obstacle_geometry_fits_candidate("high_wall", safety)
    safety.obstacle_height = 0.30
    assert obstacle_geometry_fits_candidate("high_wall", safety)


def test_competition_score_counts_unique_tasks_and_return_bonus():
    completed = ["high_wall", "high_wall", "height_bar"]
    assert mission_score(completed, False) == 300
    assert mission_score(completed, True) == 400
    assert abs(normalized_angle(3.5)) <= 3.141593


def test_inventory_lists_completed_and_pending_in_stable_rule_order():
    expected = ["right_angle_poles", "height_bar", "high_wall"]
    completed, pending = mission_inventory(
        expected,
        ["high_wall", "high_wall", "diagnostic_unknown"],
    )
    assert completed == ("high_wall",)
    assert pending == ("right_angle_poles", "height_bar")
    text = inventory_message(pending)
    assert '"count":2' in text
    assert '"ids":["right_angle_poles","height_bar"]' in text
    assert "直角绕杆区" in text
    display = inventory_display(completed, pending)
    assert "已越过(1/3): 高墙" in display
    assert "未越过(2/3): 直角绕杆区, 限高杆" in display


def test_five_second_watchdog_uses_monotonic_deadline():
    """No-motion/controller waits recover at five seconds, never before the deadline."""
    assert not timeout_reached(10.0, 14.99, 5.0)
    assert timeout_reached(10.0, 15.0, 5.0)
    assert not timeout_reached(0.0, 100.0, 5.0)
    assert not timeout_reached(10.0, 15.0, 0.0)


def test_stall_preserves_confirmed_identity_only_for_same_compatible_entry():
    """Near-field BAR/STEP flicker must not restart the same height-bar approach loop."""
    assert semantic_after_approach_stall(
        "height_bar",
        "high_wall",
        TraverseObstacle.Goal.OBSTACLE_STEP,
        True,
    ) == "height_bar"
    # Spatial disagreement means a neighboring obstacle may use its own live identity,
    # but must never inherit the old height-bar label.
    assert semantic_after_approach_stall(
        "height_bar",
        "high_wall",
        TraverseObstacle.Goal.OBSTACLE_WALL,
        False,
    ) == "high_wall"
    # An incompatible coarse type invalidates the original lock.
    assert semantic_after_approach_stall(
        "right_angle_poles",
        "",
        TraverseObstacle.Goal.OBSTACLE_STEP,
        True,
    ) == ""


def test_shipped_mission_uses_bounded_recovery_and_return_policy():
    """Protect the operator-requested five-second recovery and finite search defaults."""
    from pathlib import Path

    import yaml

    path = Path(__file__).parents[1] / "config" / "autonomous_mission.yaml"
    params = yaml.safe_load(path.read_text(encoding="utf-8"))["autonomous_mission"][
        "ros__parameters"
    ]
    assert params["nav_stall_timeout"] == 5.0
    assert params["startup_sensor_settle_time"] == 1.50
    assert params["return_nav_stall_timeout"] == 20.0
    assert params["odom_progress_timeout"] == 0.5
    assert params["nav_progress_translation"] == 0.04
    assert params["nav_progress_rotation"] == 0.06
    assert params["controller_wait_timeout"] == 5.0
    assert params["approach_stall_handoff_count"] == 1
    assert params["approach_stall_handoff_max_distance"] == 2.35
    assert params["approach_stall_handoff_max_heading_error"] == 0.22
    assert params["direct_handoff_max_distance"] == 1.45
    assert params["maximum_search_turns"] == 8
    assert params["inventory_log_period"] == 5.0
    assert params["mission_timeout"] == 300.0
    assert params["return_time_reserve"] == 60.0
    assert params["obstacle_revisit_max_cooldown"] == 64.0
    assert params["semantic_confirmation_votes"] == 3
    assert params["semantic_recent_window"] == 6
    assert params["semantic_verification_max_attempts"] == 2
    assert params["failed_entry_turn_angle"] == 1.570796
    assert params["failed_entry_settle_time"] == 0.80
    assert params["failed_entry_memory_duration"] == 45.0
    assert params["failed_entry_station_tolerance"] == 0.65
    assert params["failed_entry_heading_tolerance"] == 0.70
    assert params["failed_entry_escape_distance"] == 0.80


def test_only_nav_owned_states_receive_yaw_only_stop_recovery():
    """Approach may turn in place at STOP, but handoff/escape may not bypass it."""
    for purpose in (
        "return_home",
        "frontier",
        "coverage",
        "revisit_obstacle",
        "approach",
        "prealign_obstacle",
        "verify_obstacle",
        "entry_recovery",
        "search_turn",
    ):
        assert navigation_purpose_allows_yaw_only_recovery(purpose)
    for purpose in ("", "handoff", "traversal", "entry_escape"):
        assert not navigation_purpose_allows_yaw_only_recovery(purpose)


def test_active_search_prefers_one_near_known_unfinished_obstacle():
    records = [
        ObservedObstacle(
            "high_wall", 5.0, 0.0, 3.0, 0.0, 0.0, 0.90, 10.0
        ),
        ObservedObstacle(
            "height_bar", 1.5, 0.0, 1.0, 0.0, 0.0, 0.80, 11.0
        ),
    ]
    selected = choose_pending_obstacle(records, [], (0.0, 0.0, 0.0), 12.0)
    assert selected is not None
    assert selected.semantic_id == "height_bar"
    # 完成后必须选择下一项；冷却中的已知任务也应让位给可执行目标。
    assert choose_pending_obstacle(
        records, ["height_bar"], (0.0, 0.0, 0.0), 12.0
    ).semantic_id == "high_wall"
    records[0].retry_after = 20.0
    assert choose_pending_obstacle(
        records, ["height_bar"], (0.0, 0.0, 0.0), 12.0
    ) is None


def test_near_field_generic_geometry_reuses_only_compatible_pending_identity():
    """Regression for the gravel/high-wall revisit loop observed in the run log."""
    records = [
        ObservedObstacle(
            "gravel_wood_pit", 2.0, 0.0, 0.5, 0.0, 0.0, 0.96, 10.0
        ),
        ObservedObstacle(
            "high_wall", 8.0, 0.0, 6.0, 0.0, 0.0, 0.92, 11.0
        ),
    ]
    assert matching_pending_semantic(
        records,
        [],
        (2.18, 0.05),
        TraverseObstacle.Goal.OBSTACLE_STEP,
        0.90,
    ) == "gravel_wood_pit"
    # The same coarse STEP must not inherit a far wall or a task already completed.
    assert matching_pending_semantic(
        records,
        ["gravel_wood_pit"],
        (2.18, 0.05),
        TraverseObstacle.Goal.OBSTACLE_STEP,
        0.90,
    ) == ""


def test_revisit_viewpoint_recovers_identity_despite_long_structure_edge_drift():
    records = [
        ObservedObstacle(
            "t_shaped_stairs", 2.0, 0.0, 0.5, -0.2, 0.30, 0.96, 10.0
        ),
        ObservedObstacle(
            "high_wall", 7.0, 0.0, 5.0, 0.0, 0.0, 0.95, 10.0
        ),
    ]
    # Current nearest tread may have moved far from obstacle_x/y; the robot station
    # and compatible STEP geometry still identify the pending T stair.
    assert matching_pending_semantic_from_viewpoint(
        records, [], (0.55, -0.18, 0.35),
        TraverseObstacle.Goal.OBSTACLE_STEP, 0.60, 0.70,
    ) == "t_shaped_stairs"
    assert matching_pending_semantic_from_viewpoint(
        records, ["t_shaped_stairs"], (0.55, -0.18, 0.35),
        TraverseObstacle.Goal.OBSTACLE_STEP, 0.60, 0.70,
    ) == ""
    assert matching_pending_semantic_from_viewpoint(
        records, [], (0.55, -0.18, 1.20),
        TraverseObstacle.Goal.OBSTACLE_STEP, 0.60, 0.70,
    ) == ""
    assert matching_pending_semantic(
        records,
        [],
        (4.0, 0.0),
        TraverseObstacle.Goal.OBSTACLE_STEP,
        0.90,
    ) == ""


def test_failed_semantic_cooldown_cannot_be_bypassed_by_ledger_fallback():
    """A rejected bridge entry must not be re-armed by the next generic STEP."""
    record = ObservedObstacle(
        "wooden_bridge_b", 2.0, 0.0, 0.5, -0.2, 0.30, 0.96, 10.0,
        retry_after=30.0,
    )
    assert matching_pending_semantic(
        [record], [], (2.1, 0.0), TraverseObstacle.Goal.OBSTACLE_STEP,
        0.90, current_time=20.0,
    ) == ""
    assert matching_pending_semantic_from_viewpoint(
        [record], [], (0.52, -0.18, 0.31),
        TraverseObstacle.Goal.OBSTACLE_STEP, 0.60, 0.70,
        current_time=20.0,
    ) == ""
    # Once the bounded retry deadline expires, identical live geometry may recover
    # the semantic ID and attempt the obstacle from a newly observed entry.
    assert matching_pending_semantic(
        [record], [], (2.1, 0.0), TraverseObstacle.Goal.OBSTACLE_STEP,
        0.90, current_time=31.0,
    ) == "wooden_bridge_b"


def test_rejected_entry_memory_is_local_to_station_heading_and_expires():
    record = FailedEntry("wooden_bridge_b", 0.5, -1.7, 1.57, 50.0)
    assert failed_entry_matches(
        [record], "wooden_bridge_b", (0.55, -1.65, 1.60),
        20.0, 0.65, 0.70,
    )
    # A new observation station, a substantially different heading, another
    # semantic, or an expired record must all remain eligible.
    assert not failed_entry_matches(
        [record], "wooden_bridge_b", (1.30, -1.65, 1.60),
        20.0, 0.65, 0.70,
    )
    assert not failed_entry_matches(
        [record], "wooden_bridge_b", (0.55, -1.65, 0.20),
        20.0, 0.65, 0.70,
    )
    # A long bridge rejected from this station still has the same landing envelope
    # after a pure turn, so the mission must physically obtain a new station.
    assert failed_entry_matches(
        [record], "wooden_bridge_b", (0.55, -1.65, 0.20),
        20.0, 0.65, 0.70, require_new_station=True,
    )
    assert not failed_entry_matches(
        [record], "t_shaped_stairs", (0.55, -1.65, 1.60),
        20.0, 0.65, 0.70,
    )
    assert not failed_entry_matches(
        [record], "wooden_bridge_b", (0.55, -1.65, 1.60),
        51.0, 0.65, 0.70,
    )


def test_failed_obstacle_revisit_uses_bounded_exponential_backoff():
    assert obstacle_revisit_delay(8.0, 1, 64.0) == 8.0
    assert obstacle_revisit_delay(8.0, 2, 64.0) == 16.0
    assert obstacle_revisit_delay(8.0, 4, 64.0) == 64.0
    assert obstacle_revisit_delay(8.0, 20, 64.0) == 64.0


def test_ambiguous_view_counter_tracks_robot_station_not_moving_front_edge():
    # A pure camera turn may move the reported bridge/pit edge by metres, while the
    # base itself stays at one station and must consume the same bounded 1/4 sequence.
    anchor = (1.0, 2.0)
    assert verification_station_matches(anchor, (1.02, 1.98), 1.50)
    assert not verification_station_matches(anchor, (2.6, 2.0), 1.50)
    assert not verification_station_matches(None, anchor, 1.50)


def test_semantic_vote_converges_from_early_generic_guess():
    votes = ["wooden_bridge_unknown", "high_wall", "high_wall"]
    assert select_semantic_vote(votes) == "high_wall"
    # 一帧新类别不足以覆盖已经锁定的稳定结论。
    assert select_semantic_vote(["t_shaped_stairs"], "high_wall") == "high_wall"
    # 票数相同时选择较新的观测，适应接近后出现的更完整结构。
    assert select_semantic_vote(
        ["wooden_bridge_unknown", "high_wall"], ""
    ) == "high_wall"


def test_repeated_planar_evidence_blocks_a_short_close_side_misclassification():
    votes = ["main_slope"] * 7 + ["t_shaped_stairs"] * 3
    assert dominant_planar_vote(votes) == "main_slope"
    # A genuinely different sustained observation must eventually win once
    # the bounded history no longer contains a planar majority.
    corrected = ["main_slope"] * 3 + ["t_shaped_stairs"] * 8
    assert dominant_planar_vote(corrected) == ""


def test_entry_lock_uses_full_bounded_history_not_last_close_crop_only():
    votes = ["height_bar"] * 9 + ["t_shaped_stairs"] * 5
    assert select_semantic_vote(votes) == "t_shaped_stairs"
    assert select_full_semantic_vote(votes) == "height_bar"


def test_near_crop_geometry_does_not_discard_specific_bar_or_slope_semantics():
    assert semantic_id_for_action(
        "height_bar", TraverseObstacle.Goal.OBSTACLE_STEP
    ) == "height_bar"
    assert semantic_id_for_action(
        "height_bar", TraverseObstacle.Goal.OBSTACLE_WALL
    ) == "height_bar"
    assert semantic_id_for_action(
        "main_slope", TraverseObstacle.Goal.OBSTACLE_STEP
    ) == "main_slope"


def test_failed_frontier_exclusion_selects_next_candidate():
    first = Frontier(1.0, 0.0, 20, 1.0, 10.0)
    second = Frontier(0.0, 2.0, 15, 2.0, 5.0)
    assert choose_frontier([first, second], [(1.1, 0.0)], 0.5) == second
    assert choose_frontier([first], [(1.0, 0.0)], 0.5) is None


def test_only_success_or_obstacle_boundary_abort_can_enter_guarded_handoff():
    assert nav_status_allows_guarded_handoff(GoalStatus.STATUS_SUCCEEDED)
    assert nav_status_allows_guarded_handoff(GoalStatus.STATUS_ABORTED)
    assert not nav_status_allows_guarded_handoff(GoalStatus.STATUS_CANCELED)
    assert not nav_status_allows_guarded_handoff(GoalStatus.STATUS_UNKNOWN)


def test_completed_obstacle_filter_keeps_adjacent_different_type():
    completed = [(TraversalGuidance.OBSTACLE_STEP, 1.0, 2.0)]
    assert obstacle_was_completed(
        TraversalGuidance.OBSTACLE_STEP, (1.2, 2.0), completed, 0.65
    )
    assert not obstacle_was_completed(
        TraversalGuidance.OBSTACLE_BAR, (1.2, 2.0), completed, 0.65
    )


def test_completed_segment_requires_same_competition_semantic():
    from quadruped_planning.autonomous_mission import traversal_segment_matches

    start, end = (0.0, 0.0), (4.0, 0.0)
    position = (2.0, 0.1)
    assert traversal_segment_matches(
        "main_slope", "main_slope", position, start, end, 0.65
    )
    assert not traversal_segment_matches(
        "t_shaped_stairs", "main_slope", position, start, end, 0.65
    )
    assert not traversal_segment_matches(
        "", "main_slope", position, start, end, 0.65
    )


def test_mission_has_runtime_stop_and_no_world_coordinate_dependency():
    from pathlib import Path

    source = (
        Path(__file__).parents[1]
        / "quadruped_planning"
        / "autonomous_mission.py"
    ).read_text(encoding="utf-8")
    assert '"/traverse_obstacle"' in source
    assert '"/autonomy/set_enabled"' not in source
    assert '"/autonomy/toggle"' not in source
    assert "robocon_obstacle_field" not in source
    assert "layout_" not in source
    # lifecycle 启动窗口中的 Action reject 不能被当作真正的路径规划失败。
    rejected_branch = source.split("if handle is None or not handle.accepted:", 1)[1].split(
        "self.nav_handle, self.nav_started", 1
    )[0]
    assert "blocked_frontiers.append" not in rejected_branch
    assert "nav_retry_until" in rejected_branch
    assert "minimum_approach_goal_distance" in source
    assert "minimum_alignment_command_angle" in source
    assert "without a zero-distance Nav2 goal" in source
    assert "if not is_actionable_semantic_id(semantic_id):" in source
    assert "approach_within_tolerance" in source
    # Nav2 无法把普通底盘路径规划到坑/墙投影内部时，必须有受限、连续确认的 Action
    # 交接，不能永久重发同一个入口；该旁路仍要同时约束距离、横偏和航向误差。
    assert "approach_stall_handoff_count" in source
    assert "approach_stall_handoff_max_distance" in source
    assert "approach_stall_handoff_max_lateral" in source
    assert "approach_stall_handoff_max_heading_error" in source
    assert "obstacle_failure_cooldown" in source
    assert "blocked_obstacles" in source
    # 语义不明确或入口不安全时，任务不能在原地无限重试：先改变航向，
    # 再通过普通 Nav2 目标移动观察站；检查行为字段而不是易变化的日志措辞。
    assert "ambiguous_recovery_sign" in source
    assert "self.failed_entry_turn_pending =" in source
    assert '"entry_recovery"' in source
    assert '"entry_escape"' in source
    # 返程直线请求被正前障碍锁住时也要进入同一恢复链，而且换站目标必须获准完成，
    # 不能被“已到返程时间”的分支在下一 tick 立即取消。
    stall_recovery = source.split("stalled_purpose = self.nav_purpose", 1)[1].split(
        "if world_to_cell", 1
    )[0]
    assert '"return_home"' in stall_recovery
    assert "guidance_is_blocking" in stall_recovery
    assert 'self.nav_purpose in ("entry_recovery", "entry_escape")' in source
    # Ambiguous geometry may be approached for a better view, but all actual
    # TraverseObstacle construction paths must still reject an unconfirmed ID.
    assert source.count("is_actionable_semantic_id") >= 4
    assert "in COMPETITION_OBSTACLE_IDS" in source
    assert "WAITING_FOR_TRAVERSAL_CONTROLLER" in source
    assert "waiting for /traverse_obstacle controller" in source
    # Nav2 成功回调和周期 READY 分支都必须检查执行器，不能存在假 HANDOFF 旁路。
    assert source.count("not self.traverse_client.server_is_ready()") >= 3
    assert '"/navigation/autonomy_stop"' in source
    assert "_publish_immediate_stop()" in source
    assert '"/perception/front_obstacle_name"' in source
    assert '"RETURNING_TO_FINISH"' in source
    assert '"return_home"' in source
    assert '"/autonomy/finish_pose"' in source
    assert '"/autonomy/completed_obstacles"' in source
    assert '"/autonomy/pending_obstacles"' in source
    assert "choose_pending_obstacle" in source
    assert '"SEEKING_PENDING_OBSTACLE"' in source
    assert "SEARCHING_MISSING_OBSTACLES" in source
    assert "_nav_is_stalled" in source
    cancel_body = source.split('def _cancel_nav(self, reason="replace"):', 1)[1].split(
        "def _send_nav_goal", 1
    )[0]
    assert "self.nav_cancel_reason" in cancel_body
    assert "self.nav_purpose =" not in cancel_body
    assert 'cancel_reason in ("stall", "approach_within_tolerance")' in source
    assert "semantic_votes" in source
    # 三条交接路径（Nav2 结果、READY 周期、Action 发送前）都必须拒绝空语义；
    # 仿真替身绝不能收到会让机器人盲目前冲的匿名障碍。
    assert "handoff rejected: obstacle identity is not stable" in source
    assert "_verify_ambiguous_obstacle" in source
    assert '"VERIFYING_OBSTACLE"' in source
    assert "semantic_verification_max_attempts" in source
    assert "semantic_verification_lock_radius" in source
    assert "semantic_post_turn_settle_time" in source
    assert "semantic_settle_until" in source
    assert 'in ("verify_obstacle", "prealign_obstacle")' in source
    assert "semantic_observation_distance" in source
    assert "if is_actionable_semantic_id(stable_lock):" in source
    # 控制器 success 不能直接改任务账本：必须同时检查 ROS Action 终态，并在主循环
    # 独立验证越过入口平面与落地稳定后，才调用唯一的完成提交函数。
    assert "int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED" in source
    assert '"VERIFYING_TRAVERSAL_RESULT"' in source
    assert "_verify_traversal_completion(robot, now)" in source
    assert "traversal not counted" in source
    start_traverse = source.split("def _start_traverse", 1)[1].split(
        "def _hold_for_traversal_controller", 1
    )[0]
    assert start_traverse.index("if not self.pending_traverse_id:") < start_traverse.index(
        "goal = TraverseObstacle.Goal()"
    )

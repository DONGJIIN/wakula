"""前沿探索和自动任务安全边界的确定性测试。"""

from math import hypot

from nav_msgs.msg import OccupancyGrid

from quadruped_planning.autonomous_mission import (
    Frontier,
    action_obstacle_type,
    choose_frontier,
    extract_frontiers,
    world_to_cell,
)
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


def test_world_to_cell_rejects_robot_beyond_latest_map_boundary():
    grid = map_with_unknown_border()
    assert world_to_cell(grid, -2.75, -2.25) == (0, 0)
    assert world_to_cell(grid, 2.99, 2.49) == (11, 9)
    assert world_to_cell(grid, 3.01, 0.0) is None
    assert world_to_cell(grid, -3.01, 0.0) is None


def test_slope_handoff_has_an_unambiguous_action_type():
    guidance = TraversalGuidance()
    guidance.obstacle_type = TraversalGuidance.OBSTACLE_CLEAR
    guidance.traversal_required = True
    assert action_obstacle_type(guidance) == TraverseObstacle.Goal.OBSTACLE_SLOPE
    guidance.obstacle_type = TraversalGuidance.OBSTACLE_STEP
    assert action_obstacle_type(guidance) == TraverseObstacle.Goal.OBSTACLE_STEP


def test_failed_frontier_exclusion_selects_next_candidate():
    first = Frontier(1.0, 0.0, 20, 1.0, 10.0)
    second = Frontier(0.0, 2.0, 15, 2.0, 5.0)
    assert choose_frontier([first, second], [(1.1, 0.0)], 0.5) == second
    assert choose_frontier([first], [(1.0, 0.0)], 0.5) is None


def test_mission_has_runtime_stop_and_no_world_coordinate_dependency():
    from pathlib import Path

    source = (Path(__file__).parents[1] / "quadruped_planning" / "autonomous_mission.py").read_text(encoding="utf-8")
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
    assert "approach_within_tolerance" in source
    assert "WAITING_FOR_TRAVERSAL_CONTROLLER" in source
    assert "waiting for /traverse_obstacle controller" in source
    # Nav2 成功回调和周期 READY 分支都必须检查执行器，不能存在假 HANDOFF 旁路。
    assert source.count("not self.traverse_client.server_is_ready()") >= 3

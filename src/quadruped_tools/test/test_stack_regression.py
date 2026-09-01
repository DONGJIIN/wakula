"""Tests for the reusable Gazebo/SLAM/Nav2 regression reporter."""

import json
import math
from pathlib import Path
import shutil
import time

import rclpy
from quadruped_interfaces.msg import TraversalGuidance
from quadruped_tools.stack_regression import (
    CORE_PROCESS_EXECUTABLES,
    HeaderAgeStats,
    ResourceStats,
    StackRegression,
    StreamStats,
    _angle_error,
    _matching_core_executable,
    closed_path_consistency_report,
    health_measurement_passes,
    mapping_cycle_segments,
    parse_args,
)


def test_tools_manifest_declares_generated_message_dependency():
    """stack_regression directly imports quadruped_interfaces on a fresh install."""
    manifest = (Path(__file__).parents[1] / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>quadruped_interfaces</exec_depend>" in manifest


def test_angle_error_wraps_at_pi_boundary():
    """回环偏航误差不能把 +179°/-179° 错报为 358°。"""
    assert _angle_error(math.radians(179), math.radians(-179)) < math.radians(3)


def test_mapping_cycles_alternate_true_rectangle_and_reverse_rotation_coverage():
    """默认轨迹必须既走异路空间闭合，也保留倒退和两个方向的连续整圈旋转。"""
    rectangle_name, rectangle = mapping_cycle_segments(0, 0.16, 0.45)
    reverse_name, reverse = mapping_cycle_segments(1, 0.16, 0.45)
    assert rectangle_name == "rectangle_closed_path"
    assert reverse_name == "bidirectional_rotation_and_reverse"
    assert len([segment for segment in rectangle if segment.linear_x > 0.0]) == 4
    assert len([segment for segment in rectangle if segment.angular_z != 0.0]) == 4
    assert any(segment.linear_x < 0.0 for segment in reverse)
    assert any(segment.angular_z > 0.0 for segment in reverse)
    assert any(segment.angular_z < 0.0 for segment in reverse)

    # 用理想分段运动积分证明四边不是“原路前进再倒退”，且终点/朝向形成闭合。
    x = y = yaw = 0.0
    visited = []
    for segment in rectangle:
        x += math.cos(yaw) * segment.linear_x * segment.duration
        y += math.sin(yaw) * segment.linear_x * segment.duration
        yaw += segment.angular_z * segment.duration
        visited.append((x, y))
    assert len({(round(px, 3), round(py, 3)) for px, py in visited}) >= 4
    assert math.hypot(x, y) < 1e-9
    assert _angle_error(0.0, yaw) < 1e-9
    # 加入矩形后两轮总时长不能超过旧方案连续跑两轮，保护默认长测预算。
    assert sum(item.duration for item in rectangle) < sum(
        item.duration for item in reverse
    )


def test_closed_path_report_is_honest_and_separates_map_from_odom():
    """小闭合误差只能叫位姿一致性，禁止在报告中宣称已证明图优化回环。"""
    cycles = [
        {
            "cycle": 1,
            "trajectory": "rectangle_closed_path",
            "map_position_error_m": 0.12,
            "map_yaw_error_rad": 0.08,
            "odom_position_error_m": 0.21,
            "odom_yaw_error_rad": 0.11,
        },
        {
            "cycle": 2,
            "trajectory": "bidirectional_rotation_and_reverse",
            "map_position_error_m": 0.20,
            "map_yaw_error_rad": 0.10,
            "odom_position_error_m": 0.31,
            "odom_yaw_error_rad": 0.15,
        },
    ]
    report, passed = closed_path_consistency_report(cycles)
    assert passed
    assert report["map_max_position_error_m"] == 0.20
    assert report["map_max_yaw_error_rad"] == 0.10
    assert report["odom_max_position_error_m"] == 0.31
    assert report["odom_max_yaw_error_rad"] == 0.15
    assert report["expected_cycles"] == 2
    assert report["completed_cycles"] == 2
    assert report["missing_cycles"] == []
    assert report["invalid_cycles"] == []
    assert report["proves_slam_toolbox_loop_closure_optimization"] is False
    assert "odometry-drift bag" in report["limitation"]

    odom_only, passed = closed_path_consistency_report(
        [{"odom_position_error_m": 0.0, "odom_yaw_error_rad": 0.0}]
    )
    assert not passed
    assert odom_only["map_max_position_error_m"] is None


def test_closed_path_report_fails_when_any_requested_cycle_loses_map_tf():
    """一轮好结果不能掩盖后续周期起点或终点 TF 查询失败。"""
    report, passed = closed_path_consistency_report(
        [
            {
                "cycle": 1,
                "map_position_error_m": 0.10,
                "map_yaw_error_rad": 0.08,
            },
            {"cycle": 2, "trajectory": "bidirectional_rotation_and_reverse"},
        ],
        expected_cycles=3,
    )
    assert not passed
    assert report["expected_cycles"] == 3
    assert report["completed_cycles"] == 1
    assert report["missing_cycles"] == [2, 3]
    assert report["invalid_cycles"] == []


def test_closed_path_report_rejects_and_sanitizes_non_finite_map_metrics():
    """NaN/Inf 不得被 max 的顺序语义忽略，也不得写成非标准 JSON 数字。"""
    report, passed = closed_path_consistency_report(
        [
            {
                "cycle": 1,
                "map_position_error_m": 0.10,
                "map_yaw_error_rad": 0.08,
            },
            {
                "cycle": 2,
                "map_position_error_m": float("nan"),
                "map_yaw_error_rad": float("inf"),
                "odom_position_error_m": float("inf"),
                "odom_yaw_error_rad": 0.0,
            },
        ]
    )
    assert not passed
    assert report["completed_cycles"] == 1
    assert report["missing_cycles"] == []
    assert report["invalid_cycles"] == [2]
    assert report["cycles"][1]["map_position_error_m"] is None
    assert report["cycles"][1]["map_yaw_error_rad"] is None
    assert report["cycles"][1]["odom_position_error_m"] is None
    # Regression artifacts must remain strict JSON for CI/report consumers.
    json.dumps(report, allow_nan=False)


def test_stream_stats_tracks_largest_gap_without_exposing_internal_clock():
    stats = StreamStats()
    stats.update()
    time.sleep(0.002)
    stats.update()
    public = stats.public()
    assert public["count"] == 2
    assert public["max_gap_seconds"] > 0.0
    assert public["current_age_seconds"] is not None
    assert "_last_time" not in public


def test_stream_stats_detects_tail_dropout_and_accepts_a_continuous_stream():
    """A lone early message must not look healthy merely because max_gap is still zero."""
    lone = StreamStats(monotonic=lambda: 10.0)
    lone.update()
    lone_report = lone.public(now_seconds=12.1)
    assert lone_report["max_gap_seconds"] == 0.0
    assert lone_report["current_age_seconds"] == 2.1
    assert lone.current_age_seconds(12.1) > 2.0

    moments = iter((20.0, 20.4, 20.8))
    continuous = StreamStats(monotonic=lambda: next(moments))
    continuous.update()
    continuous.update()
    continuous.update()
    continuous_report = continuous.public(now_seconds=21.0)
    assert continuous_report["max_gap_seconds"] == 0.4
    assert continuous_report["current_age_seconds"] == 0.2
    assert continuous.current_age_seconds(21.0) < 1.0


def test_regression_requires_explicit_motion_permission_and_safe_defaults():
    options = parse_args([])
    assert not options.allow_motion
    assert options.command_topic == "/cmd_vel_teleop"
    assert options.cycles >= 3
    assert options.pipeline_latency_budget > 0.0
    assert options.use_sim_time is True
    assert parse_args(["--no-use-sim-time"]).use_sim_time is False
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "stack_regression = quadruped_tools.stack_regression:main" in setup


def test_regression_node_uses_gazebo_clock_by_default():
    """Header age 与 Action 目标必须默认和被测 Gazebo 数据处于同一时钟域。"""
    rclpy.init()
    node = None
    try:
        node = StackRegression("/test/cmd_vel")
        assert node.get_parameter("use_sim_time").value is True
        assert node.get_clock().ros_time_is_active
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_resource_sampler_has_no_optional_python_dependency():
    stats = ResourceStats()
    stats.sample()
    report = stats.public()
    assert report["samples"] == 1
    assert report["peak_rss_mib"] >= 0.0


def test_resource_executable_match_never_scans_unrelated_arguments():
    """Only argv[0], or argv[1] for a Python console script, identifies a core process."""
    assert (
        _matching_core_executable(b"/opt/ros/lib/nav2/velocity_smoother\0")
        == "velocity_smoother"
    )
    assert (
        _matching_core_executable(
            b"/usr/bin/python3\0/opt/wakula/lib/slam/navigation_health_monitor\0"
        )
        == "navigation_health_monitor"
    )
    assert (
        _matching_core_executable(b"/usr/bin/python3.12\0/opt/wakula/terrain_analyzer\0")
        == "terrain_analyzer"
    )
    assert _matching_core_executable(b"/bin/sleep\0velocity_smoother\0") is None


def _write_fake_process(
    proc_root: Path,
    pid: int,
    executable: str,
    cpu_ticks: int,
    rss_pages: int,
    start_ticks: int,
) -> None:
    """写入 ResourceStats 所需的最小 Linux /proc 快照。"""
    process = proc_root / str(pid)
    process.mkdir(parents=True, exist_ok=True)
    (process / "cmdline").write_bytes(
        b"/usr/bin/python3\0" + f"/opt/ros/bin/{executable}".encode() + b"\0"
    )
    fields = ["0"] * 52
    fields[0] = str(pid)
    # comm 故意包含空格，验证解析器使用右括号定位而不是脆弱的整行 split。
    fields[1] = "(ros core process)"
    fields[2] = "S"
    fields[13] = str(cpu_ticks)
    fields[14] = "0"
    fields[21] = str(start_ticks)
    fields[23] = str(rss_pages)
    (process / "stat").write_text(" ".join(fields), encoding="utf-8")


def test_resource_sampler_counts_full_whitelist_and_pid_churn(tmp_path):
    """资源汇总必须计入完整 Nav2 链，并把重启与累计 CPU 分开处理。"""
    required = {
        "behavior_server",
        "velocity_smoother",
        "navigation_health_monitor",
        "nav2_readiness_monitor",
    }
    assert required <= CORE_PROCESS_EXECUTABLES
    # Path smoother is not used by Wakula's behavior tree.  Keep only the velocity
    # smoother, which limits acceleration on the actual cmd_vel chain.
    assert "smoother_server" not in CORE_PROCESS_EXECUTABLES

    moments = iter((10.0, 11.0, 12.0))
    stats = ResourceStats(
        proc_root=tmp_path,
        clock_ticks_per_second=100.0,
        page_size_bytes=4096,
        monotonic=lambda: next(moments),
    )
    _write_fake_process(tmp_path, 101, "velocity_smoother", 100, 10, 1000)
    # argv 参数里出现核心节点名不能误计，只有可执行文件 basename 才属于白名单。
    _write_fake_process(tmp_path, 999, "sleep", 500, 50, 900)
    (tmp_path / "999" / "cmdline").write_bytes(
        b"/bin/sleep\0--label=velocity_smoother\0"
    )
    stats.sample()
    assert stats.public()["active_process_count"] == 1

    _write_fake_process(tmp_path, 101, "velocity_smoother", 150, 12, 1000)
    _write_fake_process(tmp_path, 202, "behavior_server", 900, 20, 2000)
    stats.sample()
    second = stats.public()
    assert second["peak_cpu_percent_one_core"] == 50.0
    assert second["pid_start_events_after_baseline"] == 1
    assert second["active_process_count"] == 2

    shutil.rmtree(tmp_path / "101")
    stats.sample()
    final = stats.public()
    assert final["pid_exit_events_after_baseline"] == 1
    assert final["peak_process_count"] == 2
    assert final["observed_process_identities"] == 2
    assert final["matched_executables"] == ["behavior_server", "velocity_smoother"]


def test_header_age_stats_reports_percentiles_invalid_stamps_and_soft_budget():
    """流水线频率正常时，历史帧积压仍必须由 Header 年龄显式暴露。"""
    stats = HeaderAgeStats(soft_budget_seconds=0.25)
    for age in (0.10, 0.20, 0.30, 0.40):
        stats.update(10, 0, 10.0 + age)
    stats.update(0, 0, 10.0)
    stats.update(11, 0, 10.0)
    report = stats.public()
    assert report["sample_count"] == 4
    assert report["invalid_stamp_count"] == 2
    assert report["p50_age_seconds"] == 0.20
    assert report["p95_age_seconds"] == 0.40
    assert report["max_age_seconds"] == 0.40
    assert report["samples_over_soft_budget"] == 2
    assert report["p95_within_soft_budget"] is False


def test_navigation_health_uses_a_clean_post_readiness_measurement_window():
    """启动期 lifecycle false 可忽略，测量期任一 false 必须令回归失败。"""
    node = object.__new__(StackRegression)
    node.health_true = 2
    node.health_false = 7
    node.latest_health = False
    node.health_measurement_started = False
    node.measurement_health_true = 0
    node.measurement_health_false = 0
    node.startup_health_true = 0
    node.startup_health_false = 0

    node._begin_health_measurement()
    assert node.startup_health_true == 2
    assert node.startup_health_false == 7
    assert not health_measurement_passes(True, 0, 0)

    node._health_callback(type("Message", (), {"data": True})())
    assert health_measurement_passes(
        node.health_measurement_started,
        node.measurement_health_true,
        node.measurement_health_false,
    )

    node._health_callback(type("Message", (), {"data": False})())
    assert not health_measurement_passes(
        node.health_measurement_started,
        node.measurement_health_true,
        node.measurement_health_false,
    )


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

"""对比赛场地的关键尺寸、颜色和算法隔离做静态回归检查。

这些测试刻意只锁定 2026 官方 V2.0 已经公布的数据。障碍的全局 pose 仍是参考布局，
正式坐标公布后允许修改，不应因此修改 SLAM、Nav2 或 OpenCV 源码。
"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Twist
import pytest
from std_msgs.msg import Bool
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE_ROOT / "worlds" / "robocon_obstacle_field.sdf"
WORLD = ET.parse(WORLD_PATH).getroot().find("world")
ROBOT_PATH = PACKAGE_ROOT / "models" / "generic_quadruped" / "model.sdf"
ROBOT = ET.parse(ROBOT_PATH).getroot().find("model")
ORANGE = [223.0 / 255.0, 117.0 / 255.0, 0.0, 1.0]


def model(name: str) -> ET.Element:
    """按唯一名称取得顶层模型，缺失时让测试给出直接可读的错误。"""
    found = WORLD.find(f"model[@name='{name}']")
    assert found is not None, f"missing model: {name}"
    return found


def collision_box(model_name: str, collision_name: str) -> list[float]:
    """返回指定碰撞盒尺寸，碰撞体比 visual 更能代表实际可通行几何。"""
    node = model(model_name).find(f".//collision[@name='{collision_name}']/geometry/box/size")
    assert node is not None and node.text
    return [float(value) for value in node.text.split()]


def assert_close(actual: list[float], expected: list[float], tolerance: float = 1e-6):
    assert len(actual) == len(expected)
    assert all(abs(a - e) <= tolerance for a, e in zip(actual, expected)), (actual, expected)


def layout_pose(name: str) -> list[float]:
    """读取集中式参考布局；只用于检查模型互不重叠，不把坐标带进算法。"""
    node = WORLD.find(f"frame[@name='layout_{name}']/pose")
    assert node is not None and node.text
    return [float(value) for value in node.text.split()]


def test_all_eight_rule_obstacles_exist():
    expected = {
        "right_angle_poles",
        "gravel_wood_pit",
        "height_bar",
        "main_slope",
        "wooden_bridge_a",
        "wooden_bridge_b",
        "t_shaped_stairs",
        "high_wall",
    }
    assert expected.issubset({item.attrib["name"] for item in WORLD.findall("model")})


def test_gazebo_field_does_not_load_algorithms_or_traversal_controller():
    """唯一场地入口只提供环境/传感器，不能装载算法或越障执行器。"""
    launch = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text(
        encoding="utf-8"
    )
    assert "sim_traverse_obstacle" not in launch
    assert 'package_file("slam"' not in launch
    assert "autonomous_navigation.launch.py" not in launch
    assert "autonomous_mission" not in launch
    assert "quadruped_teleop" not in launch
    assert '"enable_point_cloud_bridge"' in launch
    assert 'default_value="true"' in launch
    timed_launch = (
        PACKAGE_ROOT / "launch" / "robocon_field_teleport.launch.py"
    ).read_text(encoding="utf-8")
    assert '"benchmark_raw_point_cloud"' in timed_launch
    assert '"benchmark_staging"' in timed_launch
    assert '"benchmark_semantic_hint"' in timed_launch
    assert "xbox_teleop.launch.py" not in timed_launch
    mux = (PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py").read_text(encoding="utf-8")
    assert '"/navigation/autonomy_stop"' in mux
    assert "if autonomy_stop:" in mux


def test_sim_traversal_backend_is_one_shot_and_yields_cpu():
    """Teleport must use one pose call and never starve SLAM health heartbeats."""
    backend = (PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py").read_text(
        encoding="utf-8"
    )
    assert "executor.spin_once(timeout_sec=0.05)" in backend
    assert "time.sleep(0.020)" in backend
    # A/B 尚未分清时不能调用 Action；服务器不保留任何 unknown 桥的隐式落点。
    assert "wooden_bridge_unknown_span" not in backend
    assert '"wooden_bridge_b_span", 5.20' in backend
    assert '"wooden_bridge_exit_clearance", 0.35' in backend
    assert '"long_structure_exit_clearance", 0.75' in backend
    assert "pose_update_rate" not in backend
    assert "duration_scale" not in backend
    # Each event remains one SetEntityPose call: Action exit and the later benchmark
    # observation station are separate, explicitly logged simulation operations.
    assert backend.count("self._set_model_pose(") == 2
    assert "simulation teleporting to obstacle exit" in backend
    assert "SIMULATION ONLY benchmark staged at" in backend
    assert "BENCHMARK_TASK_ORDER[0] if benchmark_enabled" in backend
    assert "ReentrantCallbackGroup" in backend
    assert "rejecting TraverseObstacle goal:" in backend
    assert "benchmark_semantic_hint_enabled" in backend
    assert '"benchmark_staging_enabled", False' in backend
    assert '"benchmark_semantic_hint_enabled", False' in backend
    assert '"benchmark_staging_delay", 0.0' in backend
    assert '"benchmark_semantic_hint_settle", 0.25' in backend
    assert "            0.10," in backend
    assert '"/perception/fused_obstacle"' in backend
    assert '"/terrain/navigation_safety"' not in backend
    assert '"/traversal/guidance"' not in backend
    assert '"/autonomy/finish_pose"' in backend
    assert "finish contract synchronized after staging home" in backend
    # The Gazebo helper may emulate a standard fused sensor contract. Completion remains
    # exclusively owned by the autonomous mission after Action/posterior checks.
    assert 'create_publisher(\n            String, "/autonomy/completed_obstacles"' not in backend


def _load_sim_traverse_module(name="sim_traverse_contract"):
    """Load the installed-interface-aware simulator helper for pure contract tests."""
    path = PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _valid_traversal_goal(module, stamp_seconds=100):
    """Create one coherent PREPARING snapshot without starting an Action server."""
    goal = module.TraverseObstacle.Goal()
    goal.header.stamp.sec = int(stamp_seconds)
    goal.header.frame_id = "base_link"
    goal.obstacle_type = goal.OBSTACLE_STEP
    goal.obstacle_id = "t_shaped_stairs"
    goal.entry_stage = goal.ENTRY_PREPARING
    goal.confidence = 0.90
    goal.distance = 1.00
    goal.lateral_offset = 0.10
    goal.heading_error = 0.10
    goal.obstacle_height = 0.18
    goal.pit_depth = 0.0
    goal.slope_pitch = 0.0
    goal.slope_roll = 0.0
    goal.roughness = 0.01
    goal.width = 1.0
    goal.structure_heading = 0.0
    goal.structure_heading_confidence = 0.80
    goal.clearance_height = 0.0
    goal.valid_points = 500
    return goal


def _validate_goal(module, goal, now_seconds=100.25):
    return module.validate_traversal_goal(
        goal,
        now_seconds=now_seconds,
        required_frame="base_link",
        maximum_snapshot_age=0.75,
        maximum_future_skew=0.05,
        maximum_entry_distance=2.50,
        maximum_lateral_offset=0.35,
        maximum_alignment_error=0.22,
        ready_entry_distance=0.45,
        ready_lateral_offset=0.10,
        ready_alignment_error=0.08,
    )


def test_traverse_action_goal_is_one_timestamped_geometry_snapshot():
    """Action Goal must carry every controller input under one Header."""
    action = (
        PACKAGE_ROOT.parent / "quadruped_interfaces" / "action" / "TraverseObstacle.action"
    ).read_text(encoding="utf-8")
    required_fields = (
        "std_msgs/Header header",
        "uint8 entry_stage",
        "float32 obstacle_height",
        "float32 pit_depth",
        "float32 slope_pitch",
        "float32 slope_roll",
        "float32 roughness",
        "float32 width",
        "float32 structure_heading",
        "float32 structure_heading_confidence",
        "float32 clearance_height",
        "uint32 valid_points",
    )
    for field in required_fields:
        assert field in action
    assert "uint8 ENTRY_READY=1" in action
    assert "uint8 ENTRY_PREPARING=2" in action
    assert "不得立即抬腿" in action


def test_sim_traverse_accepts_one_fresh_finite_preparing_snapshot():
    module = _load_sim_traverse_module()
    assert _validate_goal(module, _valid_traversal_goal(module)) == ""


def test_sim_traverse_capability_table_requires_actionable_canonical_pairs():
    """The controller must not infer a route from an unknown or coarse final type."""
    module = _load_sim_traverse_module("sim_traverse_capabilities")
    expected = {
        "right_angle_poles": module.TraverseObstacle.Goal.OBSTACLE_POLE,
        "gravel_wood_pit": module.TraverseObstacle.Goal.OBSTACLE_PIT,
        "height_bar": module.TraverseObstacle.Goal.OBSTACLE_BAR,
        "main_slope": module.TraverseObstacle.Goal.OBSTACLE_SLOPE,
        "wooden_bridge_a": module.TraverseObstacle.Goal.OBSTACLE_STEP,
        "wooden_bridge_b": module.TraverseObstacle.Goal.OBSTACLE_STEP,
        "t_shaped_stairs": module.TraverseObstacle.Goal.OBSTACLE_STEP,
        "high_wall": module.TraverseObstacle.Goal.OBSTACLE_WALL,
    }
    assert module.SIM_TRAVERSAL_CAPABILITIES == expected
    for semantic_id, canonical_type in expected.items():
        goal = _valid_traversal_goal(module)
        goal.obstacle_id = semantic_id
        goal.obstacle_type = canonical_type
        assert _validate_goal(module, goal) == ""

    for semantic_id in ("", "test_step", "wooden_bridge_unknown"):
        goal = _valid_traversal_goal(module)
        goal.obstacle_id = semantic_id
        assert "unknown or not actionable" in _validate_goal(module, goal)

    # Near-field classifiers may temporarily call the bar a pole/step and the high
    # wall a bar/step.  The mission must canonicalize those final Action types; the
    # controller rejects the coarse alternatives instead of selecting another motion.
    for semantic_id, coarse_type in (
        ("height_bar", module.TraverseObstacle.Goal.OBSTACLE_POLE),
        ("height_bar", module.TraverseObstacle.Goal.OBSTACLE_STEP),
        ("high_wall", module.TraverseObstacle.Goal.OBSTACLE_BAR),
        ("high_wall", module.TraverseObstacle.Goal.OBSTACLE_STEP),
    ):
        goal = _valid_traversal_goal(module)
        goal.obstacle_id = semantic_id
        goal.obstacle_type = coarse_type
        assert "requires canonical obstacle_type" in _validate_goal(module, goal)


def test_sim_traverse_validates_raw_header_fields_before_float_conversion():
    """Malformed builtin Time fields may not normalize into a different timestamp."""
    module = _load_sim_traverse_module("sim_traverse_raw_stamp")
    negative_seconds = _valid_traversal_goal(module)
    negative_seconds.header.stamp.sec = -1
    assert "sec must be non-negative" in _validate_goal(module, negative_seconds)

    overflowing_nanoseconds = _valid_traversal_goal(module)
    overflowing_nanoseconds.header.stamp.nanosec = 1_000_000_000
    assert "nanosec must be" in _validate_goal(module, overflowing_nanoseconds)


def test_goal_time_odometry_freezes_entry_despite_execution_pose_motion():
    """A delayed execute callback cannot translate Goal.distance from current odom."""
    module = _load_sim_traverse_module("sim_traverse_frozen_entry")
    goal = _valid_traversal_goal(module)
    history = (
        module.PlanarPoseSample(stamp=100.0, x=0.0, y=0.0, yaw=0.0),
        module.PlanarPoseSample(stamp=100.2, x=0.2, y=0.0, yaw=0.0),
    )
    frozen, reason = module.frozen_entry_from_history(
        goal, history, maximum_gap=0.15, standoff=0.20
    )
    assert reason == ""
    assert frozen.snapshot_x == pytest.approx(0.0)
    assert frozen.target_x == pytest.approx(0.80)
    assert frozen.target_y == pytest.approx(0.10)
    assert frozen.remaining_distance == pytest.approx(0.20)

    # By execution time the robot may already report x=0.30.  The frozen target is
    # still x=0.80, not the erroneous current_x + (distance-standoff) = 1.10.
    execution_pose_x = 0.30
    assert frozen.target_x != pytest.approx(execution_pose_x + 0.80)

    interpolated_goal = _valid_traversal_goal(module)
    interpolated_goal.header.stamp.nanosec = 100_000_000
    interpolated, reason = module.frozen_entry_from_history(
        interpolated_goal, history, maximum_gap=0.15, standoff=0.20
    )
    assert reason == ""
    assert interpolated.snapshot_x == pytest.approx(0.10)
    assert interpolated.target_x == pytest.approx(0.90)


def test_goal_time_odometry_rejects_missing_or_distant_history():
    """No pose near header.stamp means no safe frame in which to execute the Goal."""
    module = _load_sim_traverse_module("sim_traverse_missing_history")
    goal = _valid_traversal_goal(module)
    frozen, reason = module.frozen_entry_from_history(
        goal, (), maximum_gap=0.15, standoff=0.20
    )
    assert frozen is None
    assert "odometry history unavailable" in reason

    # Exercise the real admission callback as well: syntactically valid geometry is
    # still rejected before reservation when no odometry can anchor header.stamp.
    node = module.SimTraverseObstacle.__new__(module.SimTraverseObstacle)
    node.odom_history_lock = module.Lock()
    node.odom_history = module.deque(maxlen=16)
    node.busy = False
    node.emergency_stop = False
    node.active_action_stop_latched = False
    node.active_entry_snapshot = None
    parameter_values = {
        "goal_frame_id": "base_link",
        "maximum_snapshot_age": 0.75,
        "maximum_future_skew": 0.05,
        "maximum_entry_distance": 2.50,
        "maximum_lateral_offset": 0.35,
        "maximum_alignment_error": 0.22,
        "ready_entry_distance": 0.45,
        "ready_lateral_offset": 0.10,
        "ready_alignment_error": 0.08,
        "odometry_snapshot_max_gap": 0.15,
        "preparation_standoff": 0.20,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameter_values[name])
    now = SimpleNamespace(nanoseconds=100_250_000_000)
    node.get_clock = lambda: SimpleNamespace(now=lambda: now)
    warnings = []
    node.get_logger = lambda: SimpleNamespace(
        warning=lambda message: warnings.append(message)
    )
    assert node.goal_callback(goal) == module.GoalResponse.REJECT
    assert not node.busy
    assert warnings and "odometry history unavailable" in warnings[-1]

    distant = (module.PlanarPoseSample(stamp=99.0, x=0.0, y=0.0, yaw=0.0),)
    frozen, reason = module.frozen_entry_from_history(
        goal, distant, maximum_gap=0.15, standoff=0.20
    )
    assert frozen is None
    assert "odometry history unavailable" in reason


@pytest.mark.parametrize(
    ("mutation", "reason"),
    (
        (lambda goal: setattr(goal.header.stamp, "sec", 0), "header.stamp"),
        (lambda goal: setattr(goal.header, "frame_id", "odom"), "frame_id"),
        (lambda goal: setattr(goal, "entry_stage", 0), "entry_stage"),
        (lambda goal: setattr(goal, "obstacle_height", math.nan), "not finite"),
        (lambda goal: setattr(goal, "width", -0.01), "width"),
        (lambda goal: setattr(goal, "valid_points", 0), "valid_points"),
        (lambda goal: setattr(goal, "confidence", 0.0), "confidence"),
    ),
)
def test_sim_traverse_rejects_incomplete_or_corrupted_snapshots(mutation, reason):
    module = _load_sim_traverse_module(f"sim_traverse_invalid_{reason}")
    goal = _valid_traversal_goal(module)
    mutation(goal)
    assert reason in _validate_goal(module, goal)


def test_sim_traverse_rejects_stale_future_and_false_ready_snapshots():
    module = _load_sim_traverse_module("sim_traverse_snapshot_time")
    stale = _valid_traversal_goal(module, stamp_seconds=98)
    assert "stale" in _validate_goal(module, stale)
    future = _valid_traversal_goal(module, stamp_seconds=101)
    assert "future" in _validate_goal(module, future)

    false_ready = _valid_traversal_goal(module)
    false_ready.entry_stage = false_ready.ENTRY_READY
    assert "ENTRY_READY distance" in _validate_goal(module, false_ready)
    false_ready.distance = 0.30
    false_ready.heading_error = 0.04
    assert _validate_goal(module, false_ready) == ""


def test_sim_preparing_envelope_covers_the_mission_handoff_contract():
    """Mission may never authorize a PREPARING Goal that this server rejects."""
    module = _load_sim_traverse_module("sim_traverse_cross_package_envelope")
    config_path = (
        PACKAGE_ROOT.parent
        / "quadruped_planning"
        / "config"
        / "autonomous_mission.yaml"
    )
    mission = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "autonomous_mission"
    ]["ros__parameters"]

    assert mission["approach_stall_handoff_max_distance"] <= (
        module.DEFAULT_MAXIMUM_ENTRY_DISTANCE
    )
    assert mission["direct_handoff_max_distance"] <= (
        module.DEFAULT_MAXIMUM_ENTRY_DISTANCE
    )
    assert mission["handoff_fallback_max_distance"] <= (
        module.DEFAULT_MAXIMUM_ENTRY_DISTANCE
    )
    assert mission["approach_stall_handoff_max_lateral"] <= (
        module.DEFAULT_MAXIMUM_LATERAL_OFFSET
    )
    assert mission["handoff_fallback_max_lateral"] <= (
        module.DEFAULT_MAXIMUM_LATERAL_OFFSET
    )
    assert mission["approach_stall_handoff_max_heading_error"] <= (
        module.DEFAULT_MAXIMUM_ALIGNMENT_ERROR
    )

    # Exercise the exact widest mission boundary through the server's real validator.
    boundary = _valid_traversal_goal(module)
    boundary.distance = mission["approach_stall_handoff_max_distance"]
    boundary.lateral_offset = mission["approach_stall_handoff_max_lateral"]
    boundary.heading_error = mission["approach_stall_handoff_max_heading_error"]
    assert _validate_goal(module, boundary) == ""

    # READY deliberately remains much tighter even though PREPARING is compatible.
    boundary.entry_stage = boundary.ENTRY_READY
    assert "ENTRY_READY" in _validate_goal(module, boundary)

    # The combined Gazebo launch must not silently widen the same server contract.
    combined_launch = (
        PACKAGE_ROOT / "launch" / "robocon_field_teleport.launch.py"
    ).read_text(encoding="utf-8")
    alignment_start = combined_launch.index('"maximum_alignment_error"')
    alignment_argument = combined_launch[alignment_start:]
    assert 'default_value="0.22"' in alignment_argument
    assert 'default_value="0.24"' not in alignment_argument


def test_sim_preparation_timeout_has_margin_for_the_full_server_envelope():
    """The old 15 s timeout was below a conservative max-speed motion budget."""
    module = _load_sim_traverse_module("sim_traverse_preparation_budget")
    maximum_translation = math.hypot(
        module.DEFAULT_MAXIMUM_ENTRY_DISTANCE
        - module.DEFAULT_PREPARATION_STANDOFF,
        module.DEFAULT_MAXIMUM_LATERAL_OFFSET,
    )
    # Budget independent worst axes: full planar approach plus a quarter turn and
    # the accepted final alignment.  Proportional slowdown/ROS scheduling need the
    # remaining margin rather than being hidden by a nominal 15 s value.
    conservative_motion_budget = (
        maximum_translation / module.DEFAULT_PREPARATION_LINEAR_SPEED
        + (math.pi / 2.0 + module.DEFAULT_MAXIMUM_ALIGNMENT_ERROR)
        / module.DEFAULT_PREPARATION_ANGULAR_SPEED
    )
    assert conservative_motion_budget > 15.0
    assert module.DEFAULT_PREPARATION_TIMEOUT >= conservative_motion_budget + 4.0


def test_sim_software_stop_poison_survives_clear_until_a_new_goal():
    """A short B-key stop may not let the interrupted Action resume after Start."""
    module = _load_sim_traverse_module("sim_traverse_stop_latch")
    node = module.SimTraverseObstacle.__new__(module.SimTraverseObstacle)
    node.stop_state_lock = module.Lock()
    node.emergency_stop = False
    node.active_action_stop_latched = False
    node.busy = True
    stop_calls = []
    node._stop = lambda: stop_calls.append(True)

    node._emergency_stop_callback(Bool(data=True))
    assert node.emergency_stop
    assert node.active_action_stop_latched
    assert node._software_stop_blocks_motion()
    assert stop_calls

    node._emergency_stop_callback(Bool(data=False))
    assert not node.emergency_stop
    assert node.active_action_stop_latched
    assert node._software_stop_blocks_motion()

    node._release_action_reservation()
    assert not node.busy
    assert not node._software_stop_blocks_motion()
    assert node._reserve_action_if_safe() == ""
    node._release_action_reservation()


def test_sim_software_stop_rejects_new_goal_and_blocks_pose_dispatch():
    """A latched stop covers SetEntityPose, not only the final Twist mux."""
    module = _load_sim_traverse_module("sim_traverse_stop_pose_boundary")
    node = module.SimTraverseObstacle.__new__(module.SimTraverseObstacle)
    node.stop_state_lock = module.Lock()
    node.emergency_stop = True
    node.active_action_stop_latched = False
    node.busy = False
    assert "emergency stop" in node._reserve_action_if_safe()

    class FakePoseClient:
        calls = 0

        def call_async(self, _request):
            self.calls += 1
            raise AssertionError("SetEntityPose must not be dispatched while stopped")

    node.pose_client = FakePoseClient()
    node.get_parameter = lambda _name: SimpleNamespace(value="generic_quadruped")
    assert not node._set_model_pose(1.0, 2.0, 0.0)
    assert node.pose_client.calls == 0

    source = (
        PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    ).read_text(encoding="utf-8")
    prepare = source[
        source.index("    def _prepare_entry") : source.index("    def _set_model_pose")
    ]
    execute = source[source.index("    def execute") : source.index("\ndef main")]
    assert "_software_stop_blocks_motion" in prepare
    assert execute.index("_software_stop_blocks_motion") < execute.index(
        "self._set_model_pose"
    )
    assert "interrupted stabilization" in execute


def test_sim_traverse_feedback_is_canonical_and_monotonic():
    module = _load_sim_traverse_module("sim_traverse_feedback")

    class FakeHandle:
        def __init__(self):
            self.feedback = []

        def publish_feedback(self, message):
            self.feedback.append(message)

    handle = FakeHandle()
    publisher = module.MonotonicFeedback(handle)
    publisher.publish(module.TraverseObstacle.Feedback.STATE_PREPARING, 0.0, "prepare")
    publisher.publish(module.TraverseObstacle.Feedback.STATE_PREPARING, 0.2, "align")
    publisher.publish(module.TraverseObstacle.Feedback.STATE_TRAVERSING, 0.5, "traverse")
    publisher.publish(module.TraverseObstacle.Feedback.STATE_STABILIZING, 0.8, "settle")
    publisher.publish(module.TraverseObstacle.Feedback.STATE_STABILIZING, 1.0, "done")
    assert [item.state for item in handle.feedback] == [1, 1, 2, 3, 3]
    assert [item.progress for item in handle.feedback] == sorted(
        item.progress for item in handle.feedback
    )
    with pytest.raises(RuntimeError):
        publisher.publish(module.TraverseObstacle.Feedback.STATE_TRAVERSING, 1.0, "regress")


def test_preparing_goal_finishes_low_speed_alignment_before_teleport():
    """PREPARING may not jump directly from Goal acceptance to SetEntityPose."""
    backend = (PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py").read_text(
        encoding="utf-8"
    )
    execute = backend[backend.index("    def execute(self, handle):") :]
    assert execute.index(
        "self._prepare_entry(handle, feedback, entry_snapshot)"
    ) < execute.index("STATE_TRAVERSING")
    prepare = backend[
        backend.index(
            "    def _prepare_entry(self, handle, feedback, entry_snapshot):"
        ) : backend.index("    def _set_model_pose", backend.index("    def _prepare_entry"))
    ]
    assert "preparation_linear_speed" in prepare
    assert "preparation_stall_timeout" in prepare
    assert "_set_model_pose" not in prepare


def test_benchmark_stations_follow_central_world_layout():
    """Three-minute staging must derive global coordinates from world layout frames."""
    path = PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    spec = importlib.util.spec_from_file_location("sim_traverse_obstacle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    layouts = module.load_layout_poses(WORLD_PATH)
    assert set(layouts) == set(module.BENCHMARK_TASK_ORDER)
    assert module.next_benchmark_target([]) == "right_angle_poles"
    assert module.next_benchmark_target(module.BENCHMARK_TASK_ORDER) == "__home__"
    for obstacle_id in module.BENCHMARK_TASK_ORDER:
        pose = module.benchmark_observation_pose(obstacle_id, layouts)
        assert pose is not None
        assert module.pose_inside_arena(*pose[:2], 7.0, 3.0, 0.35)


def test_benchmark_contracts_cover_every_task_without_claiming_completion():
    """Every staged obstacle supplies coherent, conservative public ROS messages."""
    path = PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    spec = importlib.util.spec_from_file_location("sim_traverse_obstacle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert set(module.BENCHMARK_CONTRACTS) == set(module.BENCHMARK_TASK_ORDER)
    for obstacle_id in module.BENCHMARK_TASK_ORDER:
        fused = module.benchmark_fused_obstacle(obstacle_id)
        assert fused.geometry_confirmed
        assert fused.vision_confirmed
        assert fused.confidence == pytest.approx(0.99)
        assert fused.valid_points >= 1000
        assert fused.distance == pytest.approx(1.0)
        assert abs(fused.structure_heading) < 1e-9


def test_sim_traversal_rejects_large_unrelated_heading_change():
    """无法沿已确认入口安全落地时应重观察，不能横穿场地伪造一次越障。"""
    path = PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    spec = importlib.util.spec_from_file_location("sim_traverse_obstacle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    landing = module.choose_safe_traversal_heading(
        -5.46, -2.63, -0.69, 7.67, 7.0, 3.0, 0.35
    )
    assert landing is None
    # 小角度即可避开边界时仍可在对正误差范围内修正。
    landing = module.choose_safe_traversal_heading(
        4.8, 0.0, 0.0, 1.5, 7.0, 3.0, 0.75, maximum_adjustment=0.35
    )
    assert landing is not None
    assert abs(landing[0]) <= 0.35


def test_sim_teleport_shortens_full_span_but_still_crosses_entry():
    """A side-on T stair may shorten only to entry distance plus completion margin."""
    path = PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    spec = importlib.util.spec_from_file_location("sim_traverse_obstacle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    landing = module.choose_safe_traversal_heading(
        2.01,
        0.77,
        1.20,
        4.43,
        7.0,
        3.0,
        0.35,
        minimum_distance=1.48,
    )
    assert landing is not None
    yaw, distance = landing
    assert abs(yaw - 1.20) < 1e-6
    assert 1.48 <= distance < 4.43
    x, y, _ = module.traversal_landing_pose(2.01, 0.77, yaw, distance)
    assert module.pose_inside_arena(x, y, 7.0, 3.0, 0.35)


def test_reference_world_clock_rate_is_bounded_for_algorithm_integration():
    """纯算法联调无需 1 kHz 物理时钟，避免每个仿真时钟节点被过度唤醒。"""
    step_size = float(WORLD.findtext("physics/max_step_size"))
    # 100 Hz 足以支撑测试 IMU，并防止 GUI/RViz 同机时关键导航数据饥饿。
    assert 0.01 <= step_size <= 0.02


def test_rule_dimensions_are_locked():
    # 高墙 1000 × 50 × 300 mm；T 台阶中心台 1000 × 1000 × 400 mm。
    assert_close(collision_box("high_wall", "wall"), [0.05, 1.0, 0.30])
    assert_close(collision_box("t_shaped_stairs", "platform"), [1.0, 1.0, 0.40])
    # V2.0 大斜坡的水平投影为 3000 mm，坡角 11.3°；SDF box
    # 长度是斜边而不是水平投影。桥 A 长条仍为 1500 × 100 mm。
    ramp = model("main_slope").find(".//collision[@name='ramp']")
    assert ramp is not None
    ramp_size = collision_box("main_slope", "ramp")
    ramp_pose = [float(value) for value in ramp.findtext("pose").split()]
    ramp_pitch = abs(ramp_pose[4])
    assert math.degrees(ramp_pitch) == pytest.approx(11.3, abs=1e-4)
    assert ramp_size[0] * math.cos(ramp_pitch) == pytest.approx(3.0, abs=2e-6)
    rise = ramp_size[0] * math.sin(ramp_pitch)
    assert rise == pytest.approx(3.0 * math.tan(math.radians(11.3)), abs=2e-6)
    assert ramp_pose[2] == pytest.approx(rise / 2.0, abs=2e-6)
    assert_close(ramp_size[1:], [2.0, 0.08])
    assert_close(collision_box("wooden_bridge_a", "beam_1"), [1.5, 0.1, 0.1])
    # 桥 B：六块 150 mm 踏板加五个 400 mm 净间隔，正好覆盖 2900 mm。
    assert_close(collision_box("wooden_bridge_b", "plank_1"), [0.15, 1.0, 0.1])
    centers = []
    for index in range(1, 7):
        pose = model("wooden_bridge_b").find(
            f".//collision[@name='plank_{index}']/pose"
        )
        assert pose is not None and pose.text
        centers.append(float(pose.text.split()[0]))
    assert_close([b - a for a, b in zip(centers, centers[1:])], [0.55] * 5)
    assert abs((centers[-1] - centers[0]) + 0.15 - 2.90) <= 1e-6
    # 两条小坡各为 14°，连接规则给出的 200 mm 高平台。
    for bridge_name, expected_pitch in (
        ("wooden_bridge_a", -0.244346),
        ("wooden_bridge_b", 0.244346),
    ):
        ramp_pose = model(bridge_name).find(
            ".//collision[@name='approach_ramp']/pose"
        )
        assert ramp_pose is not None and ramp_pose.text
        pitch = float(ramp_pose.text.split()[4])
        assert abs(pitch - expected_pitch) <= 1e-6


def test_height_bar_and_pole_geometry():
    bar = model("height_bar").find(".//collision[@name='crossbar']")
    assert bar is not None
    radius = float(bar.findtext("geometry/cylinder/radius"))
    center_z = float(bar.findtext("pose").split()[2])
    assert abs(center_z - radius - 0.30) <= 1e-6
    pole_model = model("right_angle_poles")
    poses = [
        [float(value) for value in pole_model.findtext(f".//collision[@name='pole_{i}']/pose").split()]
        for i in range(1, 4)
    ]
    assert_close([poses[1][0] - poses[0][0], poses[2][1] - poses[1][1]], [1.0, 1.0])
    # 规则参考图中的两只圆形底座也应存在，不能只画两根悬空细柱。
    height_bar = model("height_bar")
    assert height_bar.find(".//collision[@name='left_base']/geometry/cylinder") is not None
    assert height_bar.find(".//collision[@name='right_base']/geometry/cylinder") is not None


def test_pit_fill_has_physical_collision_samples():
    """砂砾和碎木不能只有贴图，否则点云/车轮永远看到平滑坑底。"""
    pit = model("gravel_wood_pit")
    for prefix in ("stone", "wood"):
        samples = pit.findall(f".//collision[@name='{prefix}_collision_1']/..")
        assert samples
        assert len(pit.findall(f".//collision[@name='{prefix}_collision_1']")) == 1


def test_published_colors_are_present_exactly():
    floor = model("competition_floor").find(".//link[@name='floor_south']/visual/material/diffuse")
    pole = model("right_angle_poles").find(".//visual[@name='visual_pole_1']/material/diffuse")
    blue = model("height_bar").find(".//visual[@name='bar_01']/material/diffuse")
    assert_close([float(x) for x in floor.text.split()], [1.0, 1.0, 0.0, 1.0])
    assert_close([float(x) for x in pole.text.split()], ORANGE)
    assert_close(
        [float(x) for x in blue.text.split()],
        [31.0 / 255.0, 65.0 / 255.0, 159.0 / 255.0, 1.0],
    )


def test_v2_reach_zones_are_red_dashed_visuals_on_yellow_floor():
    """V2.0 必达区不得再画成遮住黄地的实心红圆。"""
    pole_model = model("right_angle_poles")
    zone_centers = {
        "start": (-0.40, 0.00),
        "corner": (1.00, -0.40),
        "finish": (1.00, 1.40),
    }
    for zone, center in zone_centers.items():
        dashes = pole_model.findall(f".//visual[@name='zone_{zone}_dash_01']/..")
        assert dashes
        named = [
            item
            for item in pole_model.findall(".//visual")
            if item.attrib.get("name", "").startswith(f"zone_{zone}_dash_")
        ]
        assert len(named) == 8
        for dash in named:
            assert dash.find("geometry/box") is not None
            pose = [float(value) for value in dash.findtext("pose").split()]
            assert math.hypot(pose[0] - center[0], pose[1] - center[1]) == pytest.approx(
                0.175, abs=1e-6
            )
            assert_close(
                [float(value) for value in dash.findtext("geometry/box/size").split()],
                [0.070, 0.020, 0.006],
            )
            diffuse = [float(value) for value in dash.findtext("material/diffuse").split()]
            assert_close(diffuse, [1.0, 0.0, 0.0, 1.0])
    assert pole_model.find(".//visual[@name='zone_start']") is None
    assert not any(
        collision.attrib.get("name", "").startswith("zone_")
        for collision in pole_model.findall(".//collision")
    )


def test_v2_main_slope_simulator_truth_matches_world():
    """Gazebo-only fused truth must not retain the obsolete 10-degree baseline."""
    module = _load_sim_traverse_module("sim_traverse_v2_slope_truth")
    fused = module.benchmark_fused_obstacle("main_slope")
    assert fused is not None
    assert math.degrees(fused.slope_pitch) == pytest.approx(11.3, abs=1e-6)


def test_height_bar_simulator_truth_matches_rule_and_world_clearance():
    """Deterministic hints must preserve the published 300 mm lower clearance."""
    module = _load_sim_traverse_module("sim_traverse_height_bar_truth")
    fused = module.benchmark_fused_obstacle("height_bar")
    assert fused is not None
    crossbar = model("height_bar").find(".//collision[@name='crossbar']")
    assert crossbar is not None
    radius = float(crossbar.findtext("geometry/cylinder/radius"))
    centre_z = float(crossbar.findtext("pose").split()[2])
    world_clearance = centre_z - radius
    assert world_clearance == pytest.approx(0.30, abs=1e-6)
    assert fused.clearance_height == pytest.approx(world_clearance, abs=1e-6)


def test_obstacle_poses_are_centralized_in_layout_frames():
    """八个模型只引用集中式 frame，正式坐标不会散落在各模型内部。"""
    obstacle_names = [
        "right_angle_poles",
        "gravel_wood_pit",
        "height_bar",
        "main_slope",
        "wooden_bridge_a",
        "wooden_bridge_b",
        "t_shaped_stairs",
        "high_wall",
    ]
    world_frames = {frame.attrib["name"] for frame in WORLD.findall("frame")}
    for name in obstacle_names:
        pose = model(name).find("pose")
        assert pose is not None
        assert pose.attrib.get("relative_to") == f"layout_{name}"
        assert pose.text.strip() == "0 0 0 0 0 0"
        assert f"layout_{name}" in world_frames


def test_t_stairs_exit_does_not_land_inside_bridge_b():
    """参考布局必须允许逐障碍测试；两结构之间留出完整测试狗通道。"""
    stair_y = layout_pose("t_shaped_stairs")[1]
    bridge_y = layout_pose("wooden_bridge_b")[1]
    stair_north = stair_y + 0.50  # T 顶台/横臂的北缘。
    bridge_south = bridge_y - 0.50
    # generic_quadruped/Nav2 占位直径是 0.60 m；额外保留 0.20 m 防止定位和
    # 栅格离散误差把数学上刚好可过的间隙变成实际不可达。
    assert bridge_south - stair_north >= 0.80


def test_long_bridge_reference_layout_keeps_full_traversal_inside_arena():
    """非正式参考布局也必须能完成整桥回归，而不是在桥尾越出 14 m 场地。"""
    # Action 在距入口 1.20 m 处交接，跨结构后留 0.75 m；测试狗中心必须仍处于
    # 7.0-0.75=6.25 m 的安全内缩边界。坐标仍只从 world 的集中 frame 读取。
    spans = {"wooden_bridge_a": 4.35, "wooden_bridge_b": 5.70}
    local_west = {"wooden_bridge_a": -2.5645, "wooden_bridge_b": -2.451}
    for name, span in spans.items():
        centre_x = layout_pose(name)[0]
        entry_edge = centre_x + local_west[name]
        handoff_x = entry_edge - 1.20
        landing_x = handoff_x + 1.20 + span + 0.75
        assert landing_x <= 6.25, (name, landing_x)


def test_simulation_launch_stays_out_of_algorithm_launch():
    simulation_launch = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text()
    algorithm_launch = (PACKAGE_ROOT.parent / "slam" / "launch" / "slam.launch.py").read_text()
    assert "slam.launch.py" not in simulation_launch.replace("``slam.launch.py``", "")
    assert "navigation.launch.py" not in simulation_launch
    assert "quadruped_gazebo" not in algorithm_launch
    for interface in (
        "/scan",
        "/odom",
        "/imu/data",
        "/camera/image_raw",
        "/camera/depth/points",
        "camera_optical_frame",
    ):
        assert interface in simulation_launch
    compile(simulation_launch, "robocon_field.launch.py", "exec")


def test_sensor_carrier_matches_slam_and_perception_contracts():
    """仿真输出直接使用既有算法默认话题和可解析 TF frame。"""
    assert ROBOT is not None
    sensors = {
        sensor.attrib["type"]: sensor
        for sensor in ROBOT.findall(".//sensor")
    }
    assert sensors["gpu_lidar"].findtext("topic") == "/scan"
    assert sensors["gpu_lidar"].findtext("gz_frame_id") == "lidar_link"
    assert sensors["imu"].findtext("topic") == "/imu/data"
    assert sensors["imu"].findtext("gz_frame_id") == "imu_link"
    assert sensors["rgbd_camera"].findtext("topic") == "/camera"
    assert sensors["rgbd_camera"].findtext("gz_frame_id") == "camera_optical_frame"
    assert (
        sensors["rgbd_camera"].findtext("camera/optical_frame_id")
        == "camera_optical_frame"
    )
    motion = ROBOT.find("plugin[@name='gz::sim::systems::VelocityControl']")
    odometry = ROBOT.find("plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert motion is not None
    assert motion.findtext("topic") == "/cmd_vel"
    assert odometry is not None
    assert odometry.findtext("odom_topic") == "/odom"
    assert odometry.findtext("tf_topic") == "/tf"
    assert odometry.findtext("odom_frame") == "odom"
    assert odometry.findtext("robot_base_frame") == "base_link"
    assert odometry.findtext("dimensions") == "2"


def test_simulation_velocity_mux_autonomy_stop_keeps_manual_takeover():
    """自主进程退出只锁自主分支，持续键盘/手柄输入仍能人工接管。"""
    path = PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py"
    spec = importlib.util.spec_from_file_location("sim_cmd_vel_mux", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    autonomous = Twist()
    autonomous.linear.x = 0.3
    manual = Twist()
    manual.angular.z = 0.6
    selected = module.select_command(10.0, manual, 9.8, autonomous, 9.9, 0.7, 0.5)
    assert selected.angular.z == 0.6
    selected = module.select_command(
        10.0, manual, 9.8, autonomous, 9.9, 0.7, 0.5, autonomy_stop=True
    )
    assert selected.angular.z == 0.6
    selected = module.select_command(
        11.0, manual, 9.8, autonomous, 10.9, 0.7, 0.5, autonomy_stop=True
    )
    assert selected.linear.x == 0.0 and selected.angular.z == 0.0
    selected = module.select_command(10.6, manual, 9.8, autonomous, 10.4, 0.7, 0.5)
    assert selected.linear.x == 0.3
    selected = module.select_command(12.0, manual, 9.8, autonomous, 10.4, 0.7, 0.5)
    assert selected.linear.x == 0.0 and selected.angular.z == 0.0

    # Xbox B 的软件停车由最终仿真仲裁器消费，必须覆盖仍新鲜的人工与自主候选。
    selected = module.select_command(
        10.0,
        manual,
        9.8,
        autonomous,
        9.9,
        0.7,
        0.5,
        emergency_stop=True,
    )
    assert selected.linear.x == 0.0 and selected.angular.z == 0.0


def test_simulation_velocity_mux_emergency_stop_clears_cached_commands():
    """解除 B 键锁存后不得复用急停前仍新鲜的任一非零 Twist。"""
    path = PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py"
    spec = importlib.util.spec_from_file_location("sim_cmd_vel_mux_estop", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    published = []
    node = module.SimCmdVelMux.__new__(module.SimCmdVelMux)
    node.manual = Twist()
    node.manual.linear.x = 0.2
    node.joystick = Twist()
    node.joystick.angular.z = 0.4
    node.autonomous = Twist()
    node.autonomous.linear.x = 0.3
    node.manual_stamp = 1.0
    node.joystick_stamp = 1.1
    node.autonomous_stamp = 1.2
    node.publisher = type(
        "Publisher", (), {"publish": lambda _self, message: published.append(message)}
    )()

    node._emergency_stop_callback(Bool(data=True))
    assert node.emergency_stop
    assert node.manual_stamp is None
    assert node.joystick_stamp is None
    assert node.autonomous_stamp is None
    assert published and published[-1].linear.x == 0.0

    node._emergency_stop_callback(Bool(data=False))
    assert not node.emergency_stop
    assert node.manual_stamp is None
    assert node.joystick_stamp is None
    assert node.autonomous_stamp is None


def test_simulation_velocity_mux_handles_only_shutdown_runtime_error():
    """The launch-wide SIGINT DDS teardown race must not print a false crash."""
    source = (PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py").read_text(
        encoding="utf-8"
    )
    assert "except RuntimeError:" in source
    assert "if rclpy.ok():" in source


def test_field_launch_routes_one_arbitrated_velocity_to_gazebo():
    """Gazebo bridge 只能接收 mux 输出，避免人工和导航速度源相互覆盖。"""
    launch_source = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="sim_cmd_vel_mux"' in launch_source
    assert '("/cmd_vel", "/cmd_vel_gazebo")' in launch_source
    assert "/cmd_vel_teleop" in (
        PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py"
    ).read_text(encoding="utf-8")
    assert "/cmd_vel_joy" in (
        PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py"
    ).read_text(encoding="utf-8")
    assert "/teleop/active" in (
        PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py"
    ).read_text(encoding="utf-8")
    assert "/teleop/emergency_stop" in (
        PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py"
    ).read_text(encoding="utf-8")
    mux_source = (PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py").read_text(
        encoding="utf-8"
    )
    assert "self.autonomous_stamp = None" in mux_source
    assert "self.publisher.publish(Twist())" in mux_source
    # 测试狗没有腿部动力学，第三条自主任务中的仿真 Action 只可通过这一标准
    # Gazebo 服务跨越实体碰撞；场地 launch 本身仍不启动任何越障节点。
    assert 'world_name = LaunchConfiguration("world_name")' in launch_source
    assert '"/set_pose@ros_gz_interfaces/srv/SetEntityPose"' in launch_source
    assert "ros_gz_interfaces/srv/SetEntityPose" in launch_source


def test_simulated_teleport_landing_is_layout_independent_and_aligned():
    """仿真落点只使用实时起点/航向，不得硬编码八个 world 坐标。"""
    path = PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    spec = importlib.util.spec_from_file_location("sim_traverse_obstacle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    x, y, yaw = module.traversal_landing_pose(1.0, 2.0, 0.0, 3.0)
    assert_close([x, y, yaw], [4.0, 2.0, 0.0])
    # 普通结构只有出口落点；源码不能重新出现 progress 驱动的中间位姿。
    assert "progress" not in module.traversal_landing_pose.__code__.co_varnames
    # L 形障碍的传送终点位于第二臂出口，最终航向沿第二条臂。
    l_finish = module.traversal_landing_pose(0.0, 0.0, 0.0, 5.0, l_turn=-1)
    assert_close(list(l_finish), [2.4, -2.6, -math.pi / 2.0], tolerance=1e-5)
    safe_l = module.choose_safe_l_traversal(
        -5.85, -0.57, math.pi / 2.0, 4.33, 7.0, 3.0, 0.75
    )
    assert safe_l is not None
    assert safe_l[1] == -1  # 北向进入后向东（机体右侧）离开参考 L 形坑。
    assert safe_l[2] > 0.0
    assert module.pose_inside_arena(0.0, 0.0, 7.0, 3.0, 0.35)
    assert module.pose_inside_arena(6.64, 2.64, 7.0, 3.0, 0.35)
    assert not module.pose_inside_arena(6.66, 0.0, 7.0, 3.0, 0.35)
    assert not module.pose_inside_arena(0.0, -2.66, 7.0, 3.0, 0.35)
    source = path.read_text(encoding="utf-8")
    # The Action landing itself remains layout-independent.  The optional, later
    # three-minute staging helper is allowed to read centralized world frames, but
    # it cannot influence the pose used to verify the current traversal.
    execute_source = source[source.index("    def execute(self, handle):"):]
    execute_source = execute_source[:execute_source.index("\ndef main(args=None):")]
    assert "layout_" not in execute_source
    assert "robocon_obstacle_field.sdf" not in source
    assert "handle.request.distance" in source
    # 简化越障的落点必须同时越过机身半长和 Nav2 inflation layer，不能把下一次
    # 规划的起点留在障碍物致命代价区内。
    assert '"exit_clearance", 1.20' in source
    assert '"right_angle_poles_span", 1.00' in source
    # Competition pole semantics must use the same bounded right-angle path
    # selector as the L-shaped pit, not the legacy centreline S curve.
    assert '"right_angle_poles", "gravel_wood_pit"' in source
    assert '"t_shaped_stairs_span", 2.80' in source
    assert '"wooden_bridge_b_span", 5.20' in source
    assert "+ semantic_span" in source


def test_simulated_traversal_pose_service_follows_world_name():
    """独立 Action 替身也使用公开 world_name，而不是隐藏的默认服务路径。"""
    path = PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    spec = importlib.util.spec_from_file_location("sim_traverse_world", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.gazebo_pose_service("field_v2") == "/world/field_v2/set_pose"
    assert module.gazebo_pose_service("ignored", "/custom/set_pose") == (
        "/custom/set_pose"
    )
    with pytest.raises(ValueError):
        module.gazebo_pose_service("../field")


def test_teleport_field_launch_owns_backend_without_algorithms():
    """组合入口属于 Gazebo，只组合场地和传送服务，绝不加载核心算法。"""
    source = (
        PACKAGE_ROOT / "launch" / "robocon_field_teleport.launch.py"
    ).read_text(encoding="utf-8")
    assert "robocon_field.launch.py" in source
    assert 'package="quadruped_gazebo"' in source
    assert 'executable="sim_traverse_obstacle"' in source
    assert '"world_name": world_name' in source
    for forbidden in ("slam.launch.py", "autonomous_mission", "Nav2"):
        # Nav2 may be mentioned in the module documentation only; executable launch
        # content must not reference a package or node from the algorithm stack.
        if forbidden == "Nav2":
            continue
        assert forbidden not in source.replace("``robocon_field.launch.py``", "")
    assert 'package="slam"' not in source
    compile(source, "robocon_field_teleport.launch.py", "exec")


def test_gui_field_opens_remapped_keyboard_without_loading_algorithms():
    """第一条 GUI 命令应提供人工测试窗口，但不能借机耦合 SLAM 或自主任务。"""
    launch_source = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'package="teleop_twist_keyboard"' in launch_source
    assert '("cmd_vel", "/cmd_vel_teleop")' in launch_source
    assert '"keyboard_teleop"' in launch_source
    assert "gnome-terminal --wait" in launch_source
    assert "repeat_rate" not in launch_source
    assert "key_timeout" not in launch_source


def test_field_launch_rejects_a_duplicate_named_gazebo_world():
    """重复同名服务会混接机器人/传感器，入口必须在启动前显式拒绝。"""
    launch_source = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text(
        encoding="utf-8"
    )
    assert "def _reject_duplicate_world" in launch_source
    assert 'scene_service = f"/world/{world_name}/scene/info"' in launch_source
    assert "def _validated_world_name" in launch_source
    assert "OpaqueFunction(function=_reject_duplicate_world)" in launch_source


def test_world_name_is_a_single_safe_gazebo_service_segment():
    """自定义 world 文件只能把受校验的名称拼入 Gazebo 服务路径。"""
    path = PACKAGE_ROOT / "launch" / "robocon_field.launch.py"
    spec = importlib.util.spec_from_file_location("robocon_field_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module._validated_world_name("field_v2") == "field_v2"
    for invalid in ("", "two worlds", "../field", "/field"):
        with pytest.raises(RuntimeError):
            module._validated_world_name(invalid)


def test_generic_rgbd_resolution_is_bounded_for_realtime_integration():
    """测试相机保留可用细节，但不能以无意义的高像素拖慢整套联调。"""
    camera = ROBOT.find(".//sensor[@name='rgbd']/camera")
    assert camera is not None
    image = camera.find("image")
    assert image is not None
    width = int(image.findtext("width"))
    height = int(image.findtext("height"))
    assert width >= 320 and height >= 180
    assert width * height <= 150_000
    # RGBD 不使用可见掩码：Gazebo Harmonic 的部分渲染后端会因此输出全 Inf 深度。
    assert camera.find("visibility_mask") is None
    camera_link = ROBOT.find("link[@name='camera_link']")
    assert camera_link is not None
    camera_pose = [float(value) for value in camera_link.findtext("pose").split()]
    camera_visual_pose = [
        float(value) for value in camera_link.findtext("visual/pose").split()
    ]
    # 光心要在机头外，外观必须位于光心后方，防止相机看到自身外壳。
    assert camera_pose[0] > 0.45
    assert camera_visual_pose[0] < 0.0


def test_generic_quadruped_is_planar_and_has_no_fake_leg_controller():
    """测试替身必须保持雷达水平，且不能伪装成真正的关节/步态控制。"""
    assert ROBOT.attrib["name"] == "generic_quadruped"
    base = ROBOT.find("link[@name='base_link']")
    assert base is not None
    assert base.findtext("gravity") == "false"
    assert base.find("collision") is None
    assert not any("wheel" in link.attrib["name"] for link in ROBOT.findall("link"))
    assert ROBOT.find("plugin[@name='gz::sim::systems::JointController']") is None

    lidar = ROBOT.find("link[@name='lidar_link']")
    assert lidar is not None
    lidar_pose = [float(value) for value in lidar.findtext("pose").split()]
    assert_close(lidar_pose[3:], [0.0, 0.0, 0.0])
    scan = lidar.find("sensor/lidar/scan/horizontal")
    assert scan is not None
    # 纯 SLAM 测试替身使用 360° 雷达，避免有限视场把扇形未知区误看成“地图乱线”。
    assert float(scan.findtext("max_angle")) - float(scan.findtext("min_angle")) >= 6.28
    assert lidar.findtext("sensor/lidar/visibility_mask") == "0x01"
    # 机械狗外观使用另一可见位；激光不得把机身和腿扫入地图。
    assert all(
        visual.findtext("visibility_flags") == "0x02"
        for visual in ROBOT.findall(".//visual")
    )


def test_launch_exposes_one_step_robot_replacement_contract():
    """真实 SDF 到位后只换 launch 参数，不允许改 SLAM/Nav2/OpenCV。"""
    source = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text()
    for argument in ("world_name", "robot_sdf", "robot_name", "publish_test_sensor_tf"):
        assert "DeclareLaunchArgument" in source
        assert f'"{argument}"' in source
    assert 'models" / "generic_quadruped"' in source


def test_gazebo_manifest_declares_python_script_message_dependencies():
    """仿真 Python 节点的直接消息依赖不能只由其他包传递安装。"""
    manifest = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>std_msgs</exec_depend>" in manifest


def test_rgbd_point_cloud_bridge_corrects_gazebo_numeric_frame():
    """Gazebo 点云的 x/y/z 数值轴必须与覆盖后的 camera_link Header 一致。"""
    launch_source = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text()
    assert 'name="robocon_point_cloud_bridge"' in launch_source
    assert '{"override_frame_id": "camera_link"}' in launch_source
    assert '("/camera/points", "/camera/depth/points")' in launch_source


def test_navigation_bridge_is_not_blocked_by_unused_high_bandwidth_cloud():
    """时钟和导航关键桥要与辅助传感器隔离，且不得重复桥接激光点云。"""
    source = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text()
    assert 'name="robocon_clock_bridge"' in source
    assert 'name="robocon_navigation_bridge"' in source
    assert 'name="robocon_aux_sensor_bridge"' in source
    assert '"/scan/points@' not in source

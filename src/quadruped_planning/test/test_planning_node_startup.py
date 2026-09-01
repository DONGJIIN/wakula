"""ROS node-construction and watchdog tests for planning parameter contracts."""

import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from quadruped_interfaces.msg import NavigationSafety, TraversalGuidance
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32

import quadruped_planning.autonomous_mission as mission_module
from quadruped_planning.autonomous_mission import AutonomousMission, ObservedObstacle
from quadruped_planning.cmd_vel_gate import NavigationSpeedGate
from quadruped_planning.terrain_safety_assessor import TerrainSafetyAssessor
from quadruped_planning.traversal_guidance import TraversalGuidanceNode


class PendingFuture:
    """Deterministic stand-in for an rclpy Future whose response can be delayed."""

    def __init__(self):
        self.callback = None
        self.value = None
        self.exception = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        if self.exception is not None:
            raise self.exception
        return self.value

    def complete(self, value):
        self.value = value
        self.callback(self)

    def fail(self, exception):
        self.exception = exception
        self.callback(self)


class FakeActionClient:
    def __init__(self, ready, future=None, exception=None):
        self.ready = ready
        self.future = future or PendingFuture()
        self.exception = exception
        self.sent = 0
        self.last_goal = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.sent += 1
        self.last_goal = goal
        if self.exception is not None:
            raise self.exception
        return self.future


class FakeGoalHandle:
    accepted = True

    def __init__(self, cancel_future=None, result_future=None):
        self.cancel_future = cancel_future or PendingFuture()
        self.result_future = result_future or PendingFuture()
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


class RecordingPublisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(bool(message.data))


def valid_wall_guidance():
    """Build one internally consistent high-wall handoff snapshot."""
    guidance = TraversalGuidance()
    guidance.phase = guidance.PHASE_READY
    guidance.obstacle_type = guidance.OBSTACLE_WALL
    guidance.perception_valid = True
    guidance.traversal_required = True
    guidance.ready_for_handoff = True
    guidance.confidence = 0.9
    guidance.distance = 1.0
    guidance.speed_limit = 0.0
    return guidance


def valid_wall_safety():
    """Build point-cloud geometry satisfying the final high-wall contract."""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = safety.OBSTACLE_WALL
    safety.confidence = 0.9
    safety.obstacle_height = 0.30
    safety.width = 1.0
    return safety


def install_fresh_wall_handoff(mission, guidance=None):
    """Populate only the runtime inputs required by the final Action gate."""
    now = time.monotonic()
    guidance = guidance or valid_wall_guidance()
    mission.enabled = True
    mission._robot_pose = lambda: (0.0, 0.0, 0.0)
    mission.guidance = guidance
    mission.guidance_received = now
    mission.last_safety = valid_wall_safety()
    mission.safety_received = now
    mission.pending_traverse = guidance
    mission.pending_traverse_id = "high_wall"
    mission.pending_traverse_position = (1.0, 0.0)
    mission.pending_traverse_robot_start = (0.0, 0.0)
    mission.pending_traverse_started = now
    return guidance


@pytest.fixture
def ros_context():
    """Create and release one ROS context per test to avoid DDS state leakage."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_all_planning_nodes_accept_the_shipped_default_contract(ros_context):
    """Instantiate the real nodes so defaults, ROS types, timers, and Action clients stay valid."""
    nodes = []
    try:
        nodes.extend(
            (
                TerrainSafetyAssessor(),
                TraversalGuidanceNode(),
                NavigationSpeedGate(),
                AutonomousMission(),
            )
        )
        assert {node.get_name() for node in nodes} == {
            "terrain_safety_assessor",
            "traversal_guidance",
            "navigation_speed_gate",
            "autonomous_mission",
        }
    finally:
        for node in reversed(nodes):
            node.destroy_node()


@pytest.mark.parametrize(
    ("node_factory", "overrides", "message"),
    (
        (
            TerrainSafetyAssessor,
            [
                Parameter("step_threshold", value=0.3),
                Parameter("climb_threshold", value=0.2),
            ],
            "step < climb < stop",
        ),
        (
            TraversalGuidanceNode,
            [
                Parameter("approach_start_distance", value=0.8),
                Parameter("handoff_distance", value=1.2),
            ],
            "approach_start_distance",
        ),
        (
            NavigationSpeedGate,
            [Parameter("input_topic", value="/cmd_vel")],
            "velocity feedback loop",
        ),
        (
            AutonomousMission,
            [
                Parameter("semantic_confirmation_votes", value=6),
                Parameter("semantic_recent_window", value=5),
            ],
            "semantic_confirmation_votes",
        ),
    ),
)
def test_invalid_planning_overrides_fail_before_runtime(
    ros_context, node_factory, overrides, message
):
    """Reject unsafe threshold, handoff, velocity-loop, and vote configurations."""
    with pytest.raises(ValueError, match=message):
        node_factory(parameter_overrides=overrides)


def _spin_until(executor, predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def test_speed_gate_real_timer_stops_after_command_timeout(ros_context):
    """The online timer must overwrite a stale non-zero command with an explicit zero Twist."""
    gate = NavigationSpeedGate(
        parameter_overrides=[
            Parameter("command_timeout", value=0.12),
            Parameter("assessment_timeout", value=0.5),
            Parameter("navigation_health_timeout", value=0.5),
            Parameter("require_emergency_scan", value=False),
        ]
    )
    driver = Node("speed_gate_watchdog_test_driver")
    command_pub = driver.create_publisher(Twist, "/cmd_vel_smoothed", 10)
    limit_pub = driver.create_publisher(Float32, "/terrain/speed_limit", 10)
    health_pub = driver.create_publisher(Bool, "/navigation/healthy", 10)
    outputs = []
    driver.create_subscription(Twist, "/cmd_vel", outputs.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(driver)
    try:
        assert _spin_until(executor, lambda: command_pub.get_subscription_count() > 0, 0.8)
        command = Twist()
        command.linear.x = 0.4
        command_pub.publish(command)
        limit_pub.publish(Float32(data=0.5))
        health_pub.publish(Bool(data=True))
        assert _spin_until(
            executor,
            lambda: any(abs(message.linear.x - 0.2) < 1e-5 for message in outputs),
            0.8,
        )
        # Stop publishing only the command.  Assessment and health remain within their longer
        # windows, so the observed zero specifically verifies command_timeout rather than a
        # coincidental failure of another gate input.
        output_count = len(outputs)
        assert _spin_until(
            executor,
            lambda: len(outputs) > output_count and abs(outputs[-1].linear.x) < 1e-9,
            0.5,
        )
    finally:
        executor.remove_node(driver)
        executor.remove_node(gate)
        driver.destroy_node()
        gate.destroy_node()
        executor.shutdown()


def test_mission_runtime_uses_five_second_recovery_defaults(ros_context):
    """Construct the production node and protect the requested non-blocking policy."""
    mission = AutonomousMission()
    try:
        assert mission.params["nav_stall_timeout"] == 5.0
        assert mission.params["nav_progress_translation"] == 0.04
        assert mission.params["nav_progress_rotation"] == 0.06
        assert mission.params["controller_wait_timeout"] == 5.0
        assert mission.params["action_response_timeout"] == 2.0
        assert mission.params["action_cancel_timeout"] == 2.0
        assert mission.params["safety_geometry_stale_seconds"] == 0.35
        assert mission.params["approach_stall_handoff_count"] == 1
        assert mission.params["maximum_search_turns"] == 8
    finally:
        mission.destroy_node()
    with pytest.raises(ValueError, match="safety_geometry_stale_seconds"):
        AutonomousMission(
            parameter_overrides=[
                Parameter("safety_geometry_stale_seconds", value=0.0),
            ]
        )


def test_nav_send_watchdog_locks_speed_and_late_response_cannot_restore_goal(
    ros_context,
):
    """悬空 send response 有界进入故障；迟到 accepted handle 只能被取消。"""
    mission = AutonomousMission()
    pending = PendingFuture()
    client = FakeActionClient(False, pending)
    mission.nav_client = client
    try:
        pose = mission._make_pose(1.0, 0.0, 0.0)
        generation = mission.nav_generation
        # server 尚未 ready 不得声称提交，也不得消费/改写当前导航上下文。
        assert not mission._send_nav_goal(pose, "search_turn")
        assert mission.nav_generation == generation
        assert not mission.nav_send_pending
        assert mission.nav_purpose == ""

        client.ready = True
        assert mission._send_nav_goal(pose, "search_turn")
        assert mission.nav_send_pending
        old_generation = mission.nav_generation
        assert mission._check_action_watchdogs(
            mission.nav_send_started
            + float(mission.params["action_response_timeout"])
        )
        assert mission.action_ownership_fault
        assert mission.state == "ACTION_COMMUNICATION_FAULT"
        assert not mission.nav_send_pending
        assert mission.nav_generation > old_generation
        assert mission.completed_semantics == []

        late_handle = FakeGoalHandle()
        pending.complete(late_handle)
        assert late_handle.cancel_calls == 1
        assert mission.nav_handle is None
        assert mission.action_ownership_fault
    finally:
        mission.destroy_node()


def test_shutdown_marks_disabled_before_pending_nav_response_and_cancels_late_handle(
    ros_context,
):
    """Ctrl-C during send must cancel the handle that appears during the drain loop."""
    mission = AutonomousMission()
    pending = PendingFuture()
    mission.nav_client = FakeActionClient(True, pending)
    try:
        assert mission._send_nav_goal(
            mission._make_pose(1.0, 0.0, 0.0), "frontier"
        )
        # Mirror the ordering in ``main.finally``.  No handle exists at this point, so
        # the initial cancellation is expected to be a no-op.
        mission.enabled = False
        mission._publish_immediate_stop()
        assert not mission._cancel_nav("shutdown")

        late_handle = FakeGoalHandle()
        pending.complete(late_handle)
        assert mission.nav_handle is late_handle
        assert late_handle.cancel_calls == 1
        assert mission.nav_cancel_pending
        assert late_handle.result_future.callback is not None
    finally:
        mission.destroy_node()


def test_pending_nav_response_cannot_freeze_a_traverse_handoff(ros_context):
    """HANDOFF waits until a Nav request has an observable handle or terminal result."""
    mission = AutonomousMission()
    pending = PendingFuture()
    mission.nav_client = FakeActionClient(True, pending)
    mission.traverse_client = FakeActionClient(True)
    mission.enabled = True
    try:
        assert mission._send_nav_goal(
            mission._make_pose(2.0, 0.0, 0.0), "frontier"
        )
        guidance = valid_wall_guidance()
        assert not mission._queue_traversal_handoff(
            guidance,
            "high_wall",
            (1.0, 0.0),
            time.monotonic(),
        )
        assert mission.pending_traverse is None
        assert mission.state != "HANDOFF"

        # A normal late acceptance remains fully monitored.  A later fresh mission
        # tick may cancel this handle; stale handoff data can never start Traverse.
        late_handle = FakeGoalHandle()
        pending.complete(late_handle)
        assert mission.nav_handle is late_handle
        assert late_handle.cancel_calls == 0
        assert late_handle.result_future.callback is not None
        assert mission.pending_traverse is None
    finally:
        mission.destroy_node()


def test_cancel_result_watchdog_is_bounded_and_never_completes_a_task(ros_context):
    """cancel response/result 都不返回时必须锁速，不能永久保留活动 handle。"""
    mission = AutonomousMission()
    handle = FakeGoalHandle()
    try:
        mission.nav_generation = 3
        mission.nav_handle = handle
        mission.nav_purpose = "frontier"
        mission.nav_target = (1.0, 0.0)
        assert mission._cancel_nav("stall")
        assert mission.nav_cancel_pending
        assert mission._check_action_watchdogs(
            mission.nav_cancel_started
            + float(mission.params["action_cancel_timeout"])
        )
        assert mission.action_ownership_fault
        assert mission.nav_handle is None
        assert mission.completed_semantics == []
    finally:
        mission.destroy_node()


def test_terminal_result_invalidates_late_cancel_transport_error(ros_context):
    """Result proves release; a later cancel-service exception must not latch a fault."""
    mission = AutonomousMission()
    mission.enabled = False
    try:
        nav_handle = FakeGoalHandle()
        mission.nav_generation = 4
        mission.nav_handle = nav_handle
        mission.nav_purpose = "frontier"
        assert mission._cancel_nav("shutdown")
        nav_result = PendingFuture()
        nav_result.value = type(
            "Wrapped", (), {"status": GoalStatus.STATUS_CANCELED}
        )()
        mission._nav_result(nav_result, 4)
        assert not mission.nav_cancel_pending
        nav_handle.cancel_future.fail(RuntimeError("late Nav2 cancel error"))
        assert not mission.action_ownership_fault

        traversal = install_fresh_wall_handoff(mission)
        mission.enabled = False
        traverse_handle = FakeGoalHandle()
        mission.traverse_generation = 7
        mission.traverse_handle = traverse_handle
        assert mission._cancel_traverse("shutdown")
        traverse_result = PendingFuture()
        traverse_result.value = type(
            "Wrapped",
            (),
            {
                "status": GoalStatus.STATUS_CANCELED,
                "result": type(
                    "Result", (), {"success": False, "message": "cancelled"}
                )(),
            },
        )()
        mission._traverse_result(traverse_result, 7)
        assert not mission.traverse_cancel_pending
        traverse_handle.cancel_future.fail(
            RuntimeError("late traversal cancel error")
        )
        assert not mission.action_ownership_fault
        assert traversal is not None
    finally:
        mission.destroy_node()


def test_live_cancel_transport_error_still_latches_ownership_fault(ros_context):
    """The stale-callback guard must not hide failure while a handle remains active."""
    mission = AutonomousMission()
    handle = FakeGoalHandle()
    mission.nav_handle = handle
    mission.nav_generation = 2
    try:
        assert mission._cancel_nav("stall")
        handle.cancel_future.fail(RuntimeError("DDS cancel failure"))
        assert mission.action_ownership_fault
        assert mission.state == "ACTION_COMMUNICATION_FAULT"
    finally:
        mission.destroy_node()


def test_cancel_locks_autonomous_speed_until_matching_result(ros_context):
    """取消握手期间保持 stop=true，只有最终 result 才允许下一次自主运动。"""
    mission = AutonomousMission()
    publisher = RecordingPublisher()
    mission.autonomy_stop_pub = publisher
    mission.nav_generation = 4
    mission.nav_handle = FakeGoalHandle()
    mission.enabled = True
    mission.nav_purpose = "frontier"
    mission.nav_target = None
    try:
        assert mission._cancel_nav("replace")
        assert publisher.values[-1] is True
        wrapped = type("Wrapped", (), {"status": GoalStatus.STATUS_CANCELED})()
        result_future = PendingFuture()
        result_future.value = wrapped
        mission._nav_result(result_future, 4)
        assert publisher.values[-2:] == [True, False]
        assert mission.nav_handle is None
        assert not mission.nav_cancel_pending
    finally:
        mission.destroy_node()


def test_traverse_send_exception_is_caught_and_cannot_mark_obstacle_complete(
    ros_context,
):
    """同步发送异常也可能处在未知投递区间，必须安全锁存而不是逃出回调。"""
    mission = AutonomousMission()
    mission.traverse_client = FakeActionClient(
        True, exception=RuntimeError("DDS writer failed")
    )
    guidance = install_fresh_wall_handoff(mission)
    try:
        assert not mission._start_traverse(guidance)
        assert mission.action_ownership_fault
        assert mission.completed_semantics == []
        assert mission.pending_traverse is None
    finally:
        mission.destroy_node()


def test_traverse_final_gate_rejects_stale_or_mismatched_live_inputs(ros_context):
    """Queued history cannot authorize Action after Guidance or point-cloud changes."""
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        mission.guidance_received = (
            time.monotonic() - float(mission.params["guidance_timeout"]) - 0.01
        )
        assert not mission._start_traverse(guidance)
        assert client.sent == 0
        assert mission.pending_traverse is None
        assert mission.state == "EXPLORING"

        guidance = install_fresh_wall_handoff(mission)
        mission.safety_received = (
            time.monotonic()
            - float(mission.params["safety_geometry_stale_seconds"])
            - 0.01
        )
        assert not mission._start_traverse(guidance)
        assert client.sent == 0
        assert mission.pending_traverse is None

        guidance = install_fresh_wall_handoff(mission)
        mission.guidance.distance = 2.0
        # The current wall now projects one metre away from the frozen pending entry,
        # beyond the configured spatial identity tolerance.
        assert not mission._start_traverse(guidance)
        assert client.sent == 0
        assert mission.pending_traverse is None
    finally:
        mission.destroy_node()


def test_traverse_final_gate_sends_only_the_fresh_revalidated_snapshot(ros_context):
    """A matching live Guidance/Safety pair is copied into exactly one Action request."""
    mission = AutonomousMission()
    pending = PendingFuture()
    client = FakeActionClient(True, pending)
    mission.traverse_client = client
    try:
        queued = install_fresh_wall_handoff(mission)
        live = valid_wall_guidance()
        live.distance = 1.1
        mission.guidance = live
        mission.guidance_received = time.monotonic()
        assert mission._start_traverse(queued)
        assert client.sent == 1
        assert mission.pending_traverse is live
        assert client.last_goal.distance == pytest.approx(1.1)
        assert client.last_goal.distance != pytest.approx(queued.distance)
    finally:
        mission.destroy_node()


def test_nav_server_not_ready_does_not_consume_search_recovery_or_revisit(
    ros_context, monkeypatch
):
    """未发送的补扫、恢复和回访必须在 Nav2 恢复后仍可执行。"""
    mission = AutonomousMission()
    mission.nav_client = FakeActionClient(False)
    grid = OccupancyGrid()
    grid.info.width = 20
    grid.info.height = 20
    grid.info.resolution = 0.2
    grid.info.origin.position.x = -2.0
    grid.info.origin.position.y = -2.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 400
    now = time.monotonic()
    mission.enabled = True
    mission.map_msg = grid
    mission.map_received = now
    mission.home_pose = (0.0, 0.0, 0.0)
    mission.mission_started = now
    mission.mission_ready_after = 0.0
    mission._robot_pose = lambda: (0.0, 0.0, 0.0)
    monkeypatch.setattr(mission_module, "extract_frontiers", lambda *args, **kwargs: [])
    monkeypatch.setattr(
        mission_module, "extract_coverage_goals", lambda *args, **kwargs: []
    )
    try:
        mission.failed_entry_turn_pending = 0.7
        mission._tick()
        assert mission.failed_entry_turn_pending == 0.7
        assert not mission.nav_send_pending

        mission.failed_entry_turn_pending = 0.0
        mission.empty_frontier_count = int(
            mission.params["empty_frontier_confirmations"]
        ) - 1
        mission.search_turn_index = 0
        mission._tick()
        assert mission.search_turn_index == 0
        assert mission.empty_frontier_count >= int(
            mission.params["empty_frontier_confirmations"]
        )
        assert not mission.exploration_exhausted

        record = ObservedObstacle(
            "high_wall", 1.0, 0.0, 0.0, 0.0, 0.0, 0.9, now
        )
        mission.observed_obstacles = {"high_wall": record}
        mission._tick()
        assert record.retry_after == 0.0
        assert not mission.nav_send_pending
    finally:
        mission.destroy_node()


def test_missing_traversal_controller_keeps_task_pending_and_changes_action(ros_context):
    """A controller timeout must clear HANDOFF, cool the entry, and resume recovery."""
    mission = AutonomousMission()
    try:
        mission.pending_traverse = object()
        mission.pending_traverse_id = "high_wall"
        mission.pending_traverse_position = (1.0, 2.0)
        mission.pending_traverse_robot_start = (0.0, 0.0)
        mission.controller_wait_reported = True
        mission._abandon_controller_wait()
        assert mission.pending_traverse is None
        assert mission.pending_traverse_id == ""
        assert not mission.controller_wait_reported
        assert mission.state == "RECOVERY"
        assert mission.blocked_obstacles
        assert mission.cooldown_until > 0.0
    finally:
        mission.destroy_node()

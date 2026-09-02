"""ROS node-construction and watchdog tests for planning parameter contracts."""

from copy import deepcopy
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from quadruped_interfaces.action import TraverseObstacle
from quadruped_interfaces.msg import (
    AutonomyLease,
    NavigationSafety,
    TerrainFeatures,
    TraversalGuidance,
)
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32

import quadruped_planning.autonomous_mission as mission_module
from quadruped_planning.autonomous_mission import (
    AutonomousMission,
    Frontier,
    ObservedObstacle,
    traversal_feedback_transition_is_valid,
)
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
        self.feedback_callback = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent += 1
        self.last_goal = goal
        self.feedback_callback = feedback_callback
        if self.exception is not None:
            raise self.exception
        return self.future

    def publish_feedback(self, feedback):
        if self.feedback_callback is not None:
            self.feedback_callback(SimpleNamespace(feedback=feedback))


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


class BrokenResultHandle(FakeGoalHandle):
    """Accepted goal whose result ownership cannot be monitored."""

    def get_result_async(self):
        raise RuntimeError("result future unavailable")


class RecordingPublisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        value = (
            message.data
            if hasattr(message, "data")
            else message.active
        )
        self.values.append(bool(value))


class MessageRecordingPublisher:
    """Record arbitrary ROS messages without coercing their payload type."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class LeaseRecordingPublisher:
    """Record the complete atomic autonomy contract instead of only ``active``."""

    def __init__(self):
        self.samples = []

    def publish(self, message):
        self.samples.append(
            (
                str(message.session_id),
                int(message.sequence),
                bool(message.active),
                bool(message.motion_allowed),
            )
        )


def autonomy_lease(
    session_id: str,
    sequence: int,
    active: bool,
    motion_allowed: bool = False,
) -> AutonomyLease:
    """Build one explicit lease sample for ownership-ordering tests."""
    message = AutonomyLease()
    message.session_id = session_id
    message.sequence = sequence
    message.active = active
    message.motion_allowed = motion_allowed
    return message


def valid_wall_guidance():
    """Build one internally consistent high-wall handoff snapshot."""
    guidance = TraversalGuidance()
    guidance.phase = guidance.PHASE_READY
    guidance.obstacle_type = guidance.OBSTACLE_WALL
    guidance.semantic_id = "high_wall"
    guidance.header.frame_id = "base_link"
    guidance.header.stamp.sec = 10
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
    safety.mode = safety.MODE_STOP
    safety.obstacle_type = safety.OBSTACLE_WALL
    safety.semantic_id = "high_wall"
    safety.header.frame_id = "base_link"
    safety.header.stamp.sec = 10
    safety.confidence = 0.9
    safety.distance = 1.0
    safety.lateral_offset = 0.0
    safety.obstacle_height = 0.30
    safety.pit_depth = 0.01
    safety.slope_pitch = 0.02
    safety.slope_roll = -0.01
    safety.roughness = 0.03
    safety.width = 1.0
    safety.structure_heading = 0.04
    safety.structure_heading_confidence = 0.85
    safety.clearance_height = 0.12
    safety.valid_points = 120
    return safety


def install_fresh_wall_handoff(mission, guidance=None):
    """Populate only the runtime inputs required by the final Action gate."""
    now = time.monotonic()
    guidance = guidance or valid_wall_guidance()
    mission.enabled = True
    mission._robot_pose = lambda *_args: (0.0, 0.0, 0.0)
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


def install_stable_navigation_health(mission):
    """Satisfy the production health lease for tests focused on another boundary."""
    now = time.monotonic()
    mission.navigation_healthy = True
    mission.navigation_health_received = now
    mission.navigation_health_true_since = (
        now - float(mission.params["navigation_health_recovery_duration"]) - 0.1
    )


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
            TerrainSafetyAssessor,
            [Parameter("output_frame", value="")],
            "output_frame",
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
            TraversalGuidanceNode,
            [Parameter("type_confirmation_frames", value=0)],
            "type_confirmation_frames",
        ),
        (
            NavigationSpeedGate,
            [Parameter("input_topic", value="/cmd_vel")],
            "velocity feedback loop",
        ),
        (
            NavigationSpeedGate,
            [Parameter("autonomy_lease_timeout", value=0.0)],
            "autonomy_lease_timeout",
        ),
        (
            NavigationSpeedGate,
            [Parameter("emergency_scan_min_valid_ratio", value=0.0)],
            "emergency_scan_min_valid_ratio",
        ),
        (
            NavigationSpeedGate,
            [Parameter("emergency_scan_max_invalid_gap_angle", value=0.40)],
            "emergency_scan_max_invalid_gap_angle",
        ),
        (
            AutonomousMission,
            [
                Parameter("semantic_confirmation_votes", value=6),
                Parameter("semantic_recent_window", value=5),
            ],
            "semantic_confirmation_votes",
        ),
        (
            AutonomousMission,
            [Parameter("observation_frame", value="")],
            "observation_frame",
        ),
        (
            AutonomousMission,
            [Parameter("observation_frame", value="/base_link")],
            "observation_frame",
        ),
        (
            AutonomousMission,
            [Parameter("safety_guidance_spatial_tolerance", value=0.0)],
            "safety_guidance_spatial_tolerance",
        ),
        (
            AutonomousMission,
            [Parameter("navigation_health_timeout", value=0.0)],
            "navigation_health_timeout",
        ),
        (
            AutonomousMission,
            [Parameter("navigation_health_recovery_duration", value=0.0)],
            "navigation_health_recovery_duration",
        ),
        (
            AutonomousMission,
            [Parameter("traversal_progress_timeout", value=0.0)],
            "traversal_progress_timeout",
        ),
        (
            AutonomousMission,
            [Parameter("traversal_ready_max_distance", value=2.0)],
            "traversal_ready_max_distance",
        ),
        (
            AutonomousMission,
            [Parameter("traversal_ready_max_lateral", value=0.0)],
            "traversal_ready_max_lateral",
        ),
        (
            AutonomousMission,
            [Parameter("handoff_fallback_max_lateral", value=0.36)],
            "handoff_fallback_max_lateral",
        ),
        (
            AutonomousMission,
            [Parameter("approach_stall_handoff_max_lateral", value=0.0)],
            "approach_stall_handoff_max_lateral",
        ),
        (
            AutonomousMission,
            [Parameter("nav_progress_translation", value=0.0)],
            "nav_progress_translation",
        ),
        (
            AutonomousMission,
            [Parameter("post_traversal_beyond_margin", value=0.50)],
            "post_traversal_beyond_margin",
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


def test_speed_gate_distinguishes_absent_released_and_expired_mission_lease(
    ros_context,
):
    """普通 Nav2 无需任务心跳；失联 owner 锁存到速度门/核心栈重启。"""
    gate = NavigationSpeedGate(
        parameter_overrides=[
            Parameter("require_emergency_scan", value=False),
            Parameter("autonomy_lease_timeout", value=0.2),
        ]
    )
    output = MessageRecordingPublisher()
    gate.pub = output
    command = Twist()
    command.linear.x = 0.4
    try:
        # No lease history means no autonomous owner: an ordinary RViz/Nav2 goal works.
        gate.limit_callback(Float32(data=0.5))
        gate.health_callback(Bool(data=True))
        gate.cmd_callback(command)
        gate.publish_safe_command()
        assert output.messages[-1].linear.x == pytest.approx(0.2)

        # Acquisition itself is stopped even when a stale first packet asks for motion.
        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 1, True, motion_allowed=True)
        )
        gate.cmd_callback(command)
        gate.publish_safe_command()
        assert gate.autonomy_lease_state == "ACTIVE"
        assert not gate.autonomy_motion_allowed
        assert output.messages[-1].linear.x == 0.0

        # Only a later ordered permit from the exact owner can open the branch, and the
        # permit edge clears the cached pre-acceptance Twist.
        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 2, True, motion_allowed=True)
        )
        gate.cmd_callback(command)
        gate.publish_safe_command()
        assert gate.autonomy_motion_allowed
        assert output.messages[-1].linear.x == pytest.approx(0.2)

        # Process heartbeat interruption clears its old Twist and latches fail-closed.
        gate.last_autonomy_lease_time = (
            gate.get_clock().now() - Duration(seconds=1.0)
        )
        gate.publish_safe_command()
        assert gate.autonomy_lease_state == "EXPIRED"
        assert output.messages[-1].linear.x == 0.0
        gate.cmd_callback(command)
        # A delayed packet or a newly restarted mission can only send true; neither proves
        # that the old Nav2 goal stopped, so EXPIRED must remain latched.
        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 3, True, motion_allowed=True)
        )
        gate.publish_safe_command()
        assert gate.autonomy_lease_state == "EXPIRED"
        assert output.messages[-1].linear.x == 0.0

        # Even a newer matching release cannot prove ownership after expiry.
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 4, False))
        gate.publish_safe_command()
        assert gate.autonomy_lease_state == "EXPIRED"
        assert output.messages[-1].linear.x == 0.0
        gate.cmd_callback(command)
        gate.limit_callback(Float32(data=0.5))
        gate.health_callback(Bool(data=True))
        gate.publish_safe_command()
        assert output.messages[-1].linear.x == 0.0
    finally:
        gate.destroy_node()


def test_speed_gate_accepts_only_a_fresh_active_clean_lease_release(ros_context):
    """Normal Ctrl-C can release ACTIVE; a stale pre-timer false latches EXPIRED."""
    gate = NavigationSpeedGate(
        parameter_overrides=[
            Parameter("require_emergency_scan", value=False),
            Parameter("autonomy_lease_timeout", value=0.2),
        ]
    )
    command = Twist()
    command.linear.x = 0.3
    try:
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 1, True))
        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 2, True, motion_allowed=True)
        )
        gate.cmd_callback(command)
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 3, False))
        assert gate.autonomy_lease_state == "UNOWNED"
        # The ownership edge clears cached motion; a later ordinary Nav2 client must
        # produce a new Twist, preserving the existing clean Ctrl-C behavior.
        assert gate.last_cmd_time is None

        gate.autonomy_lease_callback(autonomy_lease("mission-b", 1, True))
        gate.last_autonomy_lease_time = (
            gate.get_clock().now() - Duration(seconds=1.0)
        )
        gate.autonomy_lease_callback(autonomy_lease("mission-b", 2, False))
        assert gate.autonomy_lease_state == "EXPIRED"
        assert gate.last_cmd_time is None
    finally:
        gate.destroy_node()


def test_speed_gate_lease_session_rejects_old_release_replay_and_bad_sequence(
    ros_context,
):
    """One old process can neither release nor reacquire a newer process's branch."""
    gate = NavigationSpeedGate(
        parameter_overrides=[Parameter("require_emergency_scan", value=False)]
    )
    try:
        # Empty identity and zero sequence are not acquisition heartbeats.
        gate.autonomy_lease_callback(autonomy_lease("", 1, True))
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 0, True))
        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 1, False, motion_allowed=True)
        )
        assert gate.autonomy_lease_state == "UNOWNED"

        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 1, True, motion_allowed=True)
        )
        assert gate.autonomy_lease_state == "ACTIVE"
        assert gate.autonomy_lease_session == "mission-a"
        assert not gate.autonomy_motion_allowed
        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 2, True, motion_allowed=True)
        )
        assert gate.autonomy_motion_allowed

        # A duplicate/replayed same-session release and another session's newer release
        # are both ignored.  Only the exact owner with a strictly newer sequence may end.
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 2, False))
        gate.autonomy_lease_callback(autonomy_lease("old-process", 9, False))
        assert gate.autonomy_lease_state == "ACTIVE"
        assert gate.autonomy_lease_session == "mission-a"
        assert gate.autonomy_motion_allowed
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 3, False))
        assert gate.autonomy_lease_state == "UNOWNED"

        # A delayed heartbeat older than the clean release remains below the retained
        # high-water mark and cannot reacquire UNOWNED.
        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 2, True, motion_allowed=True)
        )
        assert gate.autonomy_lease_state == "UNOWNED"
        gate.autonomy_lease_callback(
            autonomy_lease("mission-b", 1, True, motion_allowed=True)
        )
        assert gate.autonomy_lease_state == "ACTIVE"
        assert gate.autonomy_lease_session == "mission-b"
        assert not gate.autonomy_motion_allowed
        gate.autonomy_lease_callback(
            autonomy_lease("mission-b", 2, True, motion_allowed=True)
        )
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 4, False))
        assert gate.autonomy_lease_state == "ACTIVE"
        assert gate.autonomy_lease_session == "mission-b"
        assert gate.autonomy_motion_allowed
    finally:
        gate.destroy_node()


def test_speed_gate_anonymous_stop_false_cannot_unlock_or_replay_old_twist(ros_context):
    """Only an ordered permit may clear the legacy stop and cached command."""
    gate = NavigationSpeedGate(
        parameter_overrides=[Parameter("require_emergency_scan", value=False)]
    )
    output = MessageRecordingPublisher()
    gate.pub = output
    command = Twist()
    command.linear.x = 0.3
    try:
        gate.limit_callback(Float32(data=1.0))
        gate.health_callback(Bool(data=True))
        gate.cmd_callback(command)
        gate.publish_safe_command()
        assert output.messages[-1].linear.x == pytest.approx(0.3)

        gate.autonomy_stop_callback(Bool(data=True))
        # A delayed false from any old process is anonymous and must not clear the veto.
        gate.autonomy_stop_callback(Bool(data=False))
        gate.cmd_callback(command)
        gate.publish_safe_command()
        assert output.messages[-1].linear.x == 0.0
        assert gate.external_stop

        # First session contact acquires stopped.  The second same-session message is the
        # authenticated permit and clears both the legacy veto and cached pre-permit Twist.
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 1, True))
        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 2, True, motion_allowed=True)
        )
        assert not gate.external_stop
        assert gate.last_cmd_time is None
        gate.cmd_callback(command)
        gate.publish_safe_command()
        assert output.messages[-1].linear.x == pytest.approx(0.3)
    finally:
        gate.destroy_node()


def test_speed_gate_clock_rewind_clears_old_epoch_authority(ros_context):
    """A simulated /clock rewind must clear Twist/heartbeats and expire an active owner."""
    gate = NavigationSpeedGate(
        parameter_overrides=[Parameter("require_emergency_scan", value=False)]
    )
    output = MessageRecordingPublisher()
    gate.pub = output
    command = Twist()
    command.linear.x = 0.4
    try:
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 1, True))
        gate.autonomy_lease_callback(
            autonomy_lease("mission-a", 2, True, motion_allowed=True)
        )
        gate.limit_callback(Float32(data=1.0))
        gate.health_callback(Bool(data=True))
        gate.cmd_callback(command)
        gate.last_clock_time = gate.get_clock().now() + Duration(seconds=5.0)
        gate.publish_safe_command()
        assert output.messages[-1].linear.x == 0.0
        assert gate.autonomy_lease_state == "EXPIRED"
        assert gate.last_cmd_time is None
        assert gate.last_assessment_time is None
        assert gate.last_health_time is None
        # A second rewind cannot turn a latched ownership failure back into UNOWNED.
        gate.last_clock_time = gate.get_clock().now() + Duration(seconds=5.0)
        gate.publish_safe_command()
        assert gate.autonomy_lease_state == "EXPIRED"
        gate.autonomy_lease_callback(autonomy_lease("mission-a", 3, False))
        assert gate.autonomy_lease_state == "EXPIRED"
    finally:
        gate.destroy_node()


def test_shipped_terrain_yaml_exposes_all_runtime_safety_parameters():
    """Protect the YAML/Python contract for frames, typed input and temporal filters."""
    config_path = Path(__file__).parents[1] / "config" / "terrain_navigation.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    safety = config["terrain_safety_assessor"]["ros__parameters"]
    guidance = config["traversal_guidance"]["ros__parameters"]
    gate = config["navigation_speed_gate"]["ros__parameters"]
    assert safety["step_threshold"] == 0.07
    assert safety["output_frame"] == "base_link"
    assert safety["legacy_features_enabled"] is False
    assert guidance["type_confirmation_frames"] == 3
    assert gate["autonomy_lease_timeout"] > 0.0
    assert gate["emergency_scan_min_valid_ratio"] == 0.80
    assert gate["emergency_scan_max_invalid_gap_angle"] == 0.20


def test_python_fallbacks_match_every_shipped_terrain_yaml_value(ros_context):
    """Prevent Python declarations from silently becoming a second tuning surface."""
    config_path = Path(__file__).parents[1] / "config" / "terrain_navigation.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    nodes = {
        "terrain_safety_assessor": TerrainSafetyAssessor(),
        "traversal_guidance": TraversalGuidanceNode(),
        "navigation_speed_gate": NavigationSpeedGate(),
    }
    try:
        for node_name, node in nodes.items():
            expected = config[node_name]["ros__parameters"]
            for parameter_name, expected_value in expected.items():
                assert node.has_parameter(parameter_name), (
                    node_name,
                    parameter_name,
                )
                assert node.get_parameter(parameter_name).value == expected_value, (
                    node_name,
                    parameter_name,
                )
    finally:
        for node in nodes.values():
            node.destroy_node()


def test_no_vision_typed_invalid_geometry_publishes_stop(ros_context):
    """The live no-camera path must respect TerrainFeatures.valid, not plausible numbers."""
    assessor = TerrainSafetyAssessor(
        parameter_overrides=[Parameter("prefer_fused_obstacle", value=False)]
    )
    speeds = MessageRecordingPublisher()
    assessor.speed_pub = speeds
    features = TerrainFeatures()
    features.header.stamp = assessor.get_clock().now().to_msg()
    features.header.frame_id = "base_link"
    features.valid = False
    features.obstacle_type = TerrainFeatures.CLEAR
    features.confidence = 1.0
    features.valid_points = 200
    try:
        assessor.typed_features_callback(features)
        assert not assessor.perception_valid
        assert assessor.last_features_time is not None
        assert speeds.messages[-1].data == 0.0
        assert not assessor.get_parameter("legacy_features_enabled").value
    finally:
        assessor.destroy_node()


def test_mission_runtime_uses_five_second_recovery_defaults(ros_context):
    """Construct the production node and protect the requested non-blocking policy."""
    mission = AutonomousMission()
    try:
        assert mission.params["nav_stall_timeout"] == 5.0
        assert mission.params["nav_progress_translation"] == 0.04
        assert mission.params["nav_progress_rotation"] == 0.06
        assert mission.params["controller_wait_timeout"] == 2.0
        assert mission.params["action_response_timeout"] == 2.0
        assert mission.params["action_cancel_timeout"] == 2.0
        assert mission.params["traversal_progress_timeout"] == 5.0
        assert mission.params["traversal_ready_max_distance"] == 0.45
        assert mission.params["traversal_ready_max_lateral"] == 0.10
        assert mission.params["traversal_ready_max_heading_error"] == 0.08
        assert mission.params["safety_geometry_stale_seconds"] == 0.35
        assert mission.params["observation_frame"] == "base_link"
        assert mission.params["safety_guidance_spatial_tolerance"] == 0.12
        assert mission.params["navigation_health_timeout"] == 0.80
        assert mission.params["navigation_health_recovery_duration"] == 1.00
        assert mission.params["approach_stall_handoff_count"] == 1
        assert mission.params["maximum_search_turns"] == 4
        assert mission.params["pre_alignment_trigger_angle"] == 0.14
    finally:
        mission.destroy_node()
    with pytest.raises(ValueError, match="safety_geometry_stale_seconds"):
        AutonomousMission(
            parameter_overrides=[
                Parameter("safety_geometry_stale_seconds", value=0.0),
            ]
        )


def test_python_fallbacks_match_every_shipped_mission_yaml_value(ros_context):
    """Keep deploy-time YAML and no-file fallback behavior exactly equivalent."""
    config_path = Path(__file__).parents[1] / "config" / "autonomous_mission.yaml"
    shipped = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "autonomous_mission"
    ]["ros__parameters"]
    mission = AutonomousMission()
    try:
        assert set(mission.params) == set(shipped)
        assert mission.params == shipped
    finally:
        mission.destroy_node()


def test_out_of_order_typed_observations_cannot_mix_obstacle_identity(ros_context):
    mission = AutonomousMission()
    mission._robot_pose = lambda *_args: (0.0, 0.0, 0.0)
    try:
        newest_guidance = valid_wall_guidance()
        newest_guidance.header.stamp.sec = 20
        mission._guidance_callback(newest_guidance)
        older_guidance = valid_wall_guidance()
        older_guidance.header.stamp.sec = 19
        older_guidance.semantic_id = "t_shaped_stairs"
        older_guidance.obstacle_type = older_guidance.OBSTACLE_STEP
        mission._guidance_callback(older_guidance)
        assert mission.guidance is newest_guidance
        assert mission.guidance.semantic_id == "high_wall"

        newest_safety = valid_wall_safety()
        newest_safety.header.stamp.sec = 20
        mission._navigation_safety_callback(newest_safety)
        older_safety = valid_wall_safety()
        older_safety.header.stamp.sec = 19
        older_safety.semantic_id = "t_shaped_stairs"
        older_safety.obstacle_type = older_safety.OBSTACLE_STEP
        mission._navigation_safety_callback(older_safety)
        assert mission.last_safety is newest_safety
        assert mission.last_safety.semantic_id == "high_wall"
    finally:
        mission.destroy_node()


def test_newer_bad_payload_consumes_both_header_watermarks_and_blocks_older_pair(
    ros_context,
):
    """A superseded valid pair cannot regain authority after a newer malformed sample."""
    mission = AutonomousMission()
    mission._robot_pose = lambda *_args: (0.0, 0.0, 0.0)
    try:
        guidance_10 = valid_wall_guidance()
        safety_10 = valid_wall_safety()
        guidance_10.header.stamp.sec = 10
        safety_10.header.stamp.sec = 10
        mission._guidance_callback(guidance_10)
        mission._navigation_safety_callback(safety_10)

        guidance_bad_12 = deepcopy(guidance_10)
        safety_bad_12 = deepcopy(safety_10)
        guidance_bad_12.header.stamp.sec = 12
        safety_bad_12.header.stamp.sec = 12
        guidance_bad_12.distance = float("nan")
        safety_bad_12.obstacle_height = float("nan")
        mission._guidance_callback(guidance_bad_12)
        mission._navigation_safety_callback(safety_bad_12)
        assert mission.last_guidance_stamp == 12.0
        assert mission.last_safety_stamp == 12.0
        assert mission.guidance is None
        assert mission.last_safety is None

        # A delayed valid pair and a repaired duplicate of the consumed bad stamp are both
        # stale transport samples.  Neither may refresh receive time or restore authority.
        for stamp in (11, 12):
            delayed_guidance = deepcopy(guidance_10)
            delayed_safety = deepcopy(safety_10)
            delayed_guidance.header.stamp.sec = stamp
            delayed_safety.header.stamp.sec = stamp
            mission._guidance_callback(delayed_guidance)
            mission._navigation_safety_callback(delayed_safety)
        assert mission.last_guidance_stamp == 12.0
        assert mission.last_safety_stamp == 12.0
        assert mission.guidance is None
        assert mission.last_safety is None

        guidance_13 = deepcopy(guidance_10)
        safety_13 = deepcopy(safety_10)
        guidance_13.header.stamp.sec = 13
        safety_13.header.stamp.sec = 13
        mission._guidance_callback(guidance_13)
        mission._navigation_safety_callback(safety_13)
        assert mission.guidance is guidance_13
        assert mission.last_safety is safety_13
        assert mission._latest_navigation_safety(
            time.monotonic(), mission.guidance
        ) is safety_13
    finally:
        mission.destroy_node()


def test_bad_observation_header_revokes_cache_without_advancing_watermark(
    ros_context,
):
    """An unorderable Header fails closed but cannot poison the source watermark."""
    mission = AutonomousMission()
    mission._robot_pose = lambda *_args: (0.0, 0.0, 0.0)
    try:
        guidance_20 = valid_wall_guidance()
        safety_20 = valid_wall_safety()
        guidance_20.header.stamp.sec = 20
        safety_20.header.stamp.sec = 20
        mission._guidance_callback(guidance_20)
        mission._navigation_safety_callback(safety_20)

        bad_guidance_header = deepcopy(guidance_20)
        bad_safety_header = deepcopy(safety_20)
        bad_guidance_header.header.frame_id = "/base_link"
        bad_safety_header.header.stamp.sec = 0
        bad_safety_header.header.stamp.nanosec = 0
        mission._guidance_callback(bad_guidance_header)
        mission._navigation_safety_callback(bad_safety_header)
        assert mission.guidance is None
        assert mission.last_safety is None
        assert mission.last_guidance_stamp == 20.0
        assert mission.last_safety_stamp == 20.0

        # Since the invalid Header carried no ordering authority, the next genuinely newer
        # canonical pair recovers normally; an older pair remains below the prior watermark.
        guidance_19 = deepcopy(guidance_20)
        safety_19 = deepcopy(safety_20)
        guidance_19.header.stamp.sec = 19
        safety_19.header.stamp.sec = 19
        mission._guidance_callback(guidance_19)
        mission._navigation_safety_callback(safety_19)
        assert mission.guidance is None
        assert mission.last_safety is None

        guidance_21 = deepcopy(guidance_20)
        safety_21 = deepcopy(safety_20)
        guidance_21.header.stamp.sec = 21
        safety_21.header.stamp.sec = 21
        mission._guidance_callback(guidance_21)
        mission._navigation_safety_callback(safety_21)
        assert mission.guidance is guidance_21
        assert mission.last_safety is safety_21
    finally:
        mission.destroy_node()


def test_mission_clock_rewind_starts_a_clean_observation_epoch(ros_context):
    """Old stamp 100 must not block new stamp 1 or donate votes after /clock reset."""
    mission = AutonomousMission()
    stop = RecordingPublisher()
    mission.autonomy_stop_pub = stop
    mission._robot_pose = lambda *_args: (0.0, 0.0, 0.0)
    try:
        old_safety = valid_wall_safety()
        old_safety.header.stamp.sec = 100
        old_guidance = valid_wall_guidance()
        old_guidance.header.stamp.sec = 100
        mission._navigation_safety_callback(old_safety)
        mission._guidance_callback(old_guidance)
        mission.semantic_votes.extend(["high_wall"] * 3)
        mission.locked_obstacle_id = "high_wall"
        mission.locked_obstacle_position = (1.0, 0.0)
        mission.pending_traverse = old_guidance
        mission.pending_traverse_id = "high_wall"
        mission.pending_traverse_position = (1.0, 0.0)
        mission.observed_obstacles["high_wall"] = ObservedObstacle(
            "high_wall", 1.0, 0.0, 0.0, 0.0, 0.0, 0.9, time.monotonic()
        )
        mission.navigation_healthy = True
        mission.navigation_health_received = time.monotonic()
        mission.navigation_health_true_since = time.monotonic() - 10.0
        # An ownership fault represents a remote Action whose release is unknown.  The
        # epoch reset must preserve that latch while clearing only observation authority.
        mission.action_ownership_fault = True
        mission.action_fault_reason = "test unknown owner"

        mission.last_ros_clock_time = (
            mission.get_clock().now() + Duration(seconds=5.0)
        )
        mission._tick()

        assert mission.last_guidance_stamp is None
        assert mission.last_safety_stamp is None
        assert mission.guidance is None
        assert mission.last_safety is None
        assert list(mission.semantic_votes) == []
        assert mission.locked_obstacle_id == ""
        assert mission.pending_traverse is None
        assert mission.observed_obstacles == {}
        assert not mission.navigation_healthy
        assert not mission._navigation_health_is_stable(time.monotonic())
        assert mission.action_ownership_fault
        assert stop.values[-1] is True

        new_safety = valid_wall_safety()
        new_safety.header.stamp.sec = 1
        new_guidance = valid_wall_guidance()
        new_guidance.header.stamp.sec = 1
        mission._navigation_safety_callback(new_safety)
        mission._guidance_callback(new_guidance)
        assert mission.last_safety is new_safety
        assert mission.guidance is new_guidance
        assert mission.last_safety_stamp == pytest.approx(1.0)
        assert mission.last_guidance_stamp == pytest.approx(1.0)
    finally:
        mission.destroy_node()


def test_mission_clock_rewind_never_drops_live_traversal_ownership(ros_context):
    """A new sensor epoch cannot be treated as proof that a remote Action stopped."""
    mission = AutonomousMission()
    stop = RecordingPublisher()
    mission.autonomy_stop_pub = stop
    handle = object()
    pending = valid_wall_guidance()
    mission.traverse_handle = handle
    mission.pending_traverse = pending
    mission.pending_traverse_id = "high_wall"
    mission.pending_traverse_position = (1.0, 0.0)
    try:
        mission._reset_observation_epoch(mission.get_clock().now())
        assert mission.traverse_handle is handle
        assert mission.pending_traverse is pending
        assert mission.pending_traverse_id == "high_wall"
        assert mission.pending_traverse_position == (1.0, 0.0)
        assert stop.values[-1] is True
    finally:
        mission.destroy_node()


def test_navigation_health_pause_preserves_and_resumes_exact_nav_attempt(ros_context):
    mission = AutonomousMission()
    mission.enabled = True
    mission.nav_client = FakeActionClient(True)
    mission.nav_handle = FakeGoalHandle()
    mission.nav_goal_pose = mission._make_pose(2.0, 0.5, 0.2)
    mission.nav_target = (2.0, 0.5)
    mission.nav_purpose = "frontier"
    mission.search_turn_index = 3
    blocked_before = list(mission.blocked_frontiers)
    try:
        now = time.monotonic()
        mission._navigation_health_callback(Bool(data=False))
        assert mission._handle_navigation_health(now)
        assert mission.nav_cancel_pending
        assert mission.health_interrupted_nav is not None

        canceled = PendingFuture()
        canceled.value = SimpleNamespace(status=GoalStatus.STATUS_CANCELED)
        mission._nav_result(canceled)
        assert mission.state == "WAITING_FOR_NAVIGATION_HEALTH"
        assert mission.blocked_frontiers == blocked_before
        assert mission.search_turn_index == 3
        assert mission.nav_retry_until == 0.0

        recovered = time.monotonic()
        mission._navigation_health_callback(Bool(data=True))
        mission.navigation_health_true_since = (
            recovered
            - float(mission.params["navigation_health_recovery_duration"])
            - 0.1
        )
        mission.navigation_health_received = recovered
        assert mission._handle_navigation_health(recovered)
        assert mission.health_interrupted_nav is None
        assert mission.nav_send_pending
        assert mission.nav_client.sent == 1
        assert mission.blocked_frontiers == blocked_before
        assert mission.search_turn_index == 3
    finally:
        mission.destroy_node()


def test_first_true_heartbeat_after_stale_gap_restarts_recovery_dwell(ros_context):
    """An old true interval must not authorize the first heartbeat after an outage."""
    mission = AutonomousMission()
    try:
        now = time.monotonic()
        mission.navigation_healthy = True
        mission.navigation_health_true_since = now - 30.0
        mission.navigation_health_received = (
            now - float(mission.params["navigation_health_timeout"]) - 0.1
        )

        callback_started = time.monotonic()
        mission._navigation_health_callback(Bool(data=True))

        assert mission.navigation_health_true_since >= callback_started
        assert not mission._navigation_health_is_stable(time.monotonic())
        assert not mission._send_nav_goal(
            mission._make_pose(1.0, 0.0, 0.0), "frontier"
        )
    finally:
        mission.destroy_node()


def test_nav_send_watchdog_locks_speed_and_late_response_cannot_restore_goal(
    ros_context,
):
    """悬空 send response 有界进入故障；迟到 accepted handle 只能被取消。"""
    mission = AutonomousMission()
    install_stable_navigation_health(mission)
    pending = PendingFuture()
    client = FakeActionClient(False, pending)
    mission.nav_client = client
    lease = LeaseRecordingPublisher()
    mission.autonomy_lease_pub = lease
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
        assert lease.samples
        assert all(not motion_allowed for *_prefix, motion_allowed in lease.samples)

        late_handle = FakeGoalHandle()
        pending.complete(late_handle)
        assert late_handle.cancel_calls == 1
        assert mission.nav_handle is None
        assert mission.action_ownership_fault
    finally:
        mission.destroy_node()


@pytest.mark.parametrize("action_kind", ["nav", "traverse"])
def test_goal_response_callback_itself_rejects_response_after_watchdog_deadline(
    ros_context,
    action_kind,
):
    """A response queued before the 4 Hz timer cannot bypass its monotonic deadline."""
    mission = AutonomousMission()
    mission.enabled = True
    install_stable_navigation_health(mission)
    pending = PendingFuture()
    client = FakeActionClient(True, pending)
    lease = LeaseRecordingPublisher()
    mission.autonomy_lease_pub = lease
    try:
        if action_kind == "nav":
            mission.nav_client = client
            assert mission._send_nav_goal(
                mission._make_pose(1.0, 0.0, 0.0), "frontier"
            )
            mission.nav_send_started = (
                time.monotonic()
                - float(mission.params["action_response_timeout"])
                - 0.1
            )
        else:
            mission.traverse_client = client
            guidance = install_fresh_wall_handoff(mission)
            assert mission._start_traverse(guidance)
            mission.traverse_send_started = (
                time.monotonic()
                - float(mission.params["action_response_timeout"])
                - 0.1
            )

        late_handle = FakeGoalHandle()
        pending.complete(late_handle)
        assert late_handle.cancel_calls == 1
        assert mission.action_ownership_fault
        assert mission.state == "ACTION_COMMUNICATION_FAULT"
        assert mission.nav_handle is None
        assert mission.traverse_handle is None
        assert mission.completed_semantics == []
        assert lease.samples
        assert all(not motion_allowed for *_prefix, motion_allowed in lease.samples)
    finally:
        mission.destroy_node()


def test_shutdown_marks_disabled_before_pending_nav_response_and_cancels_late_handle(
    ros_context,
):
    """Ctrl-C during send must cancel the handle that appears during the drain loop."""
    mission = AutonomousMission()
    install_stable_navigation_health(mission)
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
    install_stable_navigation_health(mission)
    pending = PendingFuture()
    mission.nav_client = FakeActionClient(True, pending)
    mission.traverse_client = FakeActionClient(True)
    mission.enabled = True
    now = time.monotonic()
    mission.navigation_healthy = True
    mission.navigation_health_received = now
    mission.navigation_health_true_since = now - 2.0
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


def test_unstable_live_identity_does_not_report_handoff_queued(ros_context):
    """A rejected controller wait must return false and release the scheduler."""
    mission = AutonomousMission()
    mission.enabled = True
    mission.traverse_client = FakeActionClient(False)
    mission._action_semantic_id = lambda *_args: ""
    try:
        queued = mission._queue_traversal_handoff(
            valid_wall_guidance(),
            "high_wall",
            (1.0, 0.0),
            time.monotonic(),
        )
        assert not queued
        assert mission.pending_traverse is None
        assert mission.pending_traverse_id == ""
        assert mission.state == "RECOVERY"
        assert mission.blocked_obstacles
        assert mission.cooldown_until > 0.0
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
    """取消结果和 HANDOFF 均保持锁住；只有提交新 Nav2 goal 才解锁。"""
    mission = AutonomousMission()
    install_stable_navigation_health(mission)
    publisher = RecordingPublisher()
    mission.autonomy_stop_pub = publisher
    lease = LeaseRecordingPublisher()
    mission.autonomy_lease_pub = lease
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
        assert publisher.values[-2:] == [True, True]
        assert mission.nav_handle is None
        assert not mission.nav_cancel_pending

        mission.state = "EXPLORING"
        nav_client = FakeActionClient(True)
        mission.nav_client = nav_client
        assert mission._send_nav_goal(
            mission._make_pose(1.0, 0.0, 0.0), "frontier"
        )
        # Request/response is still stopped; only an accepted handle with an installed
        # result callback opens the branch.
        assert publisher.values[-1] is True
        assert lease.samples[-1][3] is False
        nav_client.future.complete(FakeGoalHandle())
        assert publisher.values[-1] is False
        assert lease.samples[-1][3] is True
    finally:
        mission.destroy_node()


def test_handoff_traversal_and_verification_never_unlock_nav2(ros_context):
    """Every non-Nav2 ownership phase must keep autonomy_stop true."""
    mission = AutonomousMission()
    install_stable_navigation_health(mission)
    stop = RecordingPublisher()
    mission.autonomy_stop_pub = stop
    mission.traverse_client = FakeActionClient(True)
    mission.enabled = True
    guidance = valid_wall_guidance()
    try:
        assert mission._queue_traversal_handoff(
            guidance, "high_wall", (1.0, 0.0), time.monotonic()
        )
        assert mission.state == "HANDOFF"
        assert stop.values[-1] is True
        # A state-machine error/direct call cannot open Nav2 while handoff owns motion.
        mission.nav_client = FakeActionClient(True)
        assert not mission._send_nav_goal(
            mission._make_pose(2.0, 0.0, 0.0), "frontier"
        )
        assert stop.values[-1] is True

        install_fresh_wall_handoff(mission, guidance)
        mission.state = "HANDOFF"
        assert mission._start_traverse(guidance)
        assert mission.state == "TRAVERSING"
        assert stop.values[-1] is True
    finally:
        mission.destroy_node()


def test_idle_or_restarted_mission_cannot_clear_an_expired_owner(ros_context):
    """Only the verified clean-release method may publish lease=false."""
    mission = AutonomousMission()
    lease = LeaseRecordingPublisher()
    mission.autonomy_lease_pub = lease
    mission.enabled = False
    mission.action_ownership_fault = False
    try:
        mission._tick()
        assert lease.samples == []
        mission._release_autonomy_owner()
        assert len(lease.samples) == 1
        session_id, sequence, active, motion_allowed = lease.samples[0]
        assert session_id == mission.autonomy_session_id
        assert sequence > 0
        assert not active
        assert not motion_allowed
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


def test_nav_send_or_result_monitor_failure_never_publishes_motion_permit(ros_context):
    """Unknown delivery and an unobservable accepted handle both remain stopped."""
    for failure_mode in ("send", "result_monitor"):
        mission = AutonomousMission()
        mission.enabled = True
        install_stable_navigation_health(mission)
        lease = LeaseRecordingPublisher()
        mission.autonomy_lease_pub = lease
        try:
            if failure_mode == "send":
                mission.nav_client = FakeActionClient(
                    True, exception=RuntimeError("DDS writer failed")
                )
                assert not mission._send_nav_goal(
                    mission._make_pose(1.0, 0.0, 0.0), "frontier"
                )
            else:
                client = FakeActionClient(True)
                mission.nav_client = client
                assert mission._send_nav_goal(
                    mission._make_pose(1.0, 0.0, 0.0), "frontier"
                )
                client.future.complete(BrokenResultHandle())
            assert mission.action_ownership_fault
            assert lease.samples
            assert all(
                not motion_allowed for *_prefix, motion_allowed in lease.samples
            )
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
        mission.last_safety.header.stamp.nanosec = 49_000_000
        assert not mission._start_traverse(guidance)
        assert client.sent == 0
        assert mission.pending_traverse is None

        guidance = install_fresh_wall_handoff(mission)
        mission.last_safety.header.frame_id = "/base_link"
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
        live.lateral_offset = 0.05
        live.heading_error = 0.08
        mission.guidance = live
        mission.guidance_received = time.monotonic()
        mission.last_safety.distance = live.distance
        mission.last_safety.lateral_offset = live.lateral_offset
        observed_headers = []

        def historical_pose(header=None):
            observed_headers.append(header)
            return (0.0, 0.0, 0.0)

        mission._robot_pose = historical_pose
        assert mission._start_traverse(queued)
        assert client.sent == 1
        assert mission.pending_traverse is not live
        assert client.last_goal.distance == pytest.approx(1.1)
        assert client.last_goal.distance != pytest.approx(queued.distance)
        assert client.last_goal.lateral_offset == pytest.approx(0.05)
        assert client.last_goal.heading_error == pytest.approx(0.08)
        assert client.last_goal.confidence == pytest.approx(live.confidence)
        assert client.last_goal.header == live.header
        assert client.last_goal.header is not live.header
        assert client.last_goal.obstacle_id == "high_wall"
        assert client.last_goal.obstacle_type == TraverseObstacle.Goal.OBSTACLE_WALL
        # Guidance READY is the broader task-level handoff at 1.20 m.  At 1.10 m the
        # Action must still perform PREPARING; it is not yet in the 0.45 m lift window.
        assert client.last_goal.entry_stage == TraverseObstacle.Goal.ENTRY_PREPARING
        safety = mission.last_safety
        for field in (
            "obstacle_height",
            "pit_depth",
            "slope_pitch",
            "slope_roll",
            "roughness",
            "width",
            "structure_heading",
            "structure_heading_confidence",
            "clearance_height",
            "valid_points",
        ):
            assert getattr(client.last_goal, field) == pytest.approx(
                getattr(safety, field)
            )
        # Guidance and Safety are each projected with historical TF, never latest TF.
        assert observed_headers == [live.header, safety.header]
        assert mission.pending_traverse_robot_start == (0.0, 0.0)
        assert mission.pending_traverse_position == pytest.approx((1.1, 0.05))
    finally:
        mission.destroy_node()


def test_traverse_snapshot_requires_points_identity_and_spatially_paired_safety(
    ros_context,
):
    """A Header match alone cannot combine geometry from a different obstacle."""
    mission = AutonomousMission()
    mission.traverse_client = FakeActionClient(True)
    try:
        guidance = install_fresh_wall_handoff(mission)
        mission.last_safety.valid_points = 0
        assert not mission._start_traverse(guidance)
        assert mission.traverse_client.sent == 0

        guidance = install_fresh_wall_handoff(mission)
        mission.last_safety.semantic_id = "t_shaped_stairs"
        assert not mission._start_traverse(guidance)
        assert mission.traverse_client.sent == 0

        guidance = install_fresh_wall_handoff(mission)
        mission.last_safety.distance = 3.0
        assert not mission._start_traverse(guidance)
        assert mission.traverse_client.sent == 0
    finally:
        mission.destroy_node()


def test_guidance_safety_spatial_pairing_has_a_tight_noise_boundary(ros_context):
    """Synchronized headers cannot hide geometry from another nearby structure."""
    mission = AutonomousMission()
    mission.traverse_client = FakeActionClient(True)
    try:
        guidance = install_fresh_wall_handoff(mission)
        mission.last_safety.distance = guidance.distance + 0.119
        assert mission._start_traverse(guidance)
        assert mission.traverse_client.sent == 1
    finally:
        mission.destroy_node()

    mission = AutonomousMission()
    mission.traverse_client = FakeActionClient(True)
    try:
        guidance = install_fresh_wall_handoff(mission)
        mission.last_safety.distance = guidance.distance + 0.121
        assert not mission._start_traverse(guidance)
        assert mission.traverse_client.sent == 0
    finally:
        mission.destroy_node()


def test_nonready_handoff_is_explicitly_a_preparing_action_goal(ros_context):
    """A guarded early/stall handoff may approach, but may not masquerade as READY."""
    mission = AutonomousMission()
    mission.traverse_client = FakeActionClient(True)
    try:
        guidance = valid_wall_guidance()
        guidance.phase = TraversalGuidance.PHASE_ALIGN
        guidance.ready_for_handoff = False
        guidance.heading_error = 0.10
        install_fresh_wall_handoff(mission, guidance)
        assert mission._start_traverse(guidance)
        assert (
            mission.traverse_client.last_goal.entry_stage
            == TraverseObstacle.Goal.ENTRY_PREPARING
        )
    finally:
        mission.destroy_node()


def test_guidance_ready_uses_controller_lift_window_for_action_entry_stage(
    ros_context,
):
    """Task READY at 1.20 m cannot bypass the controller's 0.45 m lift window."""
    mission = AutonomousMission()
    mission.traverse_client = FakeActionClient(True)
    try:
        guidance = valid_wall_guidance()
        install_fresh_wall_handoff(mission, guidance)
        # The default 1.0 m sample is READY to hand off, but the controller must still
        # close the remaining distance before lifting a foot.
        assert mission._start_traverse(guidance)
        assert (
            mission.traverse_client.last_goal.entry_stage
            == TraverseObstacle.Goal.ENTRY_PREPARING
        )
    finally:
        mission.destroy_node()

    mission = AutonomousMission()
    mission.traverse_client = FakeActionClient(True)
    try:
        guidance = valid_wall_guidance()
        guidance.distance = 0.45
        guidance.lateral_offset = 0.10
        guidance.heading_error = 0.08
        install_fresh_wall_handoff(mission, guidance)
        mission.last_safety.distance = guidance.distance
        mission.last_safety.lateral_offset = guidance.lateral_offset
        mission.pending_traverse_position = (
            guidance.distance, guidance.lateral_offset
        )
        assert mission._start_traverse(guidance)
        assert (
            mission.traverse_client.last_goal.entry_stage
            == TraverseObstacle.Goal.ENTRY_READY
        )
    finally:
        mission.destroy_node()


def test_action_handoff_rejects_lateral_pose_outside_controller_envelope(
    ros_context,
):
    """The default server's exact 0.35 m PREPARING boundary is deterministic."""
    mission = AutonomousMission()
    mission.traverse_client = FakeActionClient(True)
    try:
        guidance = valid_wall_guidance()
        guidance.phase = TraversalGuidance.PHASE_ALIGN
        guidance.ready_for_handoff = False
        guidance.lateral_offset = 0.35
        install_fresh_wall_handoff(mission, guidance)
        mission.last_safety.lateral_offset = guidance.lateral_offset
        mission.pending_traverse_position = (
            guidance.distance, guidance.lateral_offset
        )
        assert mission._start_traverse(guidance)
        assert mission.traverse_client.sent == 1
        assert (
            mission.traverse_client.last_goal.entry_stage
            == TraverseObstacle.Goal.ENTRY_PREPARING
        )
    finally:
        mission.destroy_node()

    mission = AutonomousMission()
    mission.traverse_client = FakeActionClient(True)
    try:
        guidance = valid_wall_guidance()
        guidance.phase = TraversalGuidance.PHASE_ALIGN
        guidance.ready_for_handoff = False
        guidance.lateral_offset = 0.351
        install_fresh_wall_handoff(mission, guidance)
        mission.last_safety.lateral_offset = guidance.lateral_offset
        mission.pending_traverse_position = (
            guidance.distance, guidance.lateral_offset
        )
        assert not mission._start_traverse(guidance)
        assert mission.traverse_client.sent == 0
    finally:
        mission.destroy_node()


def _traverse_feedback(state, progress):
    feedback = TraverseObstacle.Feedback()
    feedback.state = int(state)
    feedback.progress = float(progress)
    return feedback


def test_feedback_contract_enforces_entry_sequence_and_monotonic_progress():
    preparing = TraverseObstacle.Goal.ENTRY_PREPARING
    ready = TraverseObstacle.Goal.ENTRY_READY
    state_preparing = TraverseObstacle.Feedback.STATE_PREPARING
    state_traversing = TraverseObstacle.Feedback.STATE_TRAVERSING
    state_stabilizing = TraverseObstacle.Feedback.STATE_STABILIZING

    assert traversal_feedback_transition_is_valid(
        preparing, 0, 0.0, state_preparing, 0.0
    )
    assert not traversal_feedback_transition_is_valid(
        preparing, 0, 0.0, state_traversing, 0.1
    )
    assert not traversal_feedback_transition_is_valid(
        ready, 0, 0.0, state_traversing, 0.2
    )
    assert traversal_feedback_transition_is_valid(
        ready, 0, 0.0, state_preparing, 0.05
    )
    assert traversal_feedback_transition_is_valid(
        ready, state_preparing, 0.05, state_traversing, 0.2
    )
    assert traversal_feedback_transition_is_valid(
        ready, state_traversing, 0.2, state_stabilizing, 0.8
    )
    assert not traversal_feedback_transition_is_valid(
        ready, state_stabilizing, 0.8, state_traversing, 0.9
    )
    assert not traversal_feedback_transition_is_valid(
        ready, state_traversing, 0.5, state_traversing, 0.4
    )
    assert not traversal_feedback_transition_is_valid(
        ready, state_traversing, 0.5, state_stabilizing, float("nan")
    )


def test_feedback_generation_isolated_and_invalid_sequence_cancels_known_owner(
    ros_context,
):
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        generation = mission.traverse_generation
        handle = FakeGoalHandle()
        client.future.complete(handle)

        # A stale callback from a previous goal must not mutate the active generation.
        mission._traverse_feedback(
            SimpleNamespace(
                feedback=_traverse_feedback(
                    TraverseObstacle.Feedback.STATE_STABILIZING, 1.0
                )
            ),
            generation - 1,
        )
        assert mission.traverse_feedback_state == 0
        assert not mission.traverse_feedback_invalid

        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_PREPARING, 0.1)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_TRAVERSING, 0.4)
        )
        assert mission.traverse_feedback_state == TraverseObstacle.Feedback.STATE_TRAVERSING
        progress_time = mission.traverse_progress_time
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_TRAVERSING, 0.4)
        )
        assert mission.traverse_progress_time == progress_time
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_STABILIZING, 0.8)
        )
        # Regressing to TRAVERSING is a protocol fault.  The handle remains installed
        # until result arrives, while exactly one cancel request is issued.
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_TRAVERSING, 0.9)
        )
        assert mission.traverse_feedback_invalid
        assert handle.cancel_calls == 1
        assert mission.traverse_handle is handle
        assert mission.traverse_cancel_pending
    finally:
        mission.destroy_node()


def test_malformed_feedback_is_contained_cancelled_and_never_credited(ros_context):
    """A corrupt controller sample cannot escape the callback or satisfy success."""
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        handle = FakeGoalHandle()
        client.future.complete(handle)
        # Missing state/progress is a protocol error, not an executor exception.
        client.publish_feedback(SimpleNamespace())
        assert mission.traverse_feedback_invalid
        assert handle.cancel_calls == 1
        handle.result_future.complete(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(success=True, message="invalid fast success"),
        ))
        assert mission.traversal_verification is None
        assert mission.completed_semantics == []
        assert mission.state == "RECOVERY"
    finally:
        mission.destroy_node()


def test_feedback_no_progress_timeout_cancels_but_keeps_remote_ownership(
    ros_context,
):
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        handle = FakeGoalHandle()
        client.future.complete(handle)
        started = mission.traverse_progress_time
        assert not mission._check_traversal_progress(
            started + float(mission.params["traversal_progress_timeout"]) - 0.01
        )
        assert mission._check_traversal_progress(
            started + float(mission.params["traversal_progress_timeout"])
        )
        assert handle.cancel_calls == 1
        assert mission.traverse_handle is handle
        assert mission.traverse_cancel_pending
        assert mission.completed_semantics == []
    finally:
        mission.destroy_node()


def test_action_success_without_terminal_feedback_cannot_enter_verification(
    ros_context,
):
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        handle = FakeGoalHandle()
        client.future.complete(handle)
        wrapped = SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(success=True, message="premature success"),
        )
        handle.result_future.complete(wrapped)
        assert mission.traversal_verification is None
        assert mission.completed_semantics == []
        assert mission.state == "RECOVERY"
    finally:
        mission.destroy_node()


def test_partial_terminal_feedback_cannot_be_reported_as_success(ros_context):
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        handle = FakeGoalHandle()
        client.future.complete(handle)
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_PREPARING, 0.1)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_TRAVERSING, 0.5)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_STABILIZING, 0.99)
        )
        handle.result_future.complete(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(success=True, message="partial progress"),
        ))
        assert mission.traversal_verification is None
        assert mission.completed_semantics == []
        assert mission.state == "RECOVERY"
    finally:
        mission.destroy_node()


def test_valid_terminal_feedback_allows_independent_crossing_verification(ros_context):
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        handle = FakeGoalHandle()
        client.future.complete(handle)
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_PREPARING, 0.1)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_TRAVERSING, 0.5)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_STABILIZING, 1.0)
        )
        handle.result_future.complete(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(success=True, message="stable landing"),
        ))
        assert mission.traversal_verification is not None
        assert mission.traversal_verification.semantic_id == "high_wall"
        assert mission.completed_semantics == []
        assert mission.state == "VERIFYING_TRAVERSAL_RESULT"
    finally:
        mission.destroy_node()


def test_traverse_result_callback_rejects_success_after_execution_timeout(
    ros_context,
):
    """A late terminal result cannot win a scheduling race against the 4 Hz timer."""
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        handle = FakeGoalHandle()
        client.future.complete(handle)
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_PREPARING, 0.1)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_TRAVERSING, 0.5)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_STABILIZING, 1.0)
        )
        mission.traverse_started = (
            time.monotonic()
            - float(mission.params["traversal_timeout"])
            - 0.1
        )
        handle.result_future.complete(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(success=True, message="late stable landing"),
        ))
        assert mission.traversal_verification is None
        assert mission.completed_semantics == []
        assert mission.state == "RECOVERY"
    finally:
        mission.destroy_node()


def test_nav_result_callback_rejects_success_after_execution_timeout(ros_context):
    """A Nav2 success received after goal_timeout is handled as a failed target."""
    mission = AutonomousMission()
    mission.enabled = True
    install_stable_navigation_health(mission)
    client = FakeActionClient(True)
    mission.nav_client = client
    try:
        assert mission._send_nav_goal(
            mission._make_pose(1.0, 0.0, 0.0), "frontier"
        )
        handle = FakeGoalHandle()
        client.future.complete(handle)
        mission.nav_started = (
            time.monotonic() - float(mission.params["goal_timeout"]) - 0.1
        )
        handle.result_future.complete(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
        )
        assert mission.nav_handle is None
        assert mission.nav_target is None
        assert (1.0, 0.0) in mission.blocked_frontiers
        assert mission.pending_traverse is None
        assert mission.completed_semantics == []
    finally:
        mission.destroy_node()


def test_hard_deadline_without_handle_is_terminal_and_never_claims_return(
    ros_context,
):
    """At 300 s the mission stops even when map/TF/health inputs are unavailable."""
    mission = AutonomousMission()
    stop = RecordingPublisher()
    mission.autonomy_stop_pub = stop
    mission.enabled = True
    mission.home_pose = (0.0, 0.0, 0.0)
    mission.mission_started = (
        time.monotonic() - float(mission.params["mission_timeout"])
    )
    try:
        mission._tick()
        assert mission.hard_deadline_active
        assert not mission.enabled
        assert mission.state == "INCOMPLETE_STOP"
        assert not mission.returned_home
        assert mission.completed_semantics == []
        assert stop.values[-1] is True
        # A late scheduler call cannot open the Nav2 branch after the terminal state.
        install_stable_navigation_health(mission)
        mission.nav_client = FakeActionClient(True)
        assert not mission._send_nav_goal(
            mission._make_pose(1.0, 0.0, 0.0), "return_home"
        )
    finally:
        mission.destroy_node()


def test_hard_deadline_cancels_active_nav_and_late_success_stays_incomplete(
    ros_context,
):
    mission = AutonomousMission()
    handle = FakeGoalHandle()
    mission.enabled = True
    mission.home_pose = (0.0, 0.0, 0.0)
    mission.nav_generation = 4
    mission.nav_handle = handle
    mission.nav_purpose = "return_home"
    mission.nav_target = (0.0, 0.0)
    mission.nav_goal_pose = mission._make_pose(0.0, 0.0, 0.0)
    try:
        now = time.monotonic()
        mission.mission_started = now - float(mission.params["mission_timeout"])
        mission._enforce_hard_deadline(now)
        assert handle.cancel_calls == 1
        assert mission.nav_cancel_pending
        assert mission.nav_handle is handle
        result = PendingFuture()
        result.value = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
        mission._nav_result(result, 4)
        assert mission.nav_handle is None
        assert mission.state == "INCOMPLETE_STOP"
        assert not mission.returned_home
        assert mission.completed_semantics == []
    finally:
        mission.destroy_node()


def test_hard_deadline_cancels_active_traverse_and_ignores_late_success(
    ros_context,
):
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        generation = mission.traverse_generation
        handle = FakeGoalHandle()
        client.future.complete(handle)
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_PREPARING, 0.1)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_TRAVERSING, 0.5)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_STABILIZING, 1.0)
        )
        now = time.monotonic()
        mission.home_pose = (0.0, 0.0, 0.0)
        mission.mission_started = now - float(mission.params["mission_timeout"])
        mission._enforce_hard_deadline(now)
        assert handle.cancel_calls == 1
        assert mission.traverse_handle is handle
        result = PendingFuture()
        result.value = SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(success=True, message="late success"),
        )
        mission._traverse_result(result, generation)
        assert mission.traverse_handle is None
        assert mission.traversal_verification is None
        assert mission.completed_semantics == []
        assert mission.state == "INCOMPLETE_STOP"
    finally:
        mission.destroy_node()


def test_hard_deadline_pending_send_cancels_late_accepted_handle(ros_context):
    """A request with no handle remains owned until its late response can be cancelled."""
    mission = AutonomousMission()
    install_stable_navigation_health(mission)
    pending = PendingFuture()
    mission.nav_client = FakeActionClient(True, pending)
    mission.enabled = True
    mission.home_pose = (0.0, 0.0, 0.0)
    try:
        assert mission._send_nav_goal(
            mission._make_pose(2.0, 0.0, 0.0), "frontier"
        )
        now = time.monotonic()
        mission.mission_started = now - float(mission.params["mission_timeout"])
        mission._enforce_hard_deadline(now)
        assert mission.nav_send_pending
        late_handle = FakeGoalHandle()
        pending.complete(late_handle)
        assert mission.nav_handle is late_handle
        assert late_handle.cancel_calls == 1
        assert mission.nav_cancel_pending
        assert mission.state == "INCOMPLETE_STOP"
    finally:
        mission.destroy_node()


def test_hard_deadline_pending_traverse_send_cancels_late_acceptance(
    ros_context,
):
    """A handle-less Traverse request is retained until its callback can cancel it."""
    mission = AutonomousMission()
    pending = PendingFuture()
    mission.traverse_client = FakeActionClient(True, pending)
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        now = time.monotonic()
        mission.home_pose = (0.0, 0.0, 0.0)
        mission.mission_started = now - float(mission.params["mission_timeout"])
        mission._enforce_hard_deadline(now)
        assert mission.traverse_send_pending
        late_handle = FakeGoalHandle()
        pending.complete(late_handle)
        assert mission.traverse_handle is late_handle
        assert late_handle.cancel_calls == 1
        assert mission.traverse_cancel_pending
        assert mission.state == "INCOMPLETE_STOP"
        assert mission.completed_semantics == []
    finally:
        mission.destroy_node()


def test_goal_response_callback_itself_enforces_elapsed_hard_deadline(
    ros_context,
):
    """A response racing the 4 Hz timer cannot run until the next tick."""
    mission = AutonomousMission()
    install_stable_navigation_health(mission)
    pending = PendingFuture()
    mission.nav_client = FakeActionClient(True, pending)
    mission.enabled = True
    mission.home_pose = (0.0, 0.0, 0.0)
    try:
        assert mission._send_nav_goal(
            mission._make_pose(2.0, 0.0, 0.0), "frontier"
        )
        mission.mission_started = (
            time.monotonic() - float(mission.params["mission_timeout"])
        )
        late_handle = FakeGoalHandle()
        pending.complete(late_handle)
        assert mission.hard_deadline_active
        assert late_handle.cancel_calls == 1
        assert mission.nav_cancel_pending
        assert mission.state == "INCOMPLETE_STOP"
        assert not mission.returned_home
    finally:
        mission.destroy_node()


def test_result_callback_itself_rejects_success_after_hard_deadline(ros_context):
    """A terminal traversal result cannot win a race against the 300 s deadline."""
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        handle = FakeGoalHandle()
        client.future.complete(handle)
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_PREPARING, 0.1)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_TRAVERSING, 0.5)
        )
        client.publish_feedback(
            _traverse_feedback(TraverseObstacle.Feedback.STATE_STABILIZING, 1.0)
        )
        mission.home_pose = (0.0, 0.0, 0.0)
        mission.mission_started = (
            time.monotonic() - float(mission.params["mission_timeout"])
        )
        handle.result_future.complete(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(success=True, message="late success"),
        ))
        assert mission.hard_deadline_active
        assert mission.traversal_verification is None
        assert mission.completed_semantics == []
        assert mission.state == "INCOMPLETE_STOP"
        assert not mission.returned_home
    finally:
        mission.destroy_node()


def test_hard_deadline_cancel_timeout_latches_unknown_ownership_fault(ros_context):
    """No result after hard-stop cancel is an ownership fault, never a clean stop."""
    mission = AutonomousMission()
    stop = RecordingPublisher()
    mission.autonomy_stop_pub = stop
    handle = FakeGoalHandle()
    mission.enabled = True
    mission.home_pose = (0.0, 0.0, 0.0)
    mission.nav_generation = 5
    mission.nav_handle = handle
    mission.nav_purpose = "return_home"
    try:
        now = time.monotonic()
        mission.mission_started = now - float(mission.params["mission_timeout"])
        mission._enforce_hard_deadline(now)
        assert mission.nav_cancel_pending
        assert mission._check_action_watchdogs(
            mission.nav_cancel_started
            + float(mission.params["action_cancel_timeout"])
        )
        assert mission.action_ownership_fault
        assert mission.state == "INCOMPLETE_STOP_OWNERSHIP_FAULT"
        assert stop.values[-1] is True
        assert not mission.returned_home
        assert mission.completed_semantics == []
    finally:
        mission.destroy_node()


def test_work_deadline_cancels_unfinished_nav_without_consuming_failure_budget(
    ros_context,
):
    mission = AutonomousMission()
    mission.enabled = True
    mission.home_pose = (0.0, 0.0, 0.0)
    mission.nav_generation = 3
    mission.nav_handle = FakeGoalHandle()
    mission.nav_purpose = "frontier"
    mission.nav_target = (2.0, 0.0)
    blocked_before = list(mission.blocked_frontiers)
    try:
        now = time.monotonic()
        mission.mission_started = now - (
            float(mission.params["mission_timeout"])
            - float(mission.params["return_time_reserve"])
        )
        assert mission._work_deadline_reached(now)
        assert mission._begin_return_phase(now)
        assert mission.nav_cancel_pending
        assert mission.nav_cancel_reason == "work_deadline"
        result = PendingFuture()
        result.value = SimpleNamespace(status=GoalStatus.STATUS_CANCELED)
        mission._nav_result(result, 3)
        assert mission.state == "RETURNING_TO_FINISH"
        assert mission.blocked_frontiers == blocked_before
        assert not mission.returned_home
        assert not mission.hard_deadline_active
    finally:
        mission.destroy_node()


def test_nav_result_callback_latches_elapsed_work_deadline_without_new_work(
    ros_context,
):
    """A frontier result at 240 s transitions directly to return scheduling."""
    mission = AutonomousMission()
    mission.enabled = True
    mission.home_pose = (0.0, 0.0, 0.0)
    mission.nav_generation = 9
    mission.nav_handle = FakeGoalHandle()
    mission.nav_purpose = "frontier"
    mission.nav_target = (2.0, 0.0)
    blocked_before = list(mission.blocked_frontiers)
    try:
        mission.mission_started = time.monotonic() - (
            float(mission.params["mission_timeout"])
            - float(mission.params["return_time_reserve"])
        )
        result = PendingFuture()
        result.value = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
        mission._nav_result(result, 9)
        assert mission.return_phase_requested
        assert mission.state == "RETURNING_TO_FINISH"
        assert mission.blocked_frontiers == blocked_before
        assert mission.nav_handle is None
        assert not mission.returned_home
    finally:
        mission.destroy_node()


def test_work_deadline_cancels_unfinished_traverse_then_returns(ros_context):
    """The 240 s work boundary drains traversal ownership without fake credit."""
    mission = AutonomousMission()
    client = FakeActionClient(True)
    mission.traverse_client = client
    try:
        guidance = install_fresh_wall_handoff(mission)
        assert mission._start_traverse(guidance)
        generation = mission.traverse_generation
        handle = FakeGoalHandle()
        client.future.complete(handle)
        now = time.monotonic()
        mission.home_pose = (0.0, 0.0, 0.0)
        mission.mission_started = now - (
            float(mission.params["mission_timeout"])
            - float(mission.params["return_time_reserve"])
        )
        assert mission._begin_return_phase(now)
        assert handle.cancel_calls == 1
        assert mission.traverse_cancel_reason == "work deadline return"
        result = PendingFuture()
        result.value = SimpleNamespace(
            status=GoalStatus.STATUS_CANCELED,
            result=SimpleNamespace(success=False, message="work deadline"),
        )
        mission._traverse_result(result, generation)
        assert mission.traverse_handle is None
        assert mission.traversal_verification is None
        assert mission.completed_semantics == []
        assert mission.state == "RETURNING_TO_FINISH"
        assert mission.enabled
    finally:
        mission.destroy_node()


def test_work_deadline_does_not_cancel_an_already_active_return_goal(ros_context):
    """An early return remains the sole Nav owner when the reserve window begins."""
    mission = AutonomousMission()
    stop = RecordingPublisher()
    mission.autonomy_stop_pub = stop
    mission.enabled = True
    mission.home_pose = (0.0, 0.0, 0.0)
    mission.nav_handle = FakeGoalHandle()
    mission.nav_purpose = "return_home"
    try:
        now = time.monotonic()
        mission.mission_started = now - (
            float(mission.params["mission_timeout"])
            - float(mission.params["return_time_reserve"])
        )
        assert not mission._begin_return_phase(now)
        assert mission.nav_handle.cancel_calls == 0
        assert not mission.nav_cancel_pending
        assert stop.values == []
        assert mission.state == "RETURNING_TO_FINISH"
    finally:
        mission.destroy_node()


def test_successful_search_turn_reopens_free_space_coverage(ros_context):
    """A scan must be followed by fresh translational coverage, not another scan."""
    mission = AutonomousMission()
    mission.enabled = True
    mission.nav_generation = 4
    mission.nav_handle = FakeGoalHandle()
    mission.nav_purpose = "search_turn"
    mission.nav_target = (0.0, 0.0)
    mission.coverage_visited = [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0)]
    mission._robot_pose = lambda *_args: (1.25, -0.75, 1.0)
    try:
        result = PendingFuture()
        result.value = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
        mission._nav_result(result, 4)
        assert mission.coverage_visited == [(1.25, -0.75)]
        assert mission.state == "EXPLORING"
    finally:
        mission.destroy_node()


def test_coverage_prefers_live_costmap_for_translational_goal(
    ros_context, monkeypatch
):
    """Free roaming must choose stations against current terrain obstacles."""
    mission = AutonomousMission()
    mission.nav_client = FakeActionClient(True)
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = 20
    grid.info.height = 20
    grid.info.resolution = 0.2
    grid.info.origin.position.x = -2.0
    grid.info.origin.position.y = -2.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 400
    costmap = deepcopy(grid)
    now = time.monotonic()
    mission.enabled = True
    mission.map_msg = grid
    mission.map_received = now
    mission.costmap_msg = costmap
    mission.costmap_received = now
    mission.home_pose = (0.0, 0.0, 0.0)
    mission.mission_started = now
    mission.mission_ready_after = 0.0
    mission._robot_pose = lambda *_args: (0.0, 0.0, 0.0)
    install_stable_navigation_health(mission)
    observed_grids = []
    monkeypatch.setattr(
        mission_module, "extract_frontiers", lambda *args, **kwargs: []
    )

    def coverage_candidates(candidate_grid, *args, **kwargs):
        observed_grids.append(candidate_grid)
        return [Frontier(1.0, 0.0, 1, 1.0, 1.0)]

    monkeypatch.setattr(
        mission_module, "extract_coverage_goals", coverage_candidates
    )
    try:
        mission._tick()
        assert observed_grids == [costmap]
        assert mission.nav_send_pending
        assert mission.nav_purpose == "coverage"
        assert mission.state == "COVERAGE_EXPLORING"
    finally:
        mission.destroy_node()


def test_offline_traversal_controller_records_obstacle_and_keeps_exploring(
    ros_context, monkeypatch
):
    """Perception-only mode must not stop and align at every observed obstacle."""
    mission = AutonomousMission()
    mission.nav_client = FakeActionClient(True)
    mission.traverse_client = FakeActionClient(False)
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
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
    mission._robot_pose = lambda *_args: (0.0, 0.0, 0.0)
    install_stable_navigation_health(mission)
    guidance = valid_wall_guidance()
    guidance.phase = TraversalGuidance.PHASE_APPROACH
    guidance.ready_for_handoff = False
    mission._fresh_target = lambda: guidance
    mission._obstacle_position = lambda *_args: (0.8, 0.0)
    mission._current_obstacle_id = lambda *_args: "high_wall"
    monkeypatch.setattr(
        mission_module,
        "extract_frontiers",
        lambda *args, **kwargs: [Frontier(1.2, 0.6, 20, 1.34, 20.0)],
    )
    try:
        mission._tick()
        assert mission.pending_traverse is None
        assert mission.blocked_obstacles
        assert mission.nav_send_pending
        assert mission.nav_purpose == "frontier"
        assert mission.state == "EXPLORING"
    finally:
        mission.destroy_node()


def test_nav_server_not_ready_does_not_consume_search_recovery_or_revisit(
    ros_context, monkeypatch
):
    """未发送的补扫、恢复和回访必须在 Nav2 恢复后仍可执行。"""
    mission = AutonomousMission()
    mission.nav_client = FakeActionClient(False)
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
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
    mission._robot_pose = lambda *_args: (0.0, 0.0, 0.0)
    mission.navigation_healthy = True
    mission.navigation_health_received = now
    mission.navigation_health_true_since = now - 2.0
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
    """A missing server cools the entry but must not force another in-place turn."""
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
        assert mission.failed_entry_turn_pending == 0.0
        assert mission.failed_entry_escape_pending == 0.0
    finally:
        mission.destroy_node()


def test_ambiguous_obstacle_uses_one_view_then_releases_for_translation(ros_context):
    """Exhausted semantics must not append another in-place recovery rotation."""
    mission = AutonomousMission()
    guidance = valid_wall_guidance()
    mission._robot_pose = lambda *_args: (0.0, 0.0, 0.0)
    mission._obstacle_position = lambda *_args: (1.0, 0.0)
    mission.semantic_verification_attempts = int(
        mission.params["semantic_verification_max_attempts"]
    )
    mission.semantic_verification_position = (0.0, 0.0)
    try:
        mission._verify_ambiguous_obstacle(guidance, time.monotonic())
        assert mission.state == "RECOVERY"
        assert mission.blocked_obstacles
        assert mission.failed_entry_turn_pending == 0.0
        assert mission.failed_entry_escape_pending == 0.0
        assert mission.locked_obstacle_position is None
    finally:
        mission.destroy_node()

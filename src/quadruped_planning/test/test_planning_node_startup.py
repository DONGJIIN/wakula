"""ROS node-construction and watchdog tests for planning parameter contracts."""

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32

from quadruped_planning.autonomous_mission import AutonomousMission
from quadruped_planning.cmd_vel_gate import NavigationSpeedGate
from quadruped_planning.terrain_safety_assessor import TerrainSafetyAssessor
from quadruped_planning.traversal_guidance import TraversalGuidanceNode


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
        assert mission.params["approach_stall_handoff_count"] == 1
        assert mission.params["maximum_search_turns"] == 8
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

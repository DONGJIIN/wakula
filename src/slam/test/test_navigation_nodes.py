"""Real ROS-graph regression tests for navigation readiness and health gating.

The pure tests protect scan and odometry mathematics.  This file constructs the production
nodes and sends typed DDS messages so parameter overrides, TF discovery, timer evaluation,
transient-local health publication, and watchdog expiry are tested without Gazebo.
"""

import math
import time
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from slam.nav2_readiness_monitor import Nav2ReadinessMonitor
from slam.navigation_health_monitor import NavigationHealthMonitor


@pytest.fixture
def ros_context():
    """Create one isolated rclpy context and release all DDS resources after each test."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _spin_until(executor, predicate, timeout=1.0):
    """Spin bounded work while allowing DDS discovery and production timers to progress."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def _valid_scan(stamp):
    """Create a structurally valid 360-degree LaserScan with finite empty-space returns."""
    message = LaserScan()
    message.header.stamp = stamp
    message.header.frame_id = "lidar_link"
    message.angle_min = -math.pi
    message.angle_max = math.pi
    message.angle_increment = (message.angle_max - message.angle_min) / 359.0
    message.scan_time = 0.10
    message.time_increment = message.scan_time / 360.0
    message.range_min = 0.08
    message.range_max = 20.0
    message.ranges = [2.0] * 360
    return message


def _valid_odom(stamp):
    """Create a standard odom→base_link sample with normalized orientation."""
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.01
    message.pose.covariance[7] = 0.01
    return message


def test_navigation_nodes_accept_shipped_defaults(ros_context):
    """Construct both production monitors without sensors or lifecycle servers."""
    nodes = []
    try:
        nodes.extend((NavigationHealthMonitor(), Nav2ReadinessMonitor()))
        assert {node.get_name() for node in nodes} == {
            "navigation_health_monitor",
            "nav2_readiness_monitor",
        }
    finally:
        for node in reversed(nodes):
            node.destroy_node()


@pytest.mark.parametrize(
    ("node_factory", "overrides", "message"),
    (
        (
            NavigationHealthMonitor,
            [Parameter("minimum_scan_valid_ratio", value=0.0)],
            "minimum_scan_valid_ratio",
        ),
        (
            Nav2ReadinessMonitor,
            [Parameter("scan_topic", value="scan")],
            "scan_topic",
        ),
    ),
)
def test_navigation_nodes_reject_invalid_overrides(
    ros_context, node_factory, overrides, message
):
    """Fail before subscriptions/services are created for unsafe health configuration."""
    with pytest.raises(ValueError, match=message):
        node_factory(parameter_overrides=overrides)


def test_readiness_wires_yaw_covariance_and_jump_into_activation_gate(ros_context):
    """A finite odom stream with a sudden 180-degree yaw reset must remain unready."""
    monitor = Nav2ReadinessMonitor()
    try:
        first = _valid_odom(monitor.get_clock().now().to_msg())
        first.pose.covariance[35] = 0.01
        monitor._odom_callback(first)
        assert monitor.odom_valid
        assert not monitor.odom_jump

        jumped = _valid_odom(monitor.get_clock().now().to_msg())
        jumped.pose.covariance[35] = 0.01
        jumped.pose.pose.orientation.w = 0.0
        jumped.pose.pose.orientation.z = 1.0
        monitor._odom_callback(jumped)
        assert monitor.odom_valid
        assert monitor.odom_jump

        uncertain = _valid_odom(monitor.get_clock().now().to_msg())
        uncertain.pose.covariance[35] = 5.0
        monitor._odom_callback(uncertain)
        assert not monitor.odom_valid
        assert monitor.odom_jump
    finally:
        monitor.destroy_node()


@pytest.mark.parametrize(
    ("node_factory", "freshness_method"),
    (
        (NavigationHealthMonitor, "_fresh"),
        (Nav2ReadinessMonitor, "_sensor_is_fresh"),
    ),
)
def test_sensor_freshness_rechecks_source_header_age_each_cycle(
    ros_context, node_factory, freshness_method
):
    """A fresh callback receipt must not extend the older Header deadline."""
    monitor = node_factory(
        parameter_overrides=[Parameter("sensor_timeout", value=0.25)]
    )
    try:
        now = monitor.get_clock().now()
        # Reception happened now, while this source sample is already older than the
        # configured deadline.  The periodic gate must inspect both values rather than
        # granting a second full timeout from callback arrival.
        old_source_nanoseconds = now.nanoseconds - 300_000_000
        old_source = (
            old_source_nanoseconds // 1_000_000_000,
            old_source_nanoseconds % 1_000_000_000,
        )
        assert getattr(monitor, freshness_method)(now)
        assert not getattr(monitor, freshness_method)(now, old_source)
    finally:
        monitor.destroy_node()


def test_periodic_health_and_readiness_gates_use_cached_source_stamps(ros_context):
    """Online timer paths, not only helpers, must pass both sensor time domains."""
    health = NavigationHealthMonitor()
    readiness = Nav2ReadinessMonitor()
    try:
        now = health.get_clock().now()
        health.scan_valid = True
        health.odom_valid = True
        health.last_scan_time = now
        health.last_odom_time = now
        health.last_scan_source_stamp = (11, 12)
        health.last_odom_source_stamp = (21, 22)
        health_calls = []
        health._fresh = (
            lambda receipt, source=None: health_calls.append((receipt, source))
            or False
        )
        health.evaluate()
        assert [source for _receipt, source in health_calls] == [
            (11, 12),
            (21, 22),
        ]

        now = readiness.get_clock().now()
        readiness.scan_valid = True
        readiness.odom_valid = True
        readiness.odom_jump = False
        readiness.last_scan_time = now
        readiness.last_odom_time = now
        readiness.last_scan_source_stamp = (31, 32)
        readiness.last_odom_source_stamp = (41, 42)
        readiness_calls = []
        readiness._sensor_is_fresh = (
            lambda receipt, source=None: readiness_calls.append((receipt, source))
            or False
        )
        readiness._check_readiness()
        assert [source for _receipt, source in readiness_calls] == [
            (31, 32),
            (41, 42),
        ]
    finally:
        readiness.destroy_node()
        health.destroy_node()


class _PendingFuture:
    """Small future double which records timeout cancellation."""

    def __init__(self):
        self.cancelled = False

    def done(self):
        return False

    def cancel(self):
        self.cancelled = True


class _CompletedFuture:
    """Return one response to exercise late-callback generation isolation."""

    def __init__(self, response):
        self.response = response

    def result(self):
        return self.response


class _CallbackRegistrationFailure(_PendingFuture):
    """Represent a middleware future that rejects callback registration."""

    def add_done_callback(self, _callback):
        raise RuntimeError("callback registration failed")


class _RaisingClient:
    """Service client double whose send path fails synchronously."""

    @staticmethod
    def service_is_ready():
        return True

    @staticmethod
    def call_async(_request):
        raise RuntimeError("service send failed")


class _ReturningClient:
    """Service client double returning one supplied future."""

    def __init__(self, future):
        self.future = future

    @staticmethod
    def service_is_ready():
        return True

    def call_async(self, _request):
        return self.future


class _CountingClient(_ReturningClient):
    """Record whether a late state reply incorrectly sends a transition."""

    def __init__(self, future=None):
        super().__init__(future)
        self.calls = 0

    def call_async(self, _request):
        self.calls += 1
        return self.future


def test_readiness_service_timeouts_release_guards_and_ignore_late_replies(ros_context):
    """A vanished lifecycle server cannot permanently wedge either request guard."""
    monitor = Nav2ReadinessMonitor()
    try:
        startup_future = _PendingFuture()
        monitor.startup_requested = True
        monitor.startup_request_generation = 4
        monitor.startup_request_deadline = 10.0
        monitor.startup_request_future = startup_future

        recovery_future = _PendingFuture()
        monitor.slam_recovery_pending = True
        monitor.slam_recovery_generation = 8
        monitor.slam_recovery_deadline = 10.0
        monitor.slam_recovery_future = recovery_future

        monitor._expire_service_requests(now_monotonic=11.0)
        assert not monitor.startup_requested
        assert not monitor.slam_recovery_pending
        assert startup_future.cancelled
        assert recovery_future.cancelled
        assert monitor.startup_request_generation == 5
        assert monitor.slam_recovery_generation == 9

        # The old service may still reply after middleware cancellation.  Its captured
        # generation must not mark Nav2 active or clear a newer recovery request.
        monitor._startup_response(
            _CompletedFuture(SimpleNamespace(success=True)), generation=4
        )
        monitor.slam_recovery_pending = True
        monitor.slam_recovery_future = object()
        monitor._slam_transition_response(
            _CompletedFuture(SimpleNamespace(success=True)), generation=8
        )
        assert not monitor.startup_complete
        assert monitor.slam_recovery_pending
    finally:
        monitor.destroy_node()


def test_readiness_service_dispatch_errors_never_leave_a_pending_guard(ros_context):
    """Synchronous send/registration exceptions fail closed and remain retryable."""
    monitor = Nav2ReadinessMonitor()
    try:
        monitor.lifecycle_client = _RaisingClient()
        assert not monitor._request_nav2_startup()
        assert not monitor.startup_requested

        startup_future = _CallbackRegistrationFailure()
        monitor.lifecycle_client = _ReturningClient(startup_future)
        assert not monitor._request_nav2_startup()
        assert not monitor.startup_requested
        assert startup_future.cancelled

        monitor.node_started_monotonic = time.monotonic() - 10.0
        monitor.last_slam_recovery_time = None
        monitor.slam_get_state_client = _RaisingClient()
        monitor._recover_slam_if_needed()
        assert not monitor.slam_recovery_pending

        state_future = _CallbackRegistrationFailure()
        monitor.last_slam_recovery_time = None
        monitor.slam_get_state_client = _ReturningClient(state_future)
        monitor._recover_slam_if_needed()
        assert not monitor.slam_recovery_pending
        assert state_future.cancelled

        unconfigured = SimpleNamespace(
            current_state=SimpleNamespace(id=1)
        )
        monitor.slam_recovery_pending = True
        monitor.slam_recovery_generation += 1
        generation = monitor.slam_recovery_generation
        monitor.slam_change_state_client = _RaisingClient()
        monitor._slam_state_response(
            _CompletedFuture(unconfigured), generation=generation
        )
        assert not monitor.slam_recovery_pending

        transition_future = _CallbackRegistrationFailure()
        monitor.slam_recovery_pending = True
        monitor.slam_recovery_generation += 1
        generation = monitor.slam_recovery_generation
        monitor.slam_change_state_client = _ReturningClient(transition_future)
        monitor._slam_state_response(
            _CompletedFuture(unconfigured), generation=generation
        )
        assert not monitor.slam_recovery_pending
        assert transition_future.cancelled
    finally:
        monitor.destroy_node()


def test_readiness_callbacks_accept_current_success_but_reject_other_late_responses(
    ros_context, monkeypatch
):
    """A current successful STARTUP closes the active-state race; other replies stay bounded."""
    monitor = Nav2ReadinessMonitor()
    try:
        monkeypatch.setattr(
            "slam.nav2_readiness_monitor.time.monotonic", lambda: 11.0
        )
        monitor.startup_requested = True
        monitor.startup_request_generation = 2
        monitor.startup_request_deadline = 10.0
        monitor._startup_response(
            _CompletedFuture(SimpleNamespace(success=True)), generation=2
        )
        assert monitor.startup_complete
        assert monitor.startup_requested
        assert monitor.startup_request_generation == 2

        transition_client = _CountingClient()
        monitor.slam_change_state_client = transition_client
        monitor.slam_recovery_pending = True
        monitor.slam_recovery_generation = 5
        monitor.slam_recovery_deadline = 10.0
        unconfigured = SimpleNamespace(current_state=SimpleNamespace(id=1))
        monitor._slam_state_response(
            _CompletedFuture(unconfigured), generation=5
        )
        assert transition_client.calls == 0
        assert not monitor.slam_recovery_pending
        assert monitor.slam_recovery_generation == 6

        monitor.slam_recovery_pending = True
        monitor.slam_recovery_generation = 7
        monitor.slam_recovery_deadline = 10.0
        monitor._slam_transition_response(
            _CompletedFuture(SimpleNamespace(success=True)), generation=7
        )
        assert not monitor.slam_recovery_pending
        assert monitor.slam_recovery_generation == 8
    finally:
        monitor.destroy_node()


def test_navigation_health_transitions_true_then_false_on_real_dds(ros_context):
    """动态 TF 冻结时，即使 scan/odom 继续更新也必须关闭健康门。"""
    monitor = NavigationHealthMonitor(
        parameter_overrides=[
            Parameter("global_frame", value="odom"),
            Parameter("sensor_timeout", value=0.25),
        ]
    )
    driver = Node("navigation_health_test_driver")
    scan_publisher = driver.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    odom_publisher = driver.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
    health_messages = []
    diagnostics = []
    driver.create_subscription(Bool, "/navigation/healthy", health_messages.append, 10)
    driver.create_subscription(DiagnosticArray, "/diagnostics", diagnostics.append, 10)
    broadcaster = TransformBroadcaster(driver)
    transform = TransformStamped()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    transform.transform.rotation.w = 1.0

    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(driver)
    try:
        assert _spin_until(
            executor,
            lambda: scan_publisher.get_subscription_count() > 0
            and odom_publisher.get_subscription_count() > 0,
            0.8,
        )
        # Republish while TF discovery settles.  Each message gets its real source timestamp;
        # replaying one old sample must not be able to hold the health gate open.
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and not any(msg.data for msg in health_messages):
            stamp = driver.get_clock().now().to_msg()
            transform.header.stamp = stamp
            broadcaster.sendTransform(transform)
            scan_publisher.publish(_valid_scan(stamp))
            odom_publisher.publish(_valid_odom(stamp))
            executor.spin_once(timeout_sec=0.04)
        assert any(message.data for message in health_messages)
        assert _spin_until(
            executor,
            lambda: any(
                value.key == "tf_age_seconds"
                for array in diagnostics
                for status in array.status
                for value in status.values
            ),
            0.5,
        )

        # 只冻结 TF；持续发送带新源时间的 scan/odom，证明 false 的原因不是传感器断流。
        health_messages.clear()
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline and not any(
            not message.data for message in health_messages
        ):
            stamp = driver.get_clock().now().to_msg()
            scan_publisher.publish(_valid_scan(stamp))
            odom_publisher.publish(_valid_odom(stamp))
            executor.spin_once(timeout_sec=0.04)
        assert any(not message.data for message in health_messages)
        assert not health_messages[-1].data
    finally:
        executor.remove_node(driver)
        executor.remove_node(monitor)
        driver.destroy_node()
        monitor.destroy_node()
        executor.shutdown()

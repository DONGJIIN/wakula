"""Real ROS-graph regression tests for navigation readiness and health gating.

The pure tests protect scan and odometry mathematics.  This file constructs the production
nodes and sends typed DDS messages so parameter overrides, TF discovery, timer evaluation,
transient-local health publication, and watchdog expiry are tested without Gazebo.
"""

import math
import time

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

"""Tests for the minimal Nav2 and SLAM integration."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
import rclpy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from slam.nav2_readiness_monitor import Nav2ReadinessMonitor
from slam.sensor_profiles import load_sensor_profiles, resolve_sensor_topics


PACKAGE_ROOT = Path(__file__).parents[1]


def launch_argument_names(description):
    """Return public argument names from one launch description."""
    return {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def test_local_costmap_fuses_scan_and_depth_points():
    """The local planner must receive both lidar and transformed depth data."""
    nav2_file = PACKAGE_ROOT / "config" / "nav2.yaml"
    with nav2_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    obstacle_layer = costmap["obstacle_layer"]
    assert obstacle_layer["observation_sources"] == "scan terrain_points"
    assert obstacle_layer["terrain_points"]["topic"] == (
        "/perception/obstacle_points"
    )
    assert obstacle_layer["terrain_points"]["data_type"] == "PointCloud2"


def test_navigation_launch_description_is_constructible():
    """The reduced Nav2 launch entry must remain importable."""
    path = PACKAGE_ROOT / "launch" / "navigation.launch.py"
    spec = importlib.util.spec_from_file_location("navigation_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert len(description.entities) >= 2


def test_sensor_profiles_cover_common_devices_and_allow_overrides():
    """Profiles are data-driven and an unknown device needs no source edit."""
    profiles = load_sensor_profiles(
        str(PACKAGE_ROOT / "config" / "sensor_profiles.yaml")
    )
    expected = {
        "ros_default",
        "rplidar",
        "ydlidar",
        "realsense_d400",
        "orbbec_gemini2",
        "zed2",
        "oak_d",
        "velodyne",
        "ouster",
        "livox",
        "hesai",
        "robosense",
        "lslidar",
    }
    assert expected <= profiles.keys()
    resolved = resolve_sensor_topics(
        profiles,
        "realsense_d400",
        {"scan_topic": "/front/scan", "camera_topic": ""},
    )
    assert resolved["scan_topic"] == "/front/scan"
    assert resolved["camera_topic"] == "/camera/camera/color/image_raw"


def test_sensor_compat_launch_exposes_one_hardware_adaptation_point():
    """The compatibility entry publishes all replaceable source arguments."""
    path = PACKAGE_ROOT / "launch" / "sensor_compat.launch.py"
    spec = importlib.util.spec_from_file_location("sensor_compat_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert {
        "sensor_profile",
        "sensor_profiles_file",
        "scan_topic",
        "odom_topic",
        "camera_topic",
        "point_cloud_topic",
    } <= launch_argument_names(description)


def test_all_in_one_launch_exposes_complete_user_controls():
    """The one-command entry forwards runtime and sensor controls."""
    path = PACKAGE_ROOT / "launch" / "all_in_one.launch.py"
    spec = importlib.util.spec_from_file_location("all_in_one_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert {
        "sensor_profile",
        "scan_topic",
        "odom_topic",
        "camera_topic",
        "point_cloud_topic",
        "use_control",
        "rviz",
        "vision",
        "competition",
        "nav2_autostart",
    } <= launch_argument_names(description)


def test_readiness_monitor_does_not_start_without_localization_tf():
    """Sensor messages alone must not activate Nav2 without localization."""
    rclpy.init()
    node = Nav2ReadinessMonitor()
    try:
        node._scan_callback(LaserScan())
        node._odom_callback(Odometry())
        node._check_readiness()
        assert node.scan_received
        assert node.odom_received
        assert not node.startup_requested
        assert node._sensor_is_fresh(node.last_scan_time)
        assert node._sensor_is_fresh(node.last_odom_time)
    finally:
        node.destroy_node()
        rclpy.shutdown()

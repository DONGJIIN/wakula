"""Tests for the unified Nav2, SLAM and perception launch integration."""

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


def test_navigation_health_parameters_are_versioned_with_nav2():
    """导航健康阈值必须进入正式配置，不能只隐藏在源码默认值中。"""
    nav2_file = PACKAGE_ROOT / "config" / "nav2.yaml"
    with nav2_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    health = config["navigation_health_monitor"]["ros__parameters"]
    assert health["minimum_scan_valid_ratio"] >= 0.5
    assert health["sensor_timeout"] > 0.0
    assert 0.0 <= health["future_stamp_tolerance"] <= 0.2
    assert health["odom_jump_recovery_samples"] >= 2


def test_navigation_launch_description_is_constructible():
    """The reduced Nav2 launch entry must remain importable."""
    path = PACKAGE_ROOT / "launch" / "navigation.launch.py"
    spec = importlib.util.spec_from_file_location("navigation_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert len(description.entities) >= 2
    source = path.read_text(encoding="utf-8")
    assert '"use_sim_time": use_sim_time' in source


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


def test_slam_launch_is_the_complete_one_command_entry():
    """The public entry exposes runtime, sensor and config controls."""
    path = PACKAGE_ROOT / "launch" / "slam.launch.py"
    spec = importlib.util.spec_from_file_location("slam_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert {
        "sensor_profile",
        "scan_topic",
        "odom_topic",
        "camera_topic",
        "point_cloud_topic",
        "slam_enabled",
        "nav2_enabled",
        "nav2_autostart",
        "vision",
        "robot_model",
        "rviz",
        "slam_params_file",
        "nav2_params_file",
        "vision_params_file",
        "terrain_params_file",
        "terrain_navigation_params_file",
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

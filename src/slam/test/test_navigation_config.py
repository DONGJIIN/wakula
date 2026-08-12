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
from slam.collision_monitor_supervisor import DEFAULT_DRAIN_SECONDS, _drain_seconds


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
    assert health["minimum_scan_samples"] >= 90
    assert health["minimum_scan_field_of_view"] >= 3.0
    assert health["expected_odom_frame"] == "odom"
    assert health["sensor_timeout"] > 0.0
    assert 0.0 <= health["future_stamp_tolerance"] <= 0.2
    assert health["odom_jump_recovery_samples"] >= 2
    readiness = config["nav2_readiness_monitor"]["ros__parameters"]
    # 启动门与运行期健康门必须使用相同传感器合同，避免“能启动但立即不健康”。
    for key in (
        "future_stamp_tolerance",
        "minimum_scan_valid_ratio",
        "minimum_scan_samples",
        "minimum_scan_field_of_view",
        "max_xy_covariance",
        "expected_odom_frame",
    ):
        assert readiness[key] == health[key]


def test_bt_navigator_declares_every_custom_tree_error_code():
    """自定义恢复树新增行为时，错误码必须同步登记到 Nav2 聚合列表。"""
    nav2_file = PACKAGE_ROOT / "config" / "nav2.yaml"
    with nav2_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    configured = set(
        config["bt_navigator"]["ros__parameters"]["error_code_names"]
    )
    tree = (PACKAGE_ROOT / "behavior_trees" / "navigate_to_pose_wakula.xml").read_text(
        encoding="utf-8"
    )
    referenced = {
        name
        for name in (
            "compute_path_error_code",
            "follow_path_error_code",
            "backup_error_code",
            "spin_error_code",
        )
        if "{" + name + "}" in tree
    }
    assert referenced <= configured


def test_custom_tree_selects_configured_goal_and_progress_checkers():
    """行为树应显式选择唯一检查器，避免用空 ID 触发运行期回退警告。"""
    nav2_file = PACKAGE_ROOT / "config" / "nav2.yaml"
    with nav2_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    controller = config["controller_server"]["ros__parameters"]
    tree = (PACKAGE_ROOT / "behavior_trees" / "navigate_to_pose_wakula.xml").read_text(
        encoding="utf-8"
    )
    assert controller["goal_checker_plugins"] == ["goal_checker"]
    assert controller["progress_checker_plugins"] == ["progress_checker"]
    assert 'goal_checker_id="goal_checker"' in tree
    assert 'progress_checker_id="progress_checker"' in tree


def test_slam_scan_queue_absorbs_short_tf_jitter_without_large_backlog():
    """异步扫描队列要大于默认 1，同时限制在不足半秒的 15 Hz 扫描量。"""
    slam_file = PACKAGE_ROOT / "config" / "slam.yaml"
    with slam_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    queue_size = config["slam_toolbox"]["ros__parameters"]["scan_queue_size"]
    assert 2 <= queue_size <= 7


def test_mapping_and_rviz_follow_live_robot_without_long_visual_lag():
    """地图发布应快于明显运动滞后，RViz 视图则只跟随而不改变 map 固定坐标。"""
    with (PACKAGE_ROOT / "config" / "slam.yaml").open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    params = config["slam_toolbox"]["ros__parameters"]
    assert 0.1 <= params["map_update_interval"] <= 0.5
    rviz = (PACKAGE_ROOT / "rviz" / "slam.rviz").read_text(encoding="utf-8")
    assert "Fixed Frame: map" in rviz
    assert "Target Frame: base_link" in rviz


def test_depth_costmap_accepts_low_steps_after_ground_filtering():
    """上游按相对地面滤波后，Nav2 不能再用正 z 下限漏掉机身下方台阶。"""
    nav2_file = PACKAGE_ROOT / "config" / "nav2.yaml"
    with nav2_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    source = config["local_costmap"]["local_costmap"]["ros__parameters"][
        "obstacle_layer"
    ]["terrain_points"]
    assert source["min_obstacle_height"] < 0.0
    assert source["max_obstacle_height"] > 0.5


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


def test_collision_monitor_uses_ordered_shutdown_supervisor():
    """退出必须隔离官方信号竞态，同时继续沿用标准 collision_monitor 节点名。"""
    path = PACKAGE_ROOT / "launch" / "navigation.launch.py"
    source = path.read_text(encoding="utf-8")
    assert '"collision_monitor_supervisor"' in source
    assert 'name="collision_monitor"' in source
    assert '"nav2_collision_monitor",\n            "collision_monitor"' not in source
    supervisor = (PACKAGE_ROOT / "slam" / "collision_monitor_supervisor.py").read_text(
        encoding="utf-8"
    )
    assert "child.kill()" in supervisor
    assert "PR_SET_PDEATHSIG" in supervisor
    assert "child.send_signal(signal.SIGINT)" not in supervisor


def test_collision_monitor_drain_setting_is_bounded(monkeypatch):
    """排空值即使误配也不能拖过 launch 的正常终止窗口。"""
    monkeypatch.setenv("WAKULA_COLLISION_DRAIN_SECONDS", "invalid")
    assert _drain_seconds() == DEFAULT_DRAIN_SECONDS
    monkeypatch.setenv("WAKULA_COLLISION_DRAIN_SECONDS", "99")
    assert _drain_seconds() == 2.0
    monkeypatch.setenv("WAKULA_COLLISION_DRAIN_SECONDS", "-1")
    assert _drain_seconds() == 0.0


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
    """无效消息与缺失定位 TF 都不能激活 Nav2。"""
    rclpy.init()
    node = Nav2ReadinessMonitor()
    try:
        node._scan_callback(LaserScan())
        node._odom_callback(Odometry())
        node._check_readiness()
        assert node.scan_received
        assert node.odom_received
        assert not node.scan_valid
        assert not node.odom_valid
        assert not node.startup_requested
        assert node._sensor_is_fresh(node.last_scan_time)
        assert node._sensor_is_fresh(node.last_odom_time)
    finally:
        node.destroy_node()
        rclpy.shutdown()

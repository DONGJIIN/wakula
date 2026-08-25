"""Tests for the unified Nav2, SLAM and perception launch integration."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
import rclpy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from lifecycle_msgs.msg import State, Transition

from slam.nav2_readiness_monitor import (
    Nav2ReadinessMonitor,
    slam_transition_for_state,
)
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


def test_nav2_rk3588_budget_keeps_safety_rates_and_bounds_trajectory_samples():
    """性能档只能削减重复计算，不能把局部更新或控制频率降到不可用。"""
    nav2_file = PACKAGE_ROOT / "config" / "nav2.yaml"
    with nav2_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    controller = config["controller_server"]["ros__parameters"]
    dwb = controller["FollowPath"]
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = config["global_costmap"]["global_costmap"]["ros__parameters"]
    planner = config["planner_server"]["ros__parameters"]
    assert controller["controller_frequency"] >= 10.0
    assert local["update_frequency"] >= 5.0
    assert local["publish_frequency"] >= 2.0
    assert global_map["update_frequency"] >= 1.0
    assert global_map["rolling_window"] is True
    # nav2_costmap_2d 在 Jazzy 中把滚动窗口尺寸声明为 integer 参数；YAML 中写成
    # 16.0/8.0 会让 planner_server 在初始化全局代价地图时抛 InvalidParameterTypeException
    # 并 SIGABRT。这里显式约束类型，防止以后“看起来数值相同”的改动再次破坏启动。
    assert type(global_map["width"]) is int
    assert type(global_map["height"]) is int
    assert global_map["width"] >= 14.0
    assert global_map["height"] >= 6.0
    assert planner["expected_planner_frequency"] >= 1.0
    assert planner["GridBased"]["plugin"] == "nav2_navfn_planner::NavfnPlanner"
    assert 150 <= dwb["vx_samples"] * dwb["vtheta_samples"] <= 320
    assert 1.2 <= dwb["sim_time"] <= 2.0


def test_nav2_supports_multi_pose_and_bounded_dead_end_recovery():
    """多目标 Action 必须保留，死路恢复顺序必须有限且从低风险动作开始。"""
    nav2_file = PACKAGE_ROOT / "config" / "nav2.yaml"
    with nav2_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    navigator = config["bt_navigator"]["ros__parameters"]
    assert "navigate_through_poses_w_replanning_and_recovery.xml" in navigator[
        "default_nav_through_poses_bt_xml"
    ]
    tree = (PACKAGE_ROOT / "behavior_trees" / "navigate_to_pose_wakula.xml").read_text(
        encoding="utf-8"
    )
    assert 'number_of_retries="4"' in tree
    assert tree.index("ClearEntireCostmap") < tree.index("<Wait")
    assert tree.index("<Wait") < tree.index("<BackUp") < tree.index("<Spin")


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
    assert readiness["recover_slam_toolbox"] is True
    assert readiness["slam_lifecycle_node"] == "/slam_toolbox"
    assert readiness["slam_recovery_period"] >= 1.0
    assert readiness["slam_recovery_startup_grace"] >= 3.0
    readiness_source = (
        PACKAGE_ROOT / "slam" / "nav2_readiness_monitor.py"
    ).read_text(encoding="utf-8")
    assert "time.monotonic()" in readiness_source


def test_slam_lifecycle_recovery_only_moves_forward_to_active():
    """Recovery must repair startup races without stopping a healthy mapper."""
    assert slam_transition_for_state(State.PRIMARY_STATE_UNCONFIGURED) == (
        Transition.TRANSITION_CONFIGURE
    )
    assert slam_transition_for_state(State.PRIMARY_STATE_INACTIVE) == (
        Transition.TRANSITION_ACTIVATE
    )
    assert slam_transition_for_state(State.PRIMARY_STATE_ACTIVE) is None
    assert slam_transition_for_state(State.PRIMARY_STATE_FINALIZED) is None


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
    assert controller["goal_checker"]["xy_goal_tolerance"] <= 0.10
    assert controller["goal_checker"]["yaw_goal_tolerance"] <= 0.12


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
    assert 0.1 <= params["map_update_interval"] <= 0.25
    # 15 Hz 输入下将扫描匹配限制在约 10 Hz；关键帧足够密，后退和原地旋转不会等到
    # 15 cm / 8.6° 后才更新定位。
    assert 0.08 <= params["minimum_time_interval"] <= 0.12
    assert params["minimum_travel_distance"] <= 0.1
    assert params["minimum_travel_heading"] <= 0.1
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


def test_traversal_handoff_precedes_inflated_costmap_boundary():
    """越障 Action 必须在 DWB 被障碍膨胀层卡住之前取得控制权。"""
    with (PACKAGE_ROOT / "config" / "nav2.yaml").open(encoding="utf-8") as stream:
        nav2 = yaml.safe_load(stream)
    with (
        PACKAGE_ROOT.parent / "quadruped_planning" / "config" /
        "terrain_navigation.yaml"
    ).open(encoding="utf-8") as stream:
        terrain = yaml.safe_load(stream)
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    handoff = terrain["traversal_guidance"]["ros__parameters"]["handoff_distance"]
    hard_stop = terrain["terrain_safety_assessor"]["ros__parameters"][
        "hard_stop_distance"
    ]
    inflated_boundary = (
        local["robot_radius"] + local["inflation_layer"]["inflation_radius"]
    )
    assert handoff == hard_stop
    # 深度点云的障碍前缘会随视角移动数十厘米；仅比静态膨胀边界多 10 cm 仍会让
    # DWB 在任务进入 READY 前停滞。联调要求保留至少 35 cm 动态余量。
    assert handoff >= inflated_boundary + 0.35


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


def test_final_velocity_gate_does_not_depend_on_silent_collision_relay():
    """最终门直接发布 cmd_vel，并保留雷达急停，不能再串联静默断流节点。"""
    path = PACKAGE_ROOT / "launch" / "navigation.launch.py"
    source = path.read_text(encoding="utf-8")
    assert "collision_monitor_supervisor" not in source
    assert '"collision_monitor"' not in source
    terrain_file = PACKAGE_ROOT.parent / "quadruped_planning" / "config" / "terrain_navigation.yaml"
    config = yaml.safe_load(terrain_file.read_text(encoding="utf-8"))
    gate = config["navigation_speed_gate"]["ros__parameters"]
    assert gate["output_topic"] == "/cmd_vel"
    assert gate["require_emergency_scan"] is True
    assert 0.10 <= gate["emergency_stop_distance"] <= 0.40


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


def test_core_entry_auto_detects_clock_without_ros2_daemon_cache(monkeypatch):
    """即使误用核心入口，运行中的 Gazebo /clock 也应自动选择仿真时间和 TF 所有权。"""
    path = PACKAGE_ROOT / "launch" / "slam.launch.py"
    spec = importlib.util.spec_from_file_location("slam_launch_auto", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class Result:
        stdout = "/clock\n/map\n"

    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return Result()

    monkeypatch.setattr(module.subprocess, "run", fake_run)
    assert module._robocon_simulation_is_running()
    assert "--no-daemon" in calls[0][0]
    source = path.read_text(encoding="utf-8")
    assert '"use_sim_time",\n                default_value="auto"' in source
    assert '"robot_model",\n                default_value="auto"' in source


def test_core_entry_retries_transient_clock_discovery_failure(monkeypatch):
    """首次 DDS 查询漏掉 /clock 时不能立即把 Gazebo 当成真机。"""
    path = PACKAGE_ROOT / "launch" / "slam.launch.py"
    spec = importlib.util.spec_from_file_location("slam_launch_retry", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class Result:
        def __init__(self, publishers):
            self.stdout = "/clock\n/map\n" if publishers else "/map\n"

    results = iter([Result(0), Result(1)])
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return next(results)

    monkeypatch.setattr(module.subprocess, "run", fake_run)
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)
    assert module._robocon_simulation_is_running()
    assert len(calls) == 2
    assert all("--no-daemon" in command for command, _kwargs in calls)


def test_simulation_entry_locks_clock_and_tf_ownership():
    """仿真快捷入口必须固定仿真时钟并禁止算法占位 TF，避免看似断流的错配。"""
    path = PACKAGE_ROOT / "launch" / "slam_sim.launch.py"
    spec = importlib.util.spec_from_file_location("slam_sim_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert {
        "sensor_profile",
        "scan_topic",
        "odom_topic",
        "camera_topic",
        "point_cloud_topic",
        "rviz",
    } <= launch_argument_names(description)
    source = path.read_text(encoding="utf-8")
    assert '"use_sim_time": "true"' in source
    assert '"robot_model": "false"' in source
    # 文档字符串可以说明场地入口，但可执行代码不能解析或 include 仿真包。
    assert 'FindPackageShare("quadruped_gazebo")' not in source
    assert 'package="quadruped_gazebo"' not in source


def test_autonomous_entry_keeps_field_separate_and_selects_sim_action_backend():
    """核心入口不创建任务；第三入口仅在仿真时补齐可替换 Action 后端。"""
    main = (PACKAGE_ROOT / "launch" / "slam.launch.py").read_text(encoding="utf-8")
    compatibility = (
        PACKAGE_ROOT / "launch" / "autonomous_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'executable="autonomous_mission"' not in main
    assert '"autonomy_autostart"' not in main
    assert '"mission_params_file"' not in main
    assert 'executable="autonomous_mission"' in compatibility
    assert '"autostart": True' in compatibility
    assert "IncludeLaunchDescription" not in compatibility
    assert 'FindPackageShare("quadruped_gazebo")' not in main
    assert "sim_traverse_obstacle" not in main
    assert 'package="quadruped_gazebo"' in compatibility
    assert 'executable="sim_traverse_obstacle"' in compatibility
    assert '"simulation_traversal_backend"' in compatibility


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

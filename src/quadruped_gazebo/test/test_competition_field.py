"""对比赛场地的关键尺寸、颜色和算法隔离做静态回归检查。

这些测试刻意只锁定规则 V1.0 已经公布的数据。障碍的全局 pose 仍是参考布局，
正式坐标公布后允许修改，不应因此修改 SLAM、Nav2 或 OpenCV 源码。
"""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Twist


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
    mux = (PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py").read_text(encoding="utf-8")
    assert '"/navigation/autonomy_stop"' in mux
    assert "if autonomy_stop:" in mux


def test_sim_traversal_executor_yields_cpu_after_action_completion():
    """The replaceable simulation backend must not starve SLAM health heartbeats."""
    backend = (PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py").read_text(
        encoding="utf-8"
    )
    assert "executor.spin_once(timeout_sec=0.05)" in backend
    assert "time.sleep(0.020)" in backend
    # A/B 尚未由局部视角分清时仍使用统一 STEP 合同，但仿真替身只能跨当前横向结构，
    # 不能错误套用 B 桥全长并移出场地。
    assert '"wooden_bridge_unknown_span", 5.00' in backend
    assert '"wooden_bridge_unknown_duration", 14.0' in backend
    assert '"duration_scale", 0.75' in backend
    assert '"long_structure_exit_clearance", 0.75' in backend


def test_sim_traversal_rejects_large_unrelated_heading_change():
    """无法沿已确认入口安全落地时应重观察，不能横穿场地伪造一次越障。"""
    path = PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    spec = importlib.util.spec_from_file_location("sim_traverse_obstacle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    yaw = module.choose_safe_traversal_heading(
        -5.46, -2.63, -0.69, 7.67, 7.0, 3.0, 0.35
    )
    assert yaw is None
    # 小角度即可避开边界时仍可在对正误差范围内修正。
    yaw = module.choose_safe_traversal_heading(
        4.8, 0.0, 0.0, 1.5, 7.0, 3.0, 0.75, maximum_adjustment=0.35
    )
    assert yaw is not None
    assert abs(yaw) <= 0.35


def test_reference_world_clock_rate_is_bounded_for_algorithm_integration():
    """纯算法联调无需 1 kHz 物理时钟，避免每个仿真时钟节点被过度唤醒。"""
    step_size = float(WORLD.findtext("physics/max_step_size"))
    # 100 Hz 足以支撑测试 IMU，并防止 GUI/RViz 同机时关键导航数据饥饿。
    assert 0.01 <= step_size <= 0.02


def test_rule_dimensions_are_locked():
    # 高墙 1000 × 50 × 300 mm；T 台阶中心台 1000 × 1000 × 400 mm。
    assert_close(collision_box("high_wall", "wall"), [0.05, 1.0, 0.30])
    assert_close(collision_box("t_shaped_stairs", "platform"), [1.0, 1.0, 0.40])
    # 大斜坡 3000 × 2000 mm；桥 A 长条 1500 × 100 mm。
    assert_close(collision_box("main_slope", "ramp"), [3.0, 2.0, 0.08])
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
    """参考布局必须允许逐障碍测试；T 台北缘与桥 B 南缘之间保留机身通道。"""
    stair_y = layout_pose("t_shaped_stairs")[1]
    bridge_y = layout_pose("wooden_bridge_b")[1]
    stair_north = stair_y + 0.50  # T 顶台/横臂的北缘。
    bridge_south = bridge_y - 0.50
    assert bridge_south - stair_north >= 0.50


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


def test_field_launch_routes_one_arbitrated_velocity_to_gazebo():
    """Gazebo bridge 只能接收 mux 输出，避免键盘和 Collision Monitor 相互覆盖。"""
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
    mux_source = (PACKAGE_ROOT / "scripts" / "sim_cmd_vel_mux.py").read_text(
        encoding="utf-8"
    )
    assert "self.autonomous_stamp = None" in mux_source
    assert "self.publisher.publish(Twist())" in mux_source
    # 测试狗没有腿部动力学，第三条自主任务中的仿真 Action 只可通过这一标准
    # Gazebo 服务跨越实体碰撞；场地 launch 本身仍不启动任何越障节点。
    assert "/world/robocon_obstacle_field/set_pose@" in launch_source
    assert "ros_gz_interfaces/srv/SetEntityPose" in launch_source


def test_simulated_traversal_path_is_layout_independent_and_ends_aligned():
    """仿真越障只使用实时起点/航向，不得硬编码八个 world 坐标。"""
    path = PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py"
    spec = importlib.util.spec_from_file_location("sim_traverse_obstacle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    x, y, yaw = module.traversal_pose(1.0, 2.0, 0.0, 3.0, 1.0)
    assert_close([x, y, yaw], [4.0, 2.0, 0.0])
    # 绕杆轨迹中点存在横向位移，但终点回到中心线并恢复原航向。
    middle = module.traversal_pose(1.0, 2.0, 0.0, 3.0, 0.25, pole=True)
    finish = module.traversal_pose(1.0, 2.0, 0.0, 3.0, 1.0, pole=True)
    assert abs(middle[1] - 2.0) > 0.20
    assert_close(list(finish), [4.0, 2.0, 0.0], tolerance=1e-5)
    # L 形坑先沿入口方向走 60%，再右转；终点航向也必须沿第二条臂。
    corner = module.traversal_pose(0.0, 0.0, 0.0, 5.0, 0.60, l_turn=-1)
    l_finish = module.traversal_pose(0.0, 0.0, 0.0, 5.0, 1.0, l_turn=-1)
    assert_close(list(corner), [3.0, 0.0, 0.0], tolerance=1e-5)
    assert_close(list(l_finish), [3.0, -2.0, -math.pi / 2.0], tolerance=1e-5)
    safe_l = module.choose_safe_l_traversal(
        -5.85, -0.57, math.pi / 2.0, 4.33, 7.0, 3.0, 0.75
    )
    assert safe_l is not None
    assert safe_l[1] == -1  # 北向进入后向东（机体右侧）离开参考 L 形坑。
    assert module.pose_inside_arena(0.0, 0.0, 7.0, 3.0, 0.35)
    assert module.pose_inside_arena(6.64, 2.64, 7.0, 3.0, 0.35)
    assert not module.pose_inside_arena(6.66, 0.0, 7.0, 3.0, 0.35)
    assert not module.pose_inside_arena(0.0, -2.66, 7.0, 3.0, 0.35)
    source = path.read_text(encoding="utf-8")
    assert "layout_" not in source
    assert "robocon_obstacle_field.sdf" not in source
    assert "handle.request.distance" in source
    # 简化越障的落点必须同时越过机身半长和 Nav2 inflation layer，不能把下一次
    # 规划的起点留在障碍物致命代价区内。
    assert '"exit_clearance", 1.20' in source
    assert '"right_angle_poles_span", 1.00' in source
    assert '"t_shaped_stairs_span", 2.80' in source
    assert '"wooden_bridge_b_span", 5.70' in source
    assert "+ semantic_span" in source


def test_gui_field_opens_remapped_keyboard_without_loading_algorithms():
    """第一条 GUI 命令应提供人工测试窗口，但不能借机耦合 SLAM 或自主任务。"""
    launch_source = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'package="teleop_twist_keyboard"' in launch_source
    assert '("cmd_vel", "/cmd_vel_teleop")' in launch_source
    assert '"keyboard_teleop"' in launch_source
    assert "gnome-terminal --wait" in launch_source


def test_field_launch_rejects_a_duplicate_named_gazebo_world():
    """重复同名服务会混接机器人/传感器，入口必须在启动前显式拒绝。"""
    launch_source = (PACKAGE_ROOT / "launch" / "robocon_field.launch.py").read_text(
        encoding="utf-8"
    )
    assert "def _reject_duplicate_world" in launch_source
    assert "/world/robocon_obstacle_field/scene/info" in launch_source
    assert "OpaqueFunction(function=_reject_duplicate_world)" in launch_source


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
    for argument in ("robot_sdf", "robot_name", "publish_test_sensor_tf"):
        assert "DeclareLaunchArgument" in source
        assert f'"{argument}"' in source
    assert 'models" / "generic_quadruped"' in source


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

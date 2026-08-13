"""对比赛场地的关键尺寸、颜色和算法隔离做静态回归检查。

这些测试刻意只锁定规则 V1.0 已经公布的数据。障碍的全局 pose 仍是参考布局，
正式坐标公布后允许修改，不应因此修改 SLAM、Nav2 或 OpenCV 源码。
"""

import importlib.util
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


def test_autonomous_field_test_is_explicitly_simulation_only():
    """整场一键联调可包含仿真 Action，但核心算法入口不能反向依赖它。"""
    launch = (PACKAGE_ROOT / "launch" / "autonomous_field_test.launch.py").read_text(
        encoding="utf-8"
    )
    adapter = (PACKAGE_ROOT / "scripts" / "sim_traverse_obstacle.py").read_text(
        encoding="utf-8"
    )
    assert "sim_traverse_obstacle" in launch
    assert "SIMULATION ONLY" in adapter
    assert '"/cmd_vel_teleop"' in adapter


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


def test_simulation_velocity_mux_prioritizes_keyboard_and_stops_stale_input():
    """键盘命令必须覆盖算法零速度，两个来源都断流时必须回到零 Twist。"""
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


def test_generic_rgbd_resolution_is_bounded_for_realtime_integration():
    """测试相机保留可用细节，但不能以无意义的高像素拖慢整套联调。"""
    image = ROBOT.find(".//sensor[@name='rgbd']/camera/image")
    assert image is not None
    width = int(image.findtext("width"))
    height = int(image.findtext("height"))
    assert width >= 320 and height >= 180
    assert width * height <= 150_000


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

"""对比赛场地的关键尺寸、颜色和算法隔离做静态回归检查。

这些测试刻意只锁定规则 V1.0 已经公布的数据。障碍的全局 pose 仍是参考布局，
正式坐标公布后允许修改，不应因此修改 SLAM、Nav2 或 OpenCV 源码。
"""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE_ROOT / "worlds" / "robocon_obstacle_field.sdf"
WORLD = ET.parse(WORLD_PATH).getroot().find("world")
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
    compile(simulation_launch, "robocon_field.launch.py", "exec")

"""独立 Xbox launch 文件的结构回归测试。"""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def test_xbox_launch_starts_exactly_two_nodes():
    """入口必须保持为 joy 驱动加手柄适配器，不能意外带入 SLAM/Nav2。"""
    path = Path(__file__).parents[1] / "launch" / "xbox_teleop.launch.py"
    spec = importlib.util.spec_from_file_location("xbox_teleop_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()

    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert arguments == {
        "device_id",
        "joy_topic",
        "output_topic",
        "autorepeat_rate",
        "config_file",
        "use_sim_time",
    }
    assert sum(isinstance(entity, Node) for entity in description.entities) == 2


def test_xbox_manifest_declares_optional_autonomy_process_dependencies():
    """十字键运行 ros2 launch slam 时，fresh install 必须安装对应 CLI 和包。"""
    manifest = (Path(__file__).parents[1] / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>ros2launch</exec_depend>" in manifest
    assert "<exec_depend>slam</exec_depend>" in manifest

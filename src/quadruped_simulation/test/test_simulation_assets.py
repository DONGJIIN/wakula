"""Static checks keep simulation topics and world assets reproducible."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]


def test_training_world_is_valid_and_contains_required_systems_and_obstacles():
    root = ET.parse(ROOT / "worlds" / "wakula_training.sdf").getroot()
    assert root.tag == "sdf"
    plugins = {plugin.attrib.get("filename") for plugin in root.iter("plugin")}
    assert "gz-sim-physics-system" in plugins
    assert "gz-sim-sensors-system" in plugins
    models = {model.attrib.get("name") for model in root.iter("model")}
    assert {"ground", "step_08", "step_15", "wall_30", "incline"} <= models


def test_bridge_exports_hardware_compatible_default_topics():
    entries = yaml.safe_load((ROOT / "config" / "bridge.yaml").read_text())
    topics = {entry["ros_topic_name"] for entry in entries}
    assert {
        "/clock", "/scan", "/odom", "/tf", "/imu/data",
        "/camera/image_raw", "/camera/depth/points",
    } <= topics
    assert all(entry["direction"] == "GZ_TO_ROS" for entry in entries)

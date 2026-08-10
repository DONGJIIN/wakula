"""Smoke tests for the shared bringup launch description."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument


def test_bringup_launch_description_is_constructible():
    """The common launch entry must remain importable after refactors."""
    path = Path(__file__).parents[1] / "launch" / "bringup.launch.py"
    spec = importlib.util.spec_from_file_location("bringup_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert len(description.entities) >= 9
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {
        "vision_params_file",
        "terrain_params_file",
        "terrain_navigation_params_file",
        "robot_model",
        "camera_topic",
        "point_cloud_topic",
    } <= arguments

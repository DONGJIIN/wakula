"""Tests for the minimal Nav2 and SLAM integration."""

import importlib.util
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def test_local_costmap_fuses_scan_and_depth_points():
    """The local planner must receive both lidar and transformed depth data."""
    with (PACKAGE_ROOT / "config" / "nav2.yaml").open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    obstacle_layer = config["local_costmap"]["local_costmap"]["ros__parameters"][
        "obstacle_layer"
    ]
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

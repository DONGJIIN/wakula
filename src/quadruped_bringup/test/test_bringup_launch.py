"""Smoke tests for the shared bringup launch description."""

import importlib.util
from pathlib import Path


def test_bringup_launch_description_is_constructible():
    """The common launch entry must remain importable after refactors."""
    path = Path(__file__).parents[1] / "launch" / "bringup.launch.py"
    spec = importlib.util.spec_from_file_location("bringup_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert len(description.entities) >= 10

"""Unit tests for terrain and OpenCV evidence fusion."""

from quadruped_planning.obstacle_crossing_manager import (
    apply_visual_assist,
    select_terrain_decision,
    visual_target_in_path,
)


def decide(height=0.0, points=100.0, slope=0.0, roughness=0.0):
    return select_terrain_decision(
        height,
        points,
        slope,
        roughness,
        30,
        0.08,
        0.18,
        0.32,
        0.45,
        0.06,
    )


def test_invalid_terrain_stops():
    assert decide(points=10)[0] == "STOP"
    assert decide(height=float("nan"))[0] == "STOP"


def test_geometry_owns_crossing_mode():
    assert decide(height=0.04)[0] == "WALK"
    assert decide(height=0.10)[0] == "STEP"
    assert decide(height=0.20)[0] == "CLIMB"
    assert decide(height=0.35)[0] == "STOP"


def test_visual_target_requires_area_and_center():
    centered_orange = [0.05, 0.50, 0.50, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
    edge_orange = [0.05, 0.05, 0.50, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
    tiny_blue = [0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.50, 0.50, 0.1, 0.1]

    assert visual_target_in_path(centered_orange, 0.03, 0.20)
    assert not visual_target_in_path(edge_orange, 0.03, 0.20)
    assert not visual_target_in_path(tiny_blue, 0.03, 0.20)


def test_invalid_visual_data_is_ignored():
    assert not visual_target_in_path([], 0.03, 0.20)
    assert not visual_target_in_path([float("nan")] * 10, 0.03, 0.20)


def test_visual_assist_only_slows_clear_terrain():
    walk = ("WALK", "NAVIGATE", 1.0)
    step = ("STEP", "CROSS_STEP", 0.45)

    assert apply_visual_assist(walk, True, 0.35) == (
        "WALK",
        "VERIFY_VISUAL_OBSTACLE_WITH_DEPTH",
        0.35,
    )
    assert apply_visual_assist(walk, False, 0.35) == walk
    assert apply_visual_assist(step, True, 0.35) == step

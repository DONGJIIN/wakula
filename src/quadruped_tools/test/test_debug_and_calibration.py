"""浏览器调试台与离线相机标定工具的确定性测试。"""

from pathlib import Path

import numpy as np
from sensor_msgs.msg import CameraInfo

from quadruped_tools.algorithm_debug_dashboard import (
    DASHBOARD_HTML,
    StreamMetric,
    camera_info_summary,
)
from quadruped_tools.camera_calibrator import (
    calibration_quality,
    robust_keep_indices,
    ros_camera_yaml,
)


def test_stream_metric_reports_frequency_and_timeout_without_ros_clock():
    metric = StreamMetric()
    metric.update(10.0)
    metric.update(10.1)
    metric.update(10.2)
    current = metric.public(10.25)
    assert current["count"] == 3
    assert 9.9 <= current["rate_hz"] <= 10.1
    assert current["healthy"]
    assert not metric.public(13.0)["healthy"]


def test_camera_info_summary_rejects_empty_factory_calibration():
    message = CameraInfo()
    assert not camera_info_summary(message)["calibrated"]
    message.width, message.height = 640, 480
    message.k = [500.0, 0.0, 320.0, 0.0, 501.0, 240.0, 0.0, 0.0, 1.0]
    message.d = [-0.1, 0.02, 0.0, 0.0, 0.0]
    summary = camera_info_summary(message)
    assert summary["calibrated"]
    assert summary["fx"] == 500.0


def test_robust_calibration_filter_removes_large_bad_view():
    kept = robust_keep_indices([0.21, 0.24, 0.23, 0.22, 2.8], maximum_error=1.2)
    assert kept == [0, 1, 2, 3]


def test_calibration_quality_requires_enough_diverse_views():
    passed, warnings = calibration_quality(
        0.3,
        [0.3] * 8,
        [(0.45, 0.45), (0.50, 0.50)] * 4,
        8,
    )
    assert not passed
    assert any("10" in warning for warning in warnings)
    assert any("覆盖" in warning for warning in warnings)
    passed, warnings = calibration_quality(
        0.3,
        [0.3] * 12,
        [(0.1, 0.1), (0.9, 0.1), (0.1, 0.9), (0.9, 0.9)] * 3,
        12,
    )
    assert passed
    assert warnings == []


def test_camera_yaml_is_camera_info_manager_compatible():
    result = {
        "image_size": (640, 480),
        "input_images": 13,
        "detected_images": 13,
        "rms": 0.31,
        "initial_rms": 0.45,
        "camera_matrix": np.array(
            [[500.0, 0.0, 320.0], [0.0, 501.0, 240.0], [0.0, 0.0, 1.0]]
        ),
        "distortion": np.array([-0.1, 0.02, 0.0, 0.0, 0.0]),
        "accepted_images": ["a.png"] * 12,
        "rejected_images": ["bad.png"],
        "per_view_errors": [0.4] * 12,
    }
    output = ros_camera_yaml(result, "front_camera")
    assert output["camera_name"] == "front_camera"
    assert output["camera_matrix"]["data"][0] == 500.0
    assert len(output["projection_matrix"]["data"]) == 12
    assert output["calibration_report"]["accepted_views"] == 12
    assert output["calibration_report"]["undetected_views"] == 0


def test_dashboard_stays_read_only_and_tools_are_installed():
    assert "/api/status" in DASHBOARD_HTML
    assert "/api/snapshot" in DASHBOARD_HTML
    assert "/map.jpg" in DASHBOARD_HTML
    source = (
        Path(__file__).parents[1] / "quadruped_tools" / "algorithm_debug_dashboard.py"
    ).read_text(encoding="utf-8")
    assert "create_publisher" not in source
    assert "ActionClient" not in source
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "algorithm_debug_dashboard" in setup
    assert "camera_calibrator" in setup

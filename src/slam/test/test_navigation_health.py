"""Pure runtime navigation health checks."""

import math
from pathlib import Path

from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import LaserScan
import yaml

from slam.navigation_health_monitor import (
    navigation_failures,
    odometry_is_valid,
    OdometryJumpFilter,
    scan_contract_is_valid,
    scan_is_valid,
    source_stamp_is_current,
    transform_stamp_age_seconds,
    transform_stamp_is_current,
)
from slam.parameter_validation import (
    validate_nav2_readiness_parameters,
    validate_navigation_health_parameters,
)


def _shipped_monitor_parameters(node_name):
    """Load the installed-equivalent source YAML for pure contract tests."""
    path = Path(__file__).parents[1] / "config" / "nav2.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))[node_name]["ros__parameters"]


def test_shipped_navigation_monitor_parameters_are_valid():
    """Keep both launch-time monitor sections synchronized with their startup contracts."""
    validate_navigation_health_parameters(
        _shipped_monitor_parameters("navigation_health_monitor")
    )
    validate_nav2_readiness_parameters(
        _shipped_monitor_parameters("nav2_readiness_monitor")
    )


def test_navigation_contract_reports_all_related_configuration_errors():
    """One failed launch should expose the timeout, ratio, frame, and sample mistakes together."""
    values = _shipped_monitor_parameters("navigation_health_monitor")
    values.update(
        sensor_timeout=-1.0,
        minimum_scan_valid_ratio=0.0,
        minimum_scan_samples=1,
        global_frame="/map",
    )
    with pytest.raises(ValueError) as error:
        validate_navigation_health_parameters(values)
    message = str(error.value)
    assert "sensor_timeout" in message
    assert "minimum_scan_valid_ratio" in message
    assert "minimum_scan_samples" in message
    assert "global_frame" in message


def test_readiness_contract_rejects_bad_topic_and_lifecycle_names():
    """Relative sensor/service names would otherwise leave Nav2 waiting without a clear cause."""
    values = _shipped_monitor_parameters("nav2_readiness_monitor")
    values["scan_topic"] = "scan"
    values["lifecycle_service"] = "/"
    with pytest.raises(ValueError, match="scan_topic.*lifecycle_service"):
        validate_nav2_readiness_parameters(values)


def test_scan_health_accepts_inf_but_rejects_nan_stream():
    """LaserScan 的无回波 Inf 合法，整帧 NaN 非法。"""
    assert scan_is_valid([1.0, math.inf, 2.0, math.inf], 0.75)
    assert not scan_is_valid([math.nan, math.nan, 1.0], 0.60)
    assert not scan_is_valid([0.0, 0.01, math.nan], 0.60, 0.05, 10.0)
    assert scan_is_valid([math.inf, 0.10, 2.0], 0.60, 0.05, 10.0)
    assert not scan_is_valid([0.0, 0.0, math.nan], 0.60, 0.0, 10.0)


def test_scan_contract_rejects_bad_angles_sparse_samples_and_empty_frame():
    """持续发布距离数组不代表该 LaserScan 足以供 SLAM 使用。"""
    scan = LaserScan()
    scan.header.frame_id = "lidar_link"
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.ranges = [2.0] * 720
    scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(scan.ranges) - 1)
    scan.range_min = 0.08
    scan.range_max = 20.0
    assert scan_contract_is_valid(scan, 90, math.pi)

    scan.angle_increment = 0.0
    assert not scan_contract_is_valid(scan, 90, math.pi)
    scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(scan.ranges) - 1)
    scan.header.frame_id = ""
    assert not scan_contract_is_valid(scan, 90, math.pi)
    scan.header.frame_id = "lidar_link"
    scan.ranges = [2.0] * 20
    assert not scan_contract_is_valid(scan, 90, math.pi)


def test_odometry_health_checks_covariance_and_finite_pose():
    """里程计必须具有有限位姿和有界协方差。"""
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.1
    msg.pose.covariance[7] = 0.1
    assert odometry_is_valid(msg, 1.0)
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    assert odometry_is_valid(msg, 1.0, "odom", "base_link")
    msg.child_frame_id = "camera_link"
    assert not odometry_is_valid(msg, 1.0, "odom", "base_link")
    msg.child_frame_id = "base_link"
    msg.pose.covariance[0] = 5.0
    assert not odometry_is_valid(msg, 1.0)
    msg.pose.covariance[0] = 0.1
    msg.pose.pose.orientation.w = 0.0
    assert not odometry_is_valid(msg, 1.0)


def test_sensor_header_age_rejects_replayed_and_future_data():
    """持续重发旧消息不能让导航健康状态保持为真。"""
    assert source_stamp_is_current(99, 500_000_000, 100.0, 1.0, 0.1)
    assert not source_stamp_is_current(0, 0, 100.0, 1.0, 0.1)
    assert not source_stamp_is_current(98, 0, 100.0, 1.0, 0.1)
    assert not source_stamp_is_current(100, 200_000_000, 100.0, 1.0, 0.1)


@pytest.mark.parametrize(
    ("seconds", "nanoseconds", "now", "expected_age", "expected_current"),
    (
        (99, 500_000_000, 100.0, 0.5, True),
        (98, 900_000_000, 100.0, 1.1, False),
        (100, 200_000_000, 100.0, -0.2, False),
        (0, 0, 100.0, None, False),
    ),
)
def test_dynamic_localization_tf_requires_fresh_nonzero_source_stamp(
    seconds, nanoseconds, now, expected_age, expected_current
):
    """缓存中“存在”的冻结/未来/零时间 TF 不能激活或保持 Nav2 健康。"""
    age = transform_stamp_age_seconds(seconds, nanoseconds, now)
    if expected_age is None:
        assert age is None
    else:
        assert age == pytest.approx(expected_age)
    assert transform_stamp_is_current(
        seconds,
        nanoseconds,
        now,
        maximum_age=1.0,
        future_tolerance=0.1,
    ) is expected_current


def test_fault_matrix_covers_dropout_tf_loss_drift_and_recovery():
    """健康矩阵覆盖断流、TF 丢失、跳变和恢复。"""
    _, failures = navigation_failures(False, True, True, False)
    assert failures == ("scan",)
    _, failures = navigation_failures(True, False, False, True)
    assert failures == ("odom", "tf", "odom_jump")
    checks, failures = navigation_failures(True, True, True, False)
    assert all(checks.values()) and not failures


def test_odometry_jump_is_latched_and_invalid_samples_do_not_poison_reference():
    """跳变不能在下一帧漏检，NaN 帧也不能破坏后续恢复判断。"""
    monitor = OdometryJumpFilter(maximum_jump=0.75, recovery_samples=3)
    assert not monitor.update(0.0, 0.0, True)
    assert monitor.update(1.0, 0.0, True)
    assert monitor.update(float("nan"), 0.0, False)
    assert monitor.update(1.1, 0.0, True)
    assert monitor.update(1.2, 0.0, True)
    assert not monitor.update(1.3, 0.0, True)

"""Pure runtime navigation health checks."""

import math

from nav_msgs.msg import Odometry

from slam.navigation_health_monitor import (
    navigation_failures,
    odometry_is_valid,
    OdometryJumpFilter,
    scan_is_valid,
    source_stamp_is_current,
)


def test_scan_health_accepts_inf_but_rejects_nan_stream():
    """LaserScan 的无回波 Inf 合法，整帧 NaN 非法。"""
    assert scan_is_valid([1.0, math.inf, 2.0, math.inf], 0.75)
    assert not scan_is_valid([math.nan, math.nan, 1.0], 0.60)
    assert not scan_is_valid([0.0, 0.01, math.nan], 0.60, 0.05, 10.0)
    assert scan_is_valid([math.inf, 0.10, 2.0], 0.60, 0.05, 10.0)
    assert not scan_is_valid([0.0, 0.0, math.nan], 0.60, 0.0, 10.0)


def test_odometry_health_checks_covariance_and_finite_pose():
    """里程计必须具有有限位姿和有界协方差。"""
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.1
    msg.pose.covariance[7] = 0.1
    assert odometry_is_valid(msg, 1.0)
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

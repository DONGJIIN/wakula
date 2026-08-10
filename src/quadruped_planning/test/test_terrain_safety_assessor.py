"""地形安全评估和 Nav2 速度门的纯逻辑回归测试。"""

from geometry_msgs.msg import Twist
from quadruped_interfaces.msg import FusedObstacle, NavigationSafety

from quadruped_planning.cmd_vel_gate import gated_twist
from quadruped_planning.terrain_safety_assessor import (
    ConservativeAssessmentFilter,
    apply_geometry_classification,
    apply_visual_assist,
    finite_or_zero,
    fused_observation_valid,
    navigation_mode_code,
    nonnegative_finite_or_zero,
    nonnegative_integer_or_zero,
    observation_stamp_is_current,
    select_fused_assessment,
    select_terrain_assessment,
    validate_height_thresholds,
    visual_evidence_in_path,
)


def assess(height=0.0, points=100.0, slope=0.0, roughness=0.0):
    """用生产默认阈值评估一组简化几何输入。"""
    return select_terrain_assessment(
        height, points, slope, roughness, 30, 0.08, 0.18, 0.32, 0.45, 0.06
    )


def test_invalid_terrain_is_fail_closed():
    """稀疏点云、NaN 和非法上限都必须产生 STOP。"""
    assert assess(points=10) == ("STOP", 0.0)
    assert assess(height=float("nan")) == ("STOP", 0.0)
    result = select_terrain_assessment(
        0.0, 100.0, 0.0, 0.0, 30, 0.08, 0.18, 0.32, float("nan"), 0.06
    )
    assert result == ("STOP", 0.0)


def test_height_and_slope_are_classified_without_action_output():
    """高度与坡度只映射为类别和速度上限。"""
    assert assess(height=0.04) == ("WALK", 1.0)
    assert assess(height=0.10) == ("STEP", 0.0)
    assert assess(height=0.20) == ("CLIMB", 0.0)
    assert assess(height=0.35) == ("STOP", 0.0)
    assert assess(slope=-0.50) == ("CLIMB", 0.0)


def test_explicit_geometry_only_changes_navigation_constraint():
    """坑、横杆和立柱只改变导航约束，不生成动作。"""
    assert apply_geometry_classification(assess(), 3, 0.12) == ("STOP", 0.0)
    assert apply_geometry_classification(assess(), 5, 0.0) == ("STOP", 0.0)
    assert apply_geometry_classification(assess(), 6, 0.0) == ("WALK", 0.35)


def test_visual_target_requires_valid_centered_evidence():
    """视觉辅助要求有限、居中且完整的归一化目标框。"""
    assert visual_evidence_in_path(
        [1.0, 0.75, 0.50, 0.50, 0.20, 0.60], 0.55, 0.20
    )
    assert not visual_evidence_in_path(
        [1.0, 0.75, 0.05, 0.50, 0.20, 0.60], 0.55, 0.20
    )
    assert not visual_evidence_in_path([], 0.55, 0.20)
    assert not visual_evidence_in_path([float("nan")] * 6, 0.55, 0.20)


def test_invalid_height_thresholds_restore_safe_defaults():
    """乱序高度阈值恢复为保守默认值。"""
    assert validate_height_thresholds(0.08, 0.18, 0.32) == (0.08, 0.18, 0.32)
    assert validate_height_thresholds(0.30, 0.10, 0.20) == (0.08, 0.18, 0.32)


def test_typed_handoff_codes_and_numeric_sanitization_are_stable():
    """跨团队接口必须保持稳定常量，并阻止非法浮点数泄漏到消费者。"""
    assert navigation_mode_code("WALK") == NavigationSafety.MODE_WALK
    assert navigation_mode_code("STEP") == NavigationSafety.MODE_STEP
    assert navigation_mode_code("CLIMB") == NavigationSafety.MODE_CLIMB
    assert navigation_mode_code("STOP") == NavigationSafety.MODE_STOP
    assert navigation_mode_code("future-mode") == NavigationSafety.MODE_UNKNOWN
    assert finite_or_zero(float("nan")) == 0.0
    assert finite_or_zero(float("inf")) == 0.0
    assert nonnegative_finite_or_zero(-0.2) == 0.0
    assert nonnegative_integer_or_zero(float("nan")) == 0
    assert nonnegative_integer_or_zero(-1.0) == 0
    assert nonnegative_integer_or_zero(42.9) == 42


def test_observation_timestamp_rejects_replay_and_future_clock_errors():
    """重复发布旧数据或错误未来时间戳不能维持可通行状态。"""
    assert observation_stamp_is_current(100.0, 99.5, 0.7, 0.1)
    assert not observation_stamp_is_current(100.0, 0.0, 0.7, 0.1)
    assert not observation_stamp_is_current(100.0, 98.0, 0.7, 0.1)
    assert not observation_stamp_is_current(100.0, 100.2, 0.7, 0.1)
    assert not observation_stamp_is_current(
        100.0, float("nan"), 0.7, 0.1
    )


def test_visual_assist_only_limits_clear_terrain():
    """单目视觉只能降低 WALK 上限，不能改变危险类别。"""
    walk = ("WALK", 1.0)
    stopped_step = ("STEP", 0.0)
    assert apply_visual_assist(walk, True, 0.35) == ("WALK", 0.35)
    assert apply_visual_assist(walk, False, 0.35) == walk
    assert apply_visual_assist(stopped_step, True, 0.35) == stopped_step


def test_fused_observation_is_atomic_and_fail_closed():
    """融合观测缺少几何确认时必须归零。"""
    msg = FusedObstacle()
    msg.geometry_confirmed = True
    msg.confidence = 0.8
    msg.obstacle_type = FusedObstacle.STEP
    msg.obstacle_height = 0.12
    msg.valid_points = 100
    assert fused_observation_valid(msg, 0.25, 30)
    assert select_fused_assessment(
        msg, 0.25, 30, 0.08, 0.18, 0.32, 0.45, 0.06, 0.35
    ) == ("STEP", 0.0)
    msg.geometry_confirmed = False
    assert not fused_observation_valid(msg, 0.25, 30)
    assert select_fused_assessment(
        msg, 0.25, 30, 0.08, 0.18, 0.32, 0.45, 0.06, 0.35
    ) == ("STOP", 0.0)


def test_fused_observation_rejects_partial_or_invalid_contract_data():
    """生产者确认位不能掩盖未知类别、低点数或任一损坏的几何字段。"""
    msg = FusedObstacle()
    msg.geometry_confirmed = True
    msg.obstacle_type = FusedObstacle.CLEAR
    msg.confidence = 0.8
    msg.valid_points = 100
    assert fused_observation_valid(msg, 0.25, 30)
    msg.distance = float("nan")
    assert not fused_observation_valid(msg, 0.25, 30)
    msg.distance = 0.0
    msg.obstacle_type = FusedObstacle.UNKNOWN
    assert not fused_observation_valid(msg, 0.25, 30)


def test_fused_visual_confirmation_only_limits_clear_geometry():
    """同步视觉确认只对 CLEAR/WALK 施加限速。"""
    msg = FusedObstacle()
    msg.geometry_confirmed = True
    msg.vision_confirmed = True
    msg.confidence = 0.8
    msg.obstacle_type = FusedObstacle.CLEAR
    msg.valid_points = 100
    assert select_fused_assessment(
        msg, 0.25, 30, 0.08, 0.18, 0.32, 0.45, 0.06, 0.35
    ) == ("WALK", 0.35)


def test_velocity_gate_requires_fresh_command_and_assessment():
    """命令或评估任一过期/非法都输出零 Twist。"""
    command = Twist()
    command.linear.x = 1.0
    command.angular.z = 0.5
    output = gated_twist(command, 0.4, True, True)
    assert abs(output.linear.x - 0.4) < 1e-6
    assert abs(output.angular.z - 0.2) < 1e-6
    assert gated_twist(command, 1.0, False, True).linear.x == 0.0
    assert gated_twist(command, 1.0, True, False).linear.x == 0.0
    assert gated_twist(command, 1.0, True, True, False, True).linear.x == 0.0
    assert gated_twist(command, 1.0, True, True, True, False).linear.x == 0.0
    assert gated_twist(command, float("nan"), True, True).linear.x == 0.0


def test_filter_confirms_hazard_and_clearance_but_stops_immediately():
    """验证风险迟滞和 STOP 第一帧生效的非对称规则。"""
    filter_ = ConservativeAssessmentFilter(3, ("WALK", 1.0), hazard_frames=2)
    climb = ("CLIMB", 0.0)
    assert filter_.update(climb)[0] == "WALK"
    assert filter_.update(climb)[0] == "CLIMB"
    assert filter_.update(("WALK", 1.0))[0] == "CLIMB"
    assert filter_.update(("WALK", 1.0))[0] == "CLIMB"
    assert filter_.update(("WALK", 1.0))[0] == "WALK"
    assert filter_.update(("STOP", 0.0))[0] == "STOP"

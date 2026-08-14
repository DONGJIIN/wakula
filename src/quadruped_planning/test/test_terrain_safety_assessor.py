"""地形安全评估和 Nav2 速度门的纯逻辑回归测试。"""

from geometry_msgs.msg import Twist
from quadruped_interfaces.msg import FusedObstacle, NavigationSafety
from sensor_msgs.msg import LaserScan

from quadruped_planning.cmd_vel_gate import gated_twist, scan_allows_command
from quadruped_planning.terrain_safety_assessor import (
    ConservativeAssessmentFilter,
    apply_distance_aware_constraint,
    apply_geometry_classification,
    apply_visual_assist,
    finite_or_zero,
    format_front_obstacle_status,
    front_obstacle_name_zh,
    fused_observation_valid,
    navigation_mode_code,
    nonnegative_finite_or_zero,
    nonnegative_integer_or_zero,
    observation_stamp_is_current,
    obstacle_name_zh,
    select_fused_assessment,
    select_terrain_assessment,
    validate_height_thresholds,
    visual_obstacle_name_zh,
    visual_evidence_in_path,
)


def test_front_obstacle_names_and_status_are_human_readable():
    """终端摘要必须直说障碍名称，并在无效数据时避免误报无障碍。"""
    assert obstacle_name_zh(NavigationSafety.OBSTACLE_STEP) == "台阶"
    assert obstacle_name_zh(NavigationSafety.OBSTACLE_CLEAR) == "无障碍"
    assert obstacle_name_zh(NavigationSafety.OBSTACLE_CLEAR, False) == "感知数据无效"
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_BAR
    safety.mode = NavigationSafety.MODE_STOP
    safety.confidence = 0.8
    safety.distance = 0.6
    safety.obstacle_height = 0.3
    text = format_front_obstacle_status(safety)
    assert "限高杆" in text
    assert "模式=STOP" in text
    assert "距离=0.60 m" in text

    safety.obstacle_type = NavigationSafety.OBSTACLE_CLEAR
    safety.mode = NavigationSafety.MODE_WALK
    safety.visual_assist_active = True
    text = format_front_obstacle_status(safety, "横杆")
    assert "疑似横杆" in text
    assert "点云未确认" in text


def test_visual_obstacle_name_requires_valid_integer_code_and_target():
    """视觉中文名不能从无效、未知或非整数类别中猜测。"""
    assert visual_obstacle_name_zh([2.0, 0.8], True) == "横杆"
    assert visual_obstacle_name_zh([2.0, 0.8], False) == ""
    assert visual_obstacle_name_zh([2.4, 0.8], True) == ""
    assert visual_obstacle_name_zh([99.0, 0.8], True) == ""
    assert visual_obstacle_name_zh([], True) == ""


def test_front_name_keeps_visual_hint_without_promoting_it_to_geometry():
    """通用有色视觉候选应显示出来，但不能伪装成已确认几何类别。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_CLEAR
    safety.mode = NavigationSafety.MODE_WALK
    assert front_obstacle_name_zh(safety) == "无障碍"
    name = front_obstacle_name_zh(safety, "有色比赛障碍")
    assert name == "视觉检测到有色比赛障碍（点云待分类）"
    text = format_front_obstacle_status(safety, "有色比赛障碍")
    assert "点云待分类" in text
    assert "未参与限速" in text


def test_front_name_uses_measured_geometry_for_rule_obstacles():
    """规则专名只能由足以区分它们的米制几何产生。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_CLEAR
    safety.slope_pitch = 0.174533  # 规则主斜坡 10°
    assert front_obstacle_name_zh(safety) == "主斜坡（10°坡面）"
    safety.slope_pitch = 0.244346  # 两座木桥的 14° 引坡
    assert "木桥引坡" in front_obstacle_name_zh(safety)
    assert "A/B 待结构确认" in front_obstacle_name_zh(safety)

    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.slope_pitch = 0.0
    safety.obstacle_height = 0.40
    safety.width = 1.0
    assert front_obstacle_name_zh(safety) == "T 字形台阶"
    safety.obstacle_type = NavigationSafety.OBSTACLE_BAR
    assert front_obstacle_name_zh(safety) == "限高杆"
    safety.obstacle_type = NavigationSafety.OBSTACLE_POLE
    safety.obstacle_height = 0.55
    safety.width = 0.12
    assert "直角绕杆" in front_obstacle_name_zh(safety)
    safety.obstacle_type = NavigationSafety.OBSTACLE_PIT
    safety.obstacle_height = 0.15
    safety.width = 0.42
    safety.roughness = 0.047
    assert front_obstacle_name_zh(safety) == "砂砾与碎木坑"
    # 木桥 B 的桥板间隙同样是 PIT 几何，但同时存在规则中的 0.20 m 宽平桥板。
    safety.obstacle_height = 0.20
    safety.pit_depth = 0.20
    safety.width = 0.75
    safety.roughness = 0.07
    assert front_obstacle_name_zh(safety) == "木桥 B（桥板间隙）"
    safety.obstacle_type = NavigationSafety.OBSTACLE_WALL
    assert front_obstacle_name_zh(safety) == "高墙"


def test_front_name_disambiguates_bar_support_and_pit_guardrail():
    """接近阶段只看见支柱/护栏时，也不能把比赛专名说反。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_POLE
    safety.obstacle_height = 0.34
    safety.width = 0.15
    assert front_obstacle_name_zh(safety) == "限高杆（支柱结构）"

    safety.obstacle_type = NavigationSafety.OBSTACLE_BAR
    safety.obstacle_height = 0.25
    safety.width = 0.88
    safety.clearance_height = 0.25
    assert front_obstacle_name_zh(safety) == "坑区护栏（后方地形待确认）"

    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.25
    safety.roughness = 0.057
    safety.width = 0.59
    safety.clearance_height = 0.0
    assert front_obstacle_name_zh(safety) == "砂砾与碎木坑（入口/填料区）"


def test_visual_confirmation_does_not_replace_confirmed_geometry_name():
    """立柱等几何已确认后，视觉只能补充说明，不能把主名称降级为“疑似”。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_POLE
    safety.mode = NavigationSafety.MODE_WALK
    safety.visual_assist_active = True
    text = format_front_obstacle_status(safety, "立柱")
    assert "直角绕杆区（立柱）" in text
    assert "视觉疑似" not in text
    assert "已参与确认=立柱" in text


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


def test_far_traversal_target_can_be_approached_but_near_target_stops():
    """远处越障目标允许 Nav2 接近入口，进入交接区必须保持原危险等级。"""
    stopped = ("STOP", 0.0)
    assert apply_distance_aware_constraint(
        stopped, FusedObstacle.WALL, 2.0, 0.75, 0.25
    ) == ("WALK", 0.25)
    assert apply_distance_aware_constraint(
        stopped, FusedObstacle.WALL, 0.70, 0.75, 0.25
    ) == stopped
    # POLE 是 Nav2 绕杆物体，不是越障控制器交接目标，距离放行函数不应改写它。
    assert apply_distance_aware_constraint(
        stopped, FusedObstacle.POLE, 1.47, 0.75, 0.25
    ) == stopped
    # CLEAR 上的坡度危险没有可信坡脚距离，不能使用实体障碍的远距放行规则。
    assert apply_distance_aware_constraint(
        ("CLIMB", 0.0), FusedObstacle.CLEAR, 2.5, 0.75, 0.25
    ) == ("CLIMB", 0.0)
    assert apply_distance_aware_constraint(
        stopped, FusedObstacle.PIT, float("nan"), 0.75, 0.25
    ) == stopped


def test_fused_far_step_uses_low_speed_window_for_nav2_approach():
    msg = FusedObstacle()
    msg.geometry_confirmed = True
    msg.confidence = 0.8
    msg.obstacle_type = FusedObstacle.STEP
    msg.obstacle_height = 0.12
    msg.distance = 1.5
    msg.valid_points = 100
    assert select_fused_assessment(
        msg, 0.25, 30, 0.08, 0.18, 0.32, 0.45, 0.06, 0.35, 0.75, 0.25
    ) == ("WALK", 0.25)


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
    assert gated_twist(command, 1.0, True, True, True, True, True).linear.x == 0.0


def test_final_velocity_gate_checks_only_the_command_direction():
    """前后扇区和原地旋转急停应独立，不能把远处比赛障碍提前当成绕行目标。"""
    scan = LaserScan()
    scan.angle_min = -3.141592653589793
    scan.angle_increment = 3.141592653589793 / 4.0
    scan.range_min = 0.05
    scan.ranges = [1.0] * 9
    command = Twist()
    command.linear.x = 0.2
    assert scan_allows_command(scan, command, 0.22, 0.60)
    # 正前方索引 4 进入 22 cm，前进必须停车；后退仍可脱离。
    scan.ranges[4] = 0.18
    assert not scan_allows_command(scan, command, 0.22, 0.60)
    command.linear.x = -0.2
    assert scan_allows_command(scan, command, 0.22, 0.60)
    # 后方位于 ±pi 两端；任一端过近时后退被拒绝。
    scan.ranges[0] = 0.16
    assert not scan_allows_command(scan, command, 0.22, 0.60)
    command.linear.x = 0.0
    command.angular.z = 0.4
    assert not scan_allows_command(scan, command, 0.22, 0.60)


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

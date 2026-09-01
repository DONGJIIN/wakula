"""地形安全评估和 Nav2 速度门的纯逻辑回归测试。"""

from geometry_msgs.msg import Twist
from quadruped_interfaces.msg import FusedObstacle, NavigationSafety
from sensor_msgs.msg import LaserScan

from quadruped_planning.cmd_vel_gate import (
    alignment_twist,
    gated_twist,
    has_finite_yaw_request,
    is_pure_rotation_request,
    scan_allows_command,
    twist_components_are_finite,
)
from quadruped_planning.terrain_safety_assessor import (
    ObstacleNameStabilizer,
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
    observation_stamp_strictly_advances,
    obstacle_measurements_are_valid,
    obstacle_name_zh,
    select_fused_assessment,
    select_terrain_assessment,
    validate_height_thresholds,
    visual_obstacle_name_zh,
    visual_evidence_in_path,
)


def test_competition_name_requires_repeated_frames_and_invalid_clears_immediately():
    """类别边界抖动不能刷屏或污染任务投票，断流仍必须立即显式失效。"""
    filter_ = ObstacleNameStabilizer(
        confirmation_frames=3,
        clear_frames=4,
    )
    assert filter_.update("高墙", True) == "感知数据无效"
    assert filter_.update("T 字形台阶", True) == "感知数据无效"
    assert filter_.update("高墙", True) == "感知数据无效"
    assert filter_.update("高墙", True) == "感知数据无效"
    assert filter_.update("高墙", True) == "高墙"

    # 一帧近裁剪误分类不允许覆盖已确认名称。
    assert filter_.update("T 字形台阶", True) == "高墙"
    assert filter_.update("高墙", True) == "高墙"
    # 但感知断流不能因时序滤波继续显示旧障碍。
    assert filter_.update("高墙", False) == "感知数据无效"
    assert filter_.reset() == "感知数据无效"


def test_duplicate_or_out_of_order_fused_stamp_cannot_add_a_temporal_vote():
    """Several recent DDS packets are still only one physical observation."""
    assert observation_stamp_strictly_advances(None, 10.0)
    assert observation_stamp_strictly_advances(10.0, 10.1)
    assert not observation_stamp_strictly_advances(10.0, 10.0)
    assert not observation_stamp_strictly_advances(10.0, 9.9)
    assert not observation_stamp_strictly_advances(10.0, float("nan"))


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


def test_competition_name_fails_closed_on_nonfinite_geometry():
    """消息标为 valid 也不能让 NaN/Inf 进入比赛语义票或 Action 身份链。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_WALL
    safety.obstacle_height = 0.30
    safety.width = 1.00
    assert obstacle_measurements_are_valid(safety)
    assert front_obstacle_name_zh(safety) == "高墙"

    safety.width = float("nan")
    assert not obstacle_measurements_are_valid(safety)
    assert front_obstacle_name_zh(safety) == "感知数据无效"

    safety.width = 1.00
    safety.slope_pitch = float("inf")
    assert not obstacle_measurements_are_valid(safety)
    assert front_obstacle_name_zh(safety, "墙面") == "感知数据无效"


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
    safety.slope_pitch = 0.244346  # 规则木桥 A 的连续 14° 入口坡
    assert front_obstacle_name_zh(safety) == "木桥 A（14°入口坡）"

    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.slope_pitch = 0.0
    safety.obstacle_height = 0.40
    safety.width = 1.0
    assert front_obstacle_name_zh(safety) == "T 字形台阶"
    # 只看到多级踏面时，局部拟合平面会吸收部分总高度；宽度、阶梯趋势和残差仍足以
    # 与平滑斜坡（CLEAR）及坑区（PIT）区分。
    safety.obstacle_height = 0.112
    safety.slope_pitch = 0.189
    safety.roughness = 0.032
    assert front_obstacle_name_zh(safety) == "T 字形台阶"
    # 无闪现全栈回归的规则 T 台入口稳定样本：地面拟合吸收了多数级高，但规则宽度、
    # 16.23° 阶梯总体趋势与离散残差仍共同成立。该样本防止以后把上限退回 15°。
    safety.obstacle_height = 0.081
    safety.slope_pitch = 0.2833
    safety.slope_roll = 0.0005
    safety.roughness = 0.029
    safety.width = 0.998
    assert front_obstacle_name_zh(safety) == "T 字形台阶"
    safety.obstacle_type = NavigationSafety.OBSTACLE_BAR
    assert front_obstacle_name_zh(safety) == "限高杆"
    safety.obstacle_type = NavigationSafety.OBSTACLE_POLE
    safety.obstacle_height = 0.55
    safety.width = 0.12
    assert "直角绕杆" in front_obstacle_name_zh(safety)
    safety.obstacle_type = NavigationSafety.OBSTACLE_PIT
    safety.obstacle_height = 0.15
    safety.pit_depth = 0.10
    safety.slope_pitch = 0.0
    safety.width = 0.42
    safety.roughness = 0.047
    assert front_obstacle_name_zh(safety) == "砂砾与碎木坑"
    # 木桥 B 的桥板间隙同样是 PIT 几何，但同时存在规则中的 0.20 m 宽平桥板。
    safety.obstacle_height = 0.20
    safety.pit_depth = 0.20
    safety.width = 0.75
    safety.roughness = 0.07
    assert front_obstacle_name_zh(safety) == "木桥 B（桥板间隙）"
    # T 台近场的低踏面可能暂时成为 PIT；只有规则坡角/坑深/宽度且横滚较小时恢复专名。
    safety.obstacle_height = 0.08
    safety.pit_depth = 0.28
    safety.slope_pitch = 0.349066
    safety.slope_roll = 0.02
    safety.width = 1.0
    safety.roughness = 0.04
    assert front_obstacle_name_zh(safety) == "T 字形台阶"
    safety.slope_roll = 0.14
    assert front_obstacle_name_zh(safety) != "T 字形台阶"
    safety.slope_roll = 0.0
    safety.obstacle_type = NavigationSafety.OBSTACLE_WALL
    safety.obstacle_height = 0.30
    safety.width = 1.0
    assert front_obstacle_name_zh(safety) == "高墙"

    # 近距离相机只覆盖 T 台一级踏面的一部分，仍应以高度+宽度+粗糙度联合识别。
    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.30
    safety.width = 0.51
    safety.roughness = 0.06
    safety.slope_pitch = 0.0
    assert front_obstacle_name_zh(safety) == "T 字形台阶"


def test_flat_arena_edge_is_not_named_as_the_competition_pit():
    """规则场地边缘的平整负台阶不能为砂砾坑任务增加一次完成计数。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_PIT
    safety.pit_depth = 0.128
    safety.obstacle_height = 0.012
    safety.roughness = 0.028
    safety.width = 1.10
    safety.slope_pitch = 0.0
    assert front_obstacle_name_zh(safety) == "场地边界（禁止越界）"
    # 真实坑入口具有护栏/碎料起伏，仍保留比赛语义。
    safety.obstacle_height = 0.15
    safety.roughness = 0.055
    assert front_obstacle_name_zh(safety) == "砂砾与碎木坑"

    # Gazebo 联合回归的实际量测：护栏/坑深/宽度均吻合规则，即使体素化后的表面
    # 残差只有约 2.5 cm，也必须确认坑区而不是让自动任务永久重试入口。
    safety.obstacle_height = 0.14975
    safety.pit_depth = 0.10094
    safety.roughness = 0.02518
    safety.width = 0.54998
    assert front_obstacle_name_zh(safety) == "砂砾与碎木坑"


def test_tall_platform_with_negative_gaps_is_not_named_as_gravel_pit():
    """木桥/T 台侧视的高平台与负间隙组合必须等待结构确认。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_PIT
    safety.pit_depth = 0.31
    safety.obstacle_height = 0.43
    safety.roughness = 0.08
    safety.width = 1.02
    safety.slope_pitch = 0.0
    assert front_obstacle_name_zh(safety) == "台阶或木桥踏板（待结构确认）"


def test_ambiguous_pit_without_rule_guard_rail_is_not_actionable_gravel():
    """PIT 粗分类若缺少坑区护栏/碎料证据，只能继续观察而不能增加任务计数。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_PIT
    safety.pit_depth = 0.18
    safety.width = 1.0
    safety.slope_pitch = 0.0

    # 高平台侧面会在 PIT/STEP 间跳变；0.24 m 已高于规则坑区 0.15 m 护栏。
    safety.obstacle_height = 0.24
    safety.roughness = 0.06
    assert front_obstacle_name_zh(safety) == "台阶或木桥踏板（待结构确认）"

    # 只有负回波、没有任何正凸起时也不能称作比赛砂砾坑。
    safety.obstacle_height = 0.0
    safety.roughness = 0.08
    assert front_obstacle_name_zh(safety) == "坑洞（结构待确认）"

    # 明确看到规则尺寸附近的护栏、坑深和碎料粗糙度后才给出比赛专名。
    safety.obstacle_height = 0.15
    safety.pit_depth = 0.10
    safety.roughness = 0.055
    assert front_obstacle_name_zh(safety) == "砂砾与碎木坑"


def test_partial_oblique_arena_edge_is_still_boundary():
    """横偏视角只能看到窄边缘时也不能把场外落差当成比赛砂砾坑。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_PIT
    safety.obstacle_height = 0.020
    safety.pit_depth = 0.101
    safety.slope_pitch = 0.0
    safety.roughness = 0.028
    safety.width = 0.437
    assert front_obstacle_name_zh(safety) == "场地边界（禁止越界）"

    # 南侧边缘联调样本：落差接近桥板间隙，但没有任何高于地面的桥板，不能报木桥 B。
    safety.obstacle_height = 0.006
    safety.pit_depth = 0.150
    safety.roughness = 0.030
    safety.width = 0.80
    assert front_obstacle_name_zh(safety) == "场地边界（禁止越界）"

    # 北侧斜视联调样本：深度相机会看到更深的外围参考地面，但仍没有坑区护栏或碎料。
    safety.obstacle_height = 0.0
    safety.pit_depth = 0.205
    safety.roughness = 0.0001
    safety.width = 1.10
    assert front_obstacle_name_zh(safety) == "场地边界（禁止越界）"
    # 靠近边缘时看向更低参考地面会产生远大于真实 0.10 m 坑深的负落差；只要没有
    # 护栏/填料正凸起和粗糙度，仍应判为不可越界边缘。
    safety.pit_depth = 0.48
    assert front_obstacle_name_zh(safety) == "场地边界（禁止越界）"


def test_low_flat_arena_side_is_not_a_high_wall():
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_WALL
    safety.obstacle_height = 0.257
    safety.width = 0.692
    safety.roughness = 0.039
    safety.slope_pitch = 0.0
    assert front_obstacle_name_zh(safety) == "场地边界（禁止越界）"


def test_segmented_bridge_b_is_not_named_as_the_gravel_pit():
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.261
    safety.width = 1.113
    safety.roughness = 0.093
    safety.slope_pitch = 0.0
    assert front_obstacle_name_zh(safety) == "木桥 B（分段桥板）"

    # 无闪现回归中从不与主斜坡重叠的西侧入口观测；0.71 m 是 1 m 桥面被前向
    # ROI 横向裁切后的有效连通宽度，0.083 m 残差来自规则 0.40 m 周期板缝。
    safety.obstacle_height = 0.200
    safety.pit_depth = 0.0
    safety.width = 0.708
    safety.roughness = 0.083
    safety.slope_pitch = 0.0
    assert front_obstacle_name_zh(safety) == "木桥 B（分段桥板）"


def test_close_gravel_fill_is_not_named_as_segmented_bridge_b():
    """Full-field pit fill at 0.070 m roughness must retain the pit semantic."""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.217
    safety.pit_depth = 0.027
    safety.width = 1.102
    safety.roughness = 0.070
    safety.slope_pitch = -0.020
    assert front_obstacle_name_zh(safety) == "砂砾与碎木坑（填料区）"


def test_main_slope_side_is_not_segmented_bridge_b():
    """整场联调实测的主坡侧边不得再次触发长桥 Action。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.2772
    safety.width = 0.8880
    safety.roughness = 0.0590
    safety.slope_pitch = 0.0
    safety.pit_depth = 0.0
    assert "木桥 B" not in front_obstacle_name_zh(safety)


def test_crossbar_vision_preserves_height_bar_during_near_step_crop():
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.34
    safety.width = 1.12
    safety.roughness = 0.05
    assert front_obstacle_name_zh(safety, "横杆") == "限高杆"


def test_overheight_flat_step_requires_another_view_before_t_stair_handoff():
    """0.455 m 平整立面也可能是主坡侧面，不能直接授权 T 台 Action。"""
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.455
    safety.width = 1.12
    safety.roughness = 0.037
    safety.slope_pitch = 0.0
    assert front_obstacle_name_zh(safety) == "台阶或木桥踏板（待结构确认）"


def test_clear_fourteen_degree_ramp_is_bridge_a():
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_CLEAR
    safety.slope_pitch = 0.244
    safety.slope_roll = 0.0
    assert front_obstacle_name_zh(safety) == "木桥 A（14°入口坡）"


def test_field_calibration_disambiguates_slope_side_stairs_and_bridge_entries():
    """Lock the geometry combinations observed in the complete Gazebo field run."""
    safety = NavigationSafety()
    safety.perception_valid = True

    # Looking at the long side of the 10-degree ramp is a tall vertical edge, not
    # a T-shaped stair traversal entry.
    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.473
    safety.width = 1.11
    safety.roughness = 0.058
    safety.slope_pitch = 0.0
    safety.slope_roll = 0.1745
    assert "T 字形" not in front_obstacle_name_zh(safety)

    # 即使侧面只量到 0.40 m（落入普通 T 台高度范围），10° 横向坡仍必须阻止误交接。
    safety.obstacle_height = 0.40
    safety.roughness = 0.042
    assert "T 字形" not in front_obstacle_name_zh(safety)

    # Close T-stair samples can be coarsely labelled PIT after the high tread is
    # absorbed into the ground fit; its ordered 20-degree profile restores semantics.
    safety.obstacle_type = NavigationSafety.OBSTACLE_PIT
    safety.slope_roll = 0.0
    safety.obstacle_height = 0.068
    safety.pit_depth = 0.292
    safety.slope_pitch = 0.352
    safety.roughness = 0.040
    safety.width = 0.93
    assert front_obstacle_name_zh(safety) == "T 字形台阶"

    # Bridge A's partial approach ramp must not look like a flat arena drop.
    safety.obstacle_height = 0.026
    safety.pit_depth = 0.146
    safety.slope_pitch = 0.154
    safety.roughness = 0.019
    safety.width = 1.02
    assert "木桥 A" in front_obstacle_name_zh(safety)

    # A low wall can occlude the floor and transiently look like a deep negative
    # step. Its depth/width/profile must restore wall semantics without world pose.
    safety.obstacle_height = 0.0
    safety.pit_depth = 0.390
    safety.slope_pitch = -0.299
    safety.roughness = 0.047
    safety.width = 0.98
    assert front_obstacle_name_zh(safety) == "高墙（遮挡轮廓）"


def test_field_calibration_disambiguates_bridge_platform_and_pit_guardrail():
    safety = NavigationSafety()
    safety.perception_valid = True
    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.255
    safety.roughness = 0.014
    safety.width = 1.09
    assert "木桥平台" in front_obstacle_name_zh(safety)

    safety.obstacle_type = NavigationSafety.OBSTACLE_WALL
    safety.obstacle_height = 0.18
    safety.roughness = 0.057
    safety.width = 0.60
    assert front_obstacle_name_zh(safety) == "坑区护栏（后方地形待确认）"
    safety.width = 0.98
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
    safety.width = 0.60
    safety.clearance_height = 0.25
    assert front_obstacle_name_zh(safety) == "坑区护栏（后方地形待确认）"

    # 高墙侧视时可能只量到顶部边缘，粗分类会短暂成为 BAR；其连续横宽仍接近 1 m。
    safety.width = 0.98
    assert front_obstacle_name_zh(safety) == "高墙（顶边轮廓）"

    safety.obstacle_type = NavigationSafety.OBSTACLE_STEP
    safety.obstacle_height = 0.18
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
    # 本距离函数只处理实体入口远近，不决定 POLE 任务语义，因此不改写 POLE；普通/矮柱
    # 继续由 Nav2，规则高柱是否进入 Action 由任务层的独立语义和几何闸门决定。
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


def test_velocity_gate_rejects_nonfinite_twist_as_one_atomic_command():
    """任一自由度损坏都必须整条归零，不能把 NaN/Inf 送往底盘或变成限幅 yaw。"""
    command = Twist()
    command.linear.x = 0.25
    command.angular.z = 0.40
    assert twist_components_are_finite(command)

    command.angular.x = float("nan")
    assert not twist_components_are_finite(command)
    output = gated_twist(command, 1.0, True, True)
    assert output.linear.x == 0.0
    assert output.angular.z == 0.0
    assert not is_pure_rotation_request(command)
    assert not has_finite_yaw_request(command)

    # Python 的 min/max 遇到 NaN 可能返回边界值；必须在限幅前拒绝，不能把 NaN yaw
    # 意外转换成允许的最大旋转速度。
    command = Twist()
    command.angular.z = float("nan")
    assert alignment_twist(command, 0.30).angular.z == 0.0
    command.angular.z = float("inf")
    assert alignment_twist(command, 0.30).angular.z == 0.0


def test_alignment_twist_never_preserves_translation():
    """近障碍对正权限只能放行有界 yaw，不能重新放开向前运动。"""
    command = Twist()
    command.linear.x = 0.8
    command.linear.y = -0.2
    command.angular.z = 0.7
    output = alignment_twist(command, 0.3)
    assert output.linear.x == 0.0
    assert output.linear.y == 0.0
    assert output.angular.z == 0.3
    command.angular.z = -0.8
    assert alignment_twist(command, 0.3).angular.z == -0.3
    assert alignment_twist(command, 0.0).angular.z == 0.0


def test_rotation_dominant_dwb_command_can_escape_a_zero_speed_limit():
    command = Twist()
    command.angular.z = 0.5
    assert is_pure_rotation_request(command)
    command.linear.x = 0.10
    assert is_pure_rotation_request(command)
    # The permission classifier may accept DWB's small arc request, but the actual
    # STOP-mode output always drops translation before reaching the robot.
    assert alignment_twist(command, 0.3).linear.x == 0.0
    assert not is_pure_rotation_request(command, 0.02)
    # Return recovery may inspect the yaw from a larger DWB arc, but its eventual
    # output still passes through alignment_twist and therefore remains pure yaw.
    command.linear.x = 0.40
    command.angular.z = -0.61
    assert has_finite_yaw_request(command)
    recovered = alignment_twist(command, 0.30)
    assert recovered.linear.x == 0.0
    assert recovered.angular.z == -0.30
    command.linear.x = 0.0
    command.linear.y = 0.03
    assert not is_pure_rotation_request(command, 0.02)


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
    # A new rosbag/simulator time epoch must not inherit the old STOP vote.
    assert filter_.reset() == ("WALK", 1.0)

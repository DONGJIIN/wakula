"""把感知结果转换为 Nav2 可使用的地形安全等级和速度上限。

本模块位于感知与 Nav2 之间，只回答两个问题：当前地形属于哪一风险等级、导航速度最多
允许保留多少比例。它不发布动作名称，不调用 Action，不生成关节/足端轨迹，也不判断
机器人已经完成越障。所有纯函数同时供在线节点、rosbag 离线评估和单元测试复用。

坐标与单位：高度、粗糙度为米；坡度阈值与旧数组接口一致，表示 ``dz/dx``；速度上限
是 0～1 的无量纲比例。证据不足、字段非法或消息超时均按 STOP 处理。
"""

from math import atan, degrees, isfinite
from typing import Sequence, Tuple

import rclpy
from quadruped_interfaces.msg import FusedObstacle, NavigationSafety
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Header, String


Assessment = Tuple[str, float]
MODE_SEVERITY = {"WALK": 0, "STEP": 1, "CLIMB": 2, "STOP": 3}
MODE_CODES = {
    "WALK": NavigationSafety.MODE_WALK,
    "STEP": NavigationSafety.MODE_STEP,
    "CLIMB": NavigationSafety.MODE_CLIMB,
    "STOP": NavigationSafety.MODE_STOP,
}
MODE_NAMES = {int(code): name for name, code in MODE_CODES.items()}
OBSTACLE_NAMES_ZH = {
    NavigationSafety.OBSTACLE_UNKNOWN: "未知障碍",
    NavigationSafety.OBSTACLE_CLEAR: "无障碍",
    NavigationSafety.OBSTACLE_STEP: "台阶",
    NavigationSafety.OBSTACLE_PIT: "坑洞",
    NavigationSafety.OBSTACLE_WALL: "墙面",
    NavigationSafety.OBSTACLE_BAR: "横杆",
    NavigationSafety.OBSTACLE_POLE: "立柱",
}
# OpenCV 轻量检测器的数组类别。视觉结果只能作为“疑似”提示和保守限速依据；真正的
# 地形类别仍由带尺度信息的点云几何确认，避免单目颜色/轮廓误检直接触发越障决策。
VISION_NAMES_ZH = {
    1: "立柱",
    2: "横杆",
    3: "墙面",
    4: "有色比赛障碍",
}

# ``/terrain/features`` 是旧 rosbag 兼容接口。下标集中在这里，避免回调中出现难以审查
# 的魔法数字；新代码优先读取带 Header 的 FusedObstacle。
TERRAIN_OBSTACLE_HEIGHT = 2
TERRAIN_VALID_POINTS = 3
TERRAIN_GROUND_SLOPE = 4
TERRAIN_ROUGHNESS = 5
TERRAIN_FRONTAL_HEIGHT = 6
TERRAIN_LOOKAHEAD = 7
TERRAIN_PIT_DEPTH = 9
TERRAIN_SLOPE_ROLL = 10
TERRAIN_OBSTACLE_TYPE = 11
TERRAIN_CONFIDENCE = 12
TERRAIN_WIDTH = 13
TERRAIN_CLEARANCE_HEIGHT = 14

# 与 TerrainFeatures/FusedObstacle 的类别常量一致。
GEOMETRY_CLEAR, GEOMETRY_STEP, GEOMETRY_PIT = 1, 2, 3
GEOMETRY_WALL, GEOMETRY_BAR, GEOMETRY_POLE = 4, 5, 6


class ConservativeAssessmentFilter:
    """对风险升级和风险解除使用非对称时间迟滞。

    STOP 必须在第一帧立即生效；STEP/CLIMB 要求少量连续证据以抑制深度飞点；从危险
    状态恢复 WALK 则要求更多连续安全帧。过滤对象只有类别与速度上限，不含动作语义。
    """

    def __init__(
        self, clear_frames: int, initial: Assessment, hazard_frames: int = 2
    ):
        """初始化非对称迟滞；恢复所需帧数通常大于危险确认帧数。"""
        self.clear_frames = max(1, int(clear_frames))
        self.hazard_frames = max(1, int(hazard_frames))
        self.current = initial
        self.pending_mode = None
        self.pending_count = 0

    def update(self, candidate: Assessment) -> Assessment:
        """返回经过连续帧确认后的稳定评估。"""
        current_level = MODE_SEVERITY.get(self.current[0], MODE_SEVERITY["STOP"])
        candidate_level = MODE_SEVERITY.get(candidate[0], MODE_SEVERITY["STOP"])
        if candidate_level == current_level:
            self.current = candidate
            self.pending_mode = None
            self.pending_count = 0
            return self.current

        if candidate_level > current_level:
            # STOP 不延迟；较低危险等级等待连续帧，避免单点噪声造成模式闪烁。
            if candidate[0] == "STOP" or self.hazard_frames == 1:
                self.current = candidate
                self.pending_mode = None
                self.pending_count = 0
                return self.current
            required = self.hazard_frames
        else:
            required = self.clear_frames

        if candidate[0] != self.pending_mode:
            self.pending_mode = candidate[0]
            self.pending_count = 1
        else:
            self.pending_count += 1
        if self.pending_count >= required:
            self.current = candidate
            self.pending_mode = None
            self.pending_count = 0
        return self.current


def validate_height_thresholds(
    step: float, climb: float, stop: float
) -> Tuple[float, float, float]:
    """验证米制高度阈值严格递增，否则恢复保守初值。"""
    values = (step, climb, stop)
    if all(isfinite(value) for value in values) and 0.0 <= step < climb < stop:
        return values
    return 0.08, 0.18, 0.32


def navigation_mode_code(mode: str) -> int:
    """把可读字符串模式映射为稳定消息常量，未知值保持 UNKNOWN。"""
    return int(MODE_CODES.get(mode, NavigationSafety.MODE_UNKNOWN))


def obstacle_name_zh(obstacle_type: int, perception_valid: bool = True) -> str:
    """把稳定接口编码转换为终端和轻量 UI 共用的中文名称。"""
    if not perception_valid:
        return "感知数据无效"
    return OBSTACLE_NAMES_ZH.get(int(obstacle_type), "未知障碍")


def visual_obstacle_name_zh(data: Sequence[float], target_valid: bool) -> str:
    """从旧视觉数组安全读取类别；无效、非整数或未知编码均不生成误导性名称。"""
    if not target_valid or not data or not isfinite(float(data[0])):
        return ""
    code = int(round(float(data[0])))
    if abs(float(data[0]) - code) > 1e-3:
        return ""
    return VISION_NAMES_ZH.get(code, "")


def front_obstacle_name_zh(
    safety: NavigationSafety, visual_hint_name: str = ""
) -> str:
    """生成比赛现场使用的正前方名称，同时保持结论诚实可追溯。

    ``obstacle_type`` 是通用几何类别，不能完整表达规则中的八个专名。这里仅做不影响
    控制的显示细分：杆、坑、限高杆和高墙可由几何直接对应；坡面利用点云实测坡角
    区分规则中的 10° 主斜坡和 14° 木桥引坡。木桥 A/B 的桥面结构与普通台阶若只看到
    一小块局部点云无法可靠区分，因此明确显示“待结构确认”，绝不读取 Gazebo pose
    或把单帧橙色误当成确定类别。
    """
    if not safety.perception_valid:
        return "感知数据无效"

    obstacle_type = int(safety.obstacle_type)
    if obstacle_type == NavigationSafety.OBSTACLE_POLE:
        # 限高杆的横梁很细，单帧深度云常先只看到一侧 0.32 m 支柱而归入 POLE；规则
        # 绕杆立柱高度不低于 0.50 m。用量测高度给出接近阶段名称，比直接误报绕杆可靠。
        if (
            0.25 <= float(safety.obstacle_height) <= 0.42
            and float(safety.width) <= 0.22
        ):
            return "限高杆（支柱结构）"
        return "直角绕杆区（立柱）"
    if obstacle_type == NavigationSafety.OBSTACLE_PIT:
        # 木桥 B 的 0.40 m 周期性板间隙也会形成真实负高度回波。若同时看到较宽、
        # 约 0.20 m 高且表面较平整的桥板，就不能把它叫作砂砾坑。阈值来自规则尺寸，
        # 并已用 Gazebo 点云联调；真机仍需用 rosbag 校准，而不是读取场地坐标。
        if (
            float(safety.obstacle_height) >= 0.17
            and float(safety.pit_depth) >= 0.15
            and float(safety.width) >= 0.75
        ):
            return "木桥 B（桥板间隙）"
        return "砂砾与碎木坑"
    if obstacle_type == NavigationSafety.OBSTACLE_BAR:
        # 坑区 0.15 m 护栏在斜视点云中可能产生悬空外观；高度明显低于 0.30 m
        # 限高横杆时只标为坑区入口线索，等待后续负高度回波确认。
        if (
            float(safety.obstacle_height) < 0.28
            and float(safety.clearance_height) >= 0.15
        ):
            return "坑区护栏（后方地形待确认）"
        return "限高杆"
    if obstacle_type == NavigationSafety.OBSTACLE_WALL:
        return "高墙"
    if obstacle_type == NavigationSafety.OBSTACLE_STEP:
        # 进入砂砾/碎木区后，护栏与低洼填料会在单帧栅格中表现成约 0.15～0.25 m
        # 的低台阶，同时粗糙度显著升高。该组合接续上面的“坑区护栏”接近提示。
        if (
            0.12 <= float(safety.obstacle_height) <= 0.28
            and float(safety.roughness) >= 0.05
            and float(safety.width) >= 0.40
        ):
            return "砂砾与碎木坑（入口/填料区）"
        # 高墙已有独立 WALL 分支。正对 T 台时，近场平面会穿过多级踏面，使“相对平面
        # 高度”小于总高；但它仍表现为宽障碍、7～15° 的阶梯总体趋势和明显离散残差。
        # 同时支持直接看到 0.40 m 顶部与只看到多级踏面的两种距离，避免再误叫坑区。
        stepped_profile = (
            7.0 <= abs(degrees(float(safety.slope_pitch))) <= 15.0
            and float(safety.roughness) >= 0.02
        )
        if float(safety.width) >= 0.60 and (
            float(safety.obstacle_height) >= 0.32 or stepped_profile
        ):
            return "T 字形台阶"
        return "台阶或木桥踏板（待结构确认）"

    if obstacle_type == NavigationSafety.OBSTACLE_CLEAR:
        pitch_degrees = abs(degrees(float(safety.slope_pitch)))
        roll_degrees = abs(degrees(float(safety.slope_roll)))
        # 只在横滚较小时把前后坡度解释成赛道坡面；侧向倾斜仍保留通用地形名称。
        if roll_degrees <= 6.0 and 7.0 <= pitch_degrees <= 12.0:
            return "主斜坡（10°坡面）"
        if roll_degrees <= 6.0 and 12.0 < pitch_degrees <= 17.0:
            return "木桥引坡（14°，A/B 待结构确认）"
        # 有色视觉候选没有尺度，不能改变速度或声称具体障碍；但 UI 也不能把它吞掉后
        # 错报“无障碍”。名称明确表达点云尚未完成结构分类。
        if visual_hint_name:
            return f"视觉检测到{visual_hint_name}（点云待分类）"
        return "无障碍"
    return obstacle_name_zh(obstacle_type, True)


def format_front_obstacle_status(
    safety: NavigationSafety, visual_hint_name: str = ""
) -> str:
    """生成一行可读状态，明确这是前向 ROI 的融合判断而非全场物体列表。"""
    name = front_obstacle_name_zh(safety, visual_hint_name)
    mode = MODE_NAMES.get(int(safety.mode), "UNKNOWN")
    geometry_is_clear = (
        int(safety.obstacle_type) == NavigationSafety.OBSTACLE_CLEAR
    )
    if safety.visual_assist_active and visual_hint_name and geometry_is_clear:
        # 主名称直接回答“正前方是什么”；括号明确它尚不是点云尺度结论。
        name = f"视觉疑似{visual_hint_name}（点云未确认）"
        vision = "已介入限速"
    elif safety.visual_assist_active and visual_hint_name:
        # 点云已有几何类别时不得再把主名称降级为“视觉疑似”。
        vision = f"已参与确认={visual_hint_name}"
    elif safety.visual_assist_active:
        vision = "有疑似障碍（点云未确认，已限速）"
    elif visual_hint_name:
        # 例如通用有色区域：保留给人看的线索，但不伪装成已影响安全链的证据。
        vision = f"仅提示：{visual_hint_name}（未参与限速）"
    else:
        vision = "未介入"
    return (
        f"[正前方障碍] {name} | 模式={mode} | 限速={safety.speed_limit:.2f} | "
        f"置信度={safety.confidence:.2f} | 距离={safety.distance:.2f} m | "
        f"高度={safety.obstacle_height:.2f} m | 视觉辅助={vision}"
    )


def finite_or_zero(value: float) -> float:
    """将接口边界处的 NaN/Inf 收敛为零，避免污染下游控制或日志。"""
    numeric = float(value)
    return numeric if isfinite(numeric) else 0.0


def nonnegative_finite_or_zero(value: float) -> float:
    """清理必须非负的置信度、长度和粗糙度字段。"""
    return max(0.0, finite_or_zero(value))


def nonnegative_integer_or_zero(value: float) -> int:
    """安全转换点数；旧 rosbag 中的 NaN/Inf/负数均视为无有效点。"""
    numeric = float(value)
    return int(numeric) if isfinite(numeric) and numeric >= 0.0 else 0


def observation_stamp_is_current(
    now_seconds: float,
    stamp_seconds: float,
    maximum_age: float,
    future_tolerance: float = 0.10,
) -> bool:
    """拒绝零时间戳、陈旧重放帧和明显来自未来的观测。"""
    values = (now_seconds, stamp_seconds, maximum_age, future_tolerance)
    if not all(isfinite(float(value)) for value in values):
        return False
    age = float(now_seconds) - float(stamp_seconds)
    return (
        stamp_seconds > 0.0
        and maximum_age > 0.0
        and -max(0.0, future_tolerance) <= age <= maximum_age
    )


def select_terrain_assessment(
    obstacle_height: float,
    points: float,
    slope: float,
    roughness: float,
    min_points: int,
    step_threshold: float,
    climb_threshold: float,
    stop_threshold: float,
    max_slope: float,
    max_roughness: float,
) -> Assessment:
    """从一帧几何摘要返回 ``(地形模式, Nav2 速度上限)``。

    判定从最高风险向下执行，防止高墙先命中较低的 STEP 阈值。STEP/CLIMB 在当前阶段
    只是地形类别，速度上限为零表示交给 Nav2 重规划或等待人工分析，并非动作请求。
    """
    values = (
        obstacle_height,
        points,
        slope,
        roughness,
        step_threshold,
        climb_threshold,
        stop_threshold,
        max_slope,
        max_roughness,
    )
    limits_invalid = (
        not 0.0 <= step_threshold < climb_threshold < stop_threshold
        or max_slope <= 0.0
        or max_roughness <= 0.0
        or min_points < 1
    )
    if (
        not all(isfinite(value) for value in values)
        or limits_invalid
        or points < min_points
    ):
        return "STOP", 0.0
    absolute_slope = abs(slope)
    if obstacle_height >= stop_threshold or absolute_slope >= max_slope * 1.5:
        return "STOP", 0.0
    if obstacle_height >= climb_threshold or absolute_slope >= max_slope:
        return "CLIMB", 0.0
    if obstacle_height >= step_threshold or roughness >= max_roughness:
        return "STEP", 0.0
    return "WALK", 1.0


def visual_evidence_in_path(
    evidence: Sequence[float], min_confidence: float, center_margin: float
) -> bool:
    """检查已稳定视觉框是否位于前向图像通道。

    后四个字段均为 0～1 的归一化图像量，不用于推断米制距离或障碍高度。
    """
    if len(evidence) < 6 or not all(
        isfinite(float(value)) for value in evidence[:6]
    ):
        return False
    type_code, confidence, center_x, center_y, width, height = map(
        float, evidence[:6]
    )
    margin = max(0.0, min(0.49, center_margin))
    rounded_code = round(type_code)
    known_type = 1 <= rounded_code <= 4 and abs(type_code - rounded_code) < 1e-3
    return (
        known_type
        and min_confidence <= confidence <= 1.0
        and margin <= center_x <= 1.0 - margin
        and 0.0 <= center_y <= 1.0
        and 0.0 < width <= 1.0
        and 0.0 < height <= 1.0
    )


def apply_visual_assist(
    assessment: Assessment, visual_active: bool, vision_speed_scale: float
) -> Assessment:
    """视觉只能限制 WALK 的速度，不能批准或执行越障。"""
    mode, speed = assessment
    if mode != "WALK" or not visual_active:
        return assessment
    return mode, min(speed, max(0.0, min(1.0, vision_speed_scale)))


def apply_geometry_classification(
    assessment: Assessment, obstacle_type: int, pit_depth: float
) -> Assessment:
    """将显式几何类别叠加到连续量阈值结果。

    坑洞、墙和横杆需要未来越障控制器接管，因此在交接区速度上限为零。立柱本身不是
    要踩过的表面：比赛绕杆区仍由 Nav2 在立柱之间规划，采用保守低速。这里仍只输出
    导航约束，不发出任何动作指令。
    """
    if obstacle_type == GEOMETRY_PIT and pit_depth > 0.0:
        return "STOP", 0.0
    if obstacle_type in (GEOMETRY_WALL, GEOMETRY_BAR):
        return "STOP", 0.0
    if obstacle_type == GEOMETRY_POLE:
        # 高立柱可能先被高度阈值评成 STOP。显式 POLE 类别优先：保留低速 Nav2 绕杆，
        # 具体碰撞边界仍由 scan/costmap 保证，而不是在两米外冻结整机。
        return "WALK", 0.35
    return assessment


def apply_distance_aware_constraint(
    assessment: Assessment,
    obstacle_type: int,
    distance: float,
    hard_stop_distance: float,
    approach_speed: float,
) -> Assessment:
    """让 Nav2 低速接近越障入口，并在交接距离内停住。

    旧策略只要在 2.5 m ROI 内看到台阶、坑、墙或横杆就立刻输出零速度，Nav2 无法到达
    入口。对需要越障接管的明确几何类别，距离大于交接区时保留低速 ``WALK`` 窗口，
    让 Nav2 执行入口目标和姿态对正；进入交接区后恢复 STEP/CLIMB/STOP，等待未来运动
    控制器。这里不会让 Nav2 把终点直接规划到实体障碍后方。

    ``CLEAR`` 坡面不使用这条放行规则：坡度估计的 distance 不是坡脚距离，贸然放行会
    让尚无腿部控制器的机器人直接驶上坡。无效、零或负距离也继续 fail-closed。
    """
    mode, speed = assessment
    explicit_hazards = (
        GEOMETRY_STEP,
        GEOMETRY_PIT,
        GEOMETRY_WALL,
        GEOMETRY_BAR,
    )
    values = (distance, hard_stop_distance, approach_speed)
    if (
        obstacle_type in explicit_hazards
        and mode in ("STEP", "CLIMB", "STOP")
        and all(isfinite(float(value)) for value in values)
        and hard_stop_distance > 0.0
        and distance > hard_stop_distance
    ):
        return "WALK", min(1.0, max(0.0, approach_speed))
    return mode, speed


def fused_observation_valid(
    msg: FusedObstacle, min_confidence: float, min_points: int
) -> bool:
    """验证融合消息能否作为跨模块的一帧完整观测。

    ``geometry_confirmed`` 只是生产者声明；消费者还要独立检查类别、置信度、点数和所有
    连续量，防止部分损坏的 DDS/rosbag 数据被标为有效。
    """
    metrics = (
        msg.confidence,
        msg.obstacle_height,
        msg.pit_depth,
        msg.slope_pitch,
        msg.slope_roll,
        msg.roughness,
        msg.distance,
        msg.lateral_offset,
        msg.width,
        msg.clearance_height,
    )
    confidence_limit = max(0.0, min(1.0, float(min_confidence)))
    return (
        bool(msg.geometry_confirmed)
        and GEOMETRY_CLEAR <= int(msg.obstacle_type) <= GEOMETRY_POLE
        and all(isfinite(float(value)) for value in metrics)
        and confidence_limit <= float(msg.confidence) <= 1.0
        and float(msg.obstacle_height) >= 0.0
        and float(msg.pit_depth) >= 0.0
        and float(msg.roughness) >= 0.0
        and float(msg.distance) >= 0.0
        and float(msg.width) >= 0.0
        and float(msg.clearance_height) >= 0.0
        and int(msg.valid_points) >= max(1, int(min_points))
    )


def select_fused_assessment(
    msg: FusedObstacle,
    min_confidence: float,
    min_points: int,
    step_threshold: float,
    climb_threshold: float,
    stop_threshold: float,
    max_slope: float,
    max_roughness: float,
    vision_speed_scale: float,
    hard_stop_distance: float = 0.90,
    hazard_approach_speed: float = 0.25,
) -> Assessment:
    """从一条时间同步融合消息生成原子导航评估。"""
    if not fused_observation_valid(msg, min_confidence, min_points):
        return "STOP", 0.0
    assessment = select_terrain_assessment(
        float(msg.obstacle_height),
        float(msg.valid_points),
        max(abs(float(msg.slope_pitch)), abs(float(msg.slope_roll))),
        float(msg.roughness),
        min_points,
        step_threshold,
        climb_threshold,
        stop_threshold,
        max_slope,
        max_roughness,
    )
    assessment = apply_geometry_classification(
        assessment, int(msg.obstacle_type), float(msg.pit_depth)
    )
    assessment = apply_distance_aware_constraint(
        assessment,
        int(msg.obstacle_type),
        float(msg.distance),
        hard_stop_distance,
        hazard_approach_speed,
    )
    return apply_visual_assist(
        assessment, bool(msg.vision_confirmed), vision_speed_scale
    )


class TerrainSafetyAssessor(Node):
    """持续发布地形模式和 Nav2 速度上限，并监控感知心跳。"""

    def __init__(self):
        """加载安全阈值并连接强类型融合接口与旧数组兼容接口。

        ``prefer_fused_obstacle`` 为真时融合消息是唯一权威输入；旧数组仍被订阅只是为了
        兼容关闭视觉的点云路径，不能让两条路径同时争抢当前状态。
        """
        super().__init__("terrain_safety_assessor")
        for name, default in (
            ("step_threshold", 0.08),
            ("climb_threshold", 0.18),
            ("stop_threshold", 0.32),
            ("max_slope", 0.45),
            ("max_roughness", 0.06),
            # 全栈同机运行时点云几何会有短时计算抖动。3.0 s 可容纳实测约 2.7 s 的极端
            # 帧间隔；速度门仍以 0.7 s 独立监控输入，控制安全响应没有被这项 UI/决策
            # 状态容错放慢，真正断流最终保持 fail-closed STOP。
            ("sensor_timeout", 3.0),
            ("fused_min_confidence", 0.25),
            ("vision_timeout", 0.6),
            ("vision_min_confidence", 0.55),
            ("vision_center_margin", 0.20),
            ("vision_speed_scale", 0.35),
            ("hard_stop_distance", 0.90),
            ("hazard_approach_speed", 0.25),
            ("future_stamp_tolerance", 0.10),
            ("status_log_period", 1.0),
        ):
            self.declare_parameter(name, default)
        self.declare_parameter("min_points", 30)
        self.declare_parameter("prefer_fused_obstacle", True)
        self.declare_parameter("clear_confirmation_frames", 5)
        self.declare_parameter("hazard_confirmation_frames", 3)
        self.declare_parameter("vision_assist_enabled", True)
        self.declare_parameter("output_frame", "base_link")

        configured_thresholds = tuple(
            float(self.get_parameter(name).value)
            for name in ("step_threshold", "climb_threshold", "stop_threshold")
        )
        self.step_threshold, self.climb_threshold, self.stop_threshold = (
            validate_height_thresholds(*configured_thresholds)
        )
        if configured_thresholds != (
            self.step_threshold,
            self.climb_threshold,
            self.stop_threshold,
        ):
            self.get_logger().warning(
                "Invalid height thresholds; restored 0.08/0.18/0.32 m"
            )
        self.max_slope = self._positive_parameter("max_slope", 0.45)
        self.max_roughness = self._positive_parameter("max_roughness", 0.06)
        self.sensor_timeout = self._positive_parameter("sensor_timeout", 3.0)
        self.future_stamp_tolerance = self._positive_parameter(
            "future_stamp_tolerance", 0.10
        )
        self.min_points = max(1, int(self.get_parameter("min_points").value))
        self.prefer_fused = bool(
            self.get_parameter("prefer_fused_obstacle").value
        )
        self.fused_min_confidence = self._unit_parameter(
            "fused_min_confidence"
        )
        self.vision_enabled = bool(
            self.get_parameter("vision_assist_enabled").value
        )
        self.vision_timeout = self._positive_parameter("vision_timeout", 0.6)
        self.vision_min_confidence = self._unit_parameter(
            "vision_min_confidence"
        )
        self.vision_center_margin = float(
            self.get_parameter("vision_center_margin").value
        )
        self.vision_speed_scale = self._unit_parameter("vision_speed_scale")
        self.hard_stop_distance = self._positive_parameter(
            "hard_stop_distance", 0.90
        )
        self.hazard_approach_speed = self._unit_parameter(
            "hazard_approach_speed"
        )
        self.output_frame = (
            str(self.get_parameter("output_frame").value) or "base_link"
        )
        self.status_log_period = self._positive_parameter("status_log_period", 1.0)
        self.assessment_filter = ConservativeAssessmentFilter(
            int(self.get_parameter("clear_confirmation_frames").value),
            ("STOP", 0.0),
            int(self.get_parameter("hazard_confirmation_frames").value),
        )

        self.mode_pub = self.create_publisher(
            String, "/terrain/navigation_mode", 10
        )
        self.speed_pub = self.create_publisher(Float32, "/terrain/speed_limit", 10)
        self.visual_active_pub = self.create_publisher(
            Bool, "/terrain/visual_assist_active", 10
        )
        # 未来运动控制/硬件团队应优先订阅这个原子只读接口，而不是在不同时间分别读取
        # mode、speed_limit 和 FusedObstacle 后自行拼接。
        self.navigation_safety_pub = self.create_publisher(
            NavigationSafety, "/terrain/navigation_safety", 10
        )
        self.front_obstacle_name_pub = self.create_publisher(
            String, "/perception/front_obstacle_name", 10
        )
        self.create_subscription(
            Float32MultiArray, "/terrain/features", self.features_callback, 10
        )
        self.create_subscription(
            Float32MultiArray,
            "/vision/obstacle_evidence",
            self.vision_callback,
            10,
        )
        self.create_subscription(
            FusedObstacle,
            "/perception/fused_obstacle",
            self.fused_callback,
            10,
        )
        self.last_features_time = self.get_clock().now()
        self.last_vision_time = None
        self.visual_target = False
        self.visual_hint_name = ""
        self.visual_assist_active = False
        self.perception_valid = False
        self.latest_observation = None
        self.last_assessment = None
        self.last_obstacle_status = None
        self.last_status_log_time = None
        self.create_timer(0.1, self.timeout_callback)
        self.publish_assessment("STOP", 0.0)
        self.get_logger().info("Terrain navigation safety assessor ready")

    def _positive_parameter(self, name: str, fallback: float) -> float:
        """读取有限正参数，拒绝会破坏安全比较的 NaN、Inf 和非正值。"""
        value = float(self.get_parameter(name).value)
        return value if isfinite(value) and value > 0.0 else fallback

    def _unit_parameter(self, name: str) -> float:
        """读取并夹紧 0～1 的无量纲参数。"""
        value = float(self.get_parameter(name).value)
        return max(0.0, min(1.0, value)) if isfinite(value) else 0.0

    def features_callback(self, msg: Float32MultiArray) -> None:
        """处理无相机模式下的旧数组特征，保留既有 rosbag 可回放性。"""
        if self.prefer_fused:
            return
        if len(msg.data) < 4:
            self._publish_candidate(("STOP", 0.0))
            return
        self.last_features_time = self.get_clock().now()
        height_index = (
            TERRAIN_FRONTAL_HEIGHT
            if len(msg.data) > TERRAIN_FRONTAL_HEIGHT
            else TERRAIN_OBSTACLE_HEIGHT
        )
        obstacle_type = (
            int(round(msg.data[TERRAIN_OBSTACLE_TYPE]))
            if len(msg.data) > TERRAIN_OBSTACLE_TYPE
            and isfinite(float(msg.data[TERRAIN_OBSTACLE_TYPE]))
            else 0
        )
        # 兼容数组没有 Header；构造一条等价几何上下文，使下游仍只需订阅一个强类型接口。
        observation = FusedObstacle()
        observation.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id=self.output_frame
        )
        observation.obstacle_type = obstacle_type
        observation.obstacle_height = float(msg.data[height_index])
        observation.valid_points = nonnegative_integer_or_zero(
            msg.data[TERRAIN_VALID_POINTS]
        )
        observation.slope_pitch = atan(
            float(msg.data[TERRAIN_GROUND_SLOPE])
            if len(msg.data) > TERRAIN_GROUND_SLOPE
            else 0.0
        )
        observation.slope_roll = (
            float(msg.data[TERRAIN_SLOPE_ROLL])
            if len(msg.data) > TERRAIN_SLOPE_ROLL
            else 0.0
        )
        observation.roughness = (
            float(msg.data[TERRAIN_ROUGHNESS])
            if len(msg.data) > TERRAIN_ROUGHNESS
            else 0.0
        )
        observation.distance = (
            float(msg.data[TERRAIN_LOOKAHEAD])
            if len(msg.data) > TERRAIN_LOOKAHEAD
            else 0.0
        )
        observation.pit_depth = (
            float(msg.data[TERRAIN_PIT_DEPTH])
            if len(msg.data) > TERRAIN_PIT_DEPTH
            else 0.0
        )
        observation.confidence = (
            float(msg.data[TERRAIN_CONFIDENCE])
            if len(msg.data) > TERRAIN_CONFIDENCE
            else 0.0
        )
        observation.width = (
            float(msg.data[TERRAIN_WIDTH])
            if len(msg.data) > TERRAIN_WIDTH
            else 0.0
        )
        observation.clearance_height = (
            float(msg.data[TERRAIN_CLEARANCE_HEIGHT])
            if len(msg.data) > TERRAIN_CLEARANCE_HEIGHT
            else 0.0
        )
        observation.geometry_confirmed = all(
            isfinite(value)
            for value in (
                observation.obstacle_height,
                observation.slope_pitch,
                observation.slope_roll,
                observation.roughness,
            )
        ) and observation.valid_points >= self.min_points
        self.latest_observation = observation
        self.perception_valid = bool(observation.geometry_confirmed)
        assessment = select_terrain_assessment(
            float(msg.data[height_index]),
            float(msg.data[TERRAIN_VALID_POINTS]),
            float(msg.data[TERRAIN_GROUND_SLOPE])
            if len(msg.data) > TERRAIN_GROUND_SLOPE
            else 0.0,
            float(msg.data[TERRAIN_ROUGHNESS])
            if len(msg.data) > TERRAIN_ROUGHNESS
            else 0.0,
            self.min_points,
            self.step_threshold,
            self.climb_threshold,
            self.stop_threshold,
            self.max_slope,
            self.max_roughness,
        )
        pit_depth = (
            float(msg.data[TERRAIN_PIT_DEPTH])
            if len(msg.data) > TERRAIN_PIT_DEPTH
            else 0.0
        )
        assessment = apply_geometry_classification(
            assessment, obstacle_type, pit_depth
        )
        assessment = apply_distance_aware_constraint(
            assessment,
            obstacle_type,
            float(observation.distance),
            self.hard_stop_distance,
            self.hazard_approach_speed,
        )
        assessment = self.assessment_filter.update(assessment)
        visual_active = self._fresh_visual_target()
        assessment = apply_visual_assist(
            assessment, visual_active, self.vision_speed_scale
        )
        self.visual_active_pub.publish(Bool(data=visual_active))
        self.visual_assist_active = visual_active
        self.publish_assessment(*assessment)

    def fused_callback(self, msg: FusedObstacle) -> None:
        """处理相机与点云按时间戳配对后的强类型原子观测。"""
        if not self.prefer_fused:
            return
        now = self.get_clock().now()
        stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        if not observation_stamp_is_current(
            now.nanoseconds * 1e-9,
            stamp,
            self.sensor_timeout,
            self.future_stamp_tolerance,
        ):
            self.perception_valid = False
            self.visual_assist_active = False
            self.latest_observation = None
            self._publish_candidate(("STOP", 0.0))
            return
        self.last_features_time = now
        self.latest_observation = msg
        self.perception_valid = fused_observation_valid(
            msg, self.fused_min_confidence, self.min_points
        )
        assessment = select_fused_assessment(
            msg,
            self.fused_min_confidence,
            self.min_points,
            self.step_threshold,
            self.climb_threshold,
            self.stop_threshold,
            self.max_slope,
            self.max_roughness,
            self.vision_speed_scale,
            self.hard_stop_distance,
            self.hazard_approach_speed,
        )
        assessment = self.assessment_filter.update(assessment)
        visual_active = bool(msg.vision_confirmed) and assessment[0] == "WALK"
        self.visual_active_pub.publish(Bool(data=visual_active))
        self.visual_assist_active = visual_active
        self.publish_assessment(*assessment)

    def vision_callback(self, msg: Float32MultiArray) -> None:
        """缓存视觉辅助证据；超时后自动失效，不能持续限制新场景。"""
        self.last_vision_time = self.get_clock().now()
        self.visual_target = visual_evidence_in_path(
            msg.data, self.vision_min_confidence, self.vision_center_margin
        )
        self.visual_hint_name = visual_obstacle_name_zh(
            msg.data, self.visual_target
        )

    def _fresh_visual_target(self) -> bool:
        """仅在视觉启用、证据有效且接收时间新鲜时返回真。"""
        if not self.vision_enabled or self.last_vision_time is None:
            return False
        age = (self.get_clock().now() - self.last_vision_time).nanoseconds / 1e9
        return age <= self.vision_timeout and self.visual_target

    def _fresh_visual_hint_name(self) -> str:
        """仅返回仍在超时窗口内的视觉名称，防止终端残留上一处障碍。"""
        return self.visual_hint_name if self._fresh_visual_target() else ""

    def timeout_callback(self) -> None:
        """独立检查感知心跳；断流时持续发布零速度上限。"""
        age = (self.get_clock().now() - self.last_features_time).nanoseconds / 1e9
        if age > self.sensor_timeout:
            self.perception_valid = False
            self.visual_assist_active = False
            self.latest_observation = None
            self._publish_candidate(("STOP", 0.0))

    def _publish_candidate(self, candidate: Assessment) -> None:
        """让非法/超时结果经过同一过滤器；STOP 仍会立即生效。"""
        self.publish_assessment(*self.assessment_filter.update(candidate))

    def publish_assessment(self, mode: str, speed: float) -> None:
        """发布人类可读话题及强类型原子接口，并仅在变化时记录日志。"""
        safe_mode = mode if mode in MODE_SEVERITY else "STOP"
        safe_speed = max(0.0, min(1.0, speed)) if isfinite(speed) else 0.0
        self.mode_pub.publish(String(data=safe_mode))
        self.speed_pub.publish(Float32(data=safe_speed))
        safety = NavigationSafety()
        observation = self.latest_observation
        safety.header = (
            observation.header
            if observation is not None
            else Header(
                stamp=self.get_clock().now().to_msg(), frame_id=self.output_frame
            )
        )
        safety.mode = navigation_mode_code(safe_mode)
        safety.speed_limit = safe_speed
        safety.perception_valid = bool(self.perception_valid)
        safety.visual_assist_active = bool(self.visual_assist_active)
        if observation is not None:
            safety.obstacle_type = int(observation.obstacle_type)
            safety.confidence = min(
                1.0, nonnegative_finite_or_zero(observation.confidence)
            )
            safety.obstacle_height = nonnegative_finite_or_zero(
                observation.obstacle_height
            )
            safety.pit_depth = nonnegative_finite_or_zero(
                observation.pit_depth
            )
            safety.slope_pitch = finite_or_zero(observation.slope_pitch)
            safety.slope_roll = finite_or_zero(observation.slope_roll)
            safety.roughness = nonnegative_finite_or_zero(
                observation.roughness
            )
            safety.distance = nonnegative_finite_or_zero(
                observation.distance
            )
            safety.lateral_offset = finite_or_zero(observation.lateral_offset)
            safety.width = nonnegative_finite_or_zero(observation.width)
            safety.clearance_height = nonnegative_finite_or_zero(
                observation.clearance_height
            )
            safety.valid_points = nonnegative_integer_or_zero(
                observation.valid_points
            )
        self.navigation_safety_pub.publish(safety)
        visual_hint_name = self._fresh_visual_hint_name()
        obstacle_name = front_obstacle_name_zh(safety, visual_hint_name)
        # 该文本话题面向终端/UI；明确区分“点云几何确认”和“视觉疑似”，避免把
        # OpenCV 单目分类误当成已量测高度的安全结论。
        if (
            safety.visual_assist_active
            and visual_hint_name
            and int(safety.obstacle_type) == NavigationSafety.OBSTACLE_CLEAR
        ):
            obstacle_name = f"视觉疑似{visual_hint_name}（点云未确认）"
        self.front_obstacle_name_pub.publish(String(data=obstacle_name))

        # 类别/模式变化立即打印；稳定状态每秒刷新一次。这样终端始终能看到当前结果，
        # 又不会按 10 Hz 感知帧率刷屏。数值不参与变化签名，由周期日志展示最新测量。
        status_signature = (
            bool(safety.perception_valid),
            int(safety.obstacle_type),
            int(safety.mode),
            bool(safety.visual_assist_active),
            visual_hint_name,
        )
        now = self.get_clock().now()
        log_age = (
            float("inf")
            if self.last_status_log_time is None
            else (now - self.last_status_log_time).nanoseconds / 1e9
        )
        if (
            status_signature != self.last_obstacle_status
            or log_age >= self.status_log_period
        ):
            self.get_logger().info(
                format_front_obstacle_status(safety, visual_hint_name)
            )
            self.last_obstacle_status = status_signature
            self.last_status_log_time = now
        assessment = (safe_mode, safe_speed)
        if assessment != self.last_assessment:
            self.get_logger().info(
                f"Terrain mode -> {safe_mode}, Nav2 speed limit -> {safe_speed:.2f}"
            )
            self.last_assessment = assessment


def main(args=None):
    """启动地形安全评估节点。"""
    rclpy.init(args=args)
    node = TerrainSafetyAssessor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

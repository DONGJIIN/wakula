"""把地形观测转换为 Nav2 安全等级和速度上限。

职责
----
本模块只回答“当前风险等级”和“Nav2 最多保留多少速度”。它不调用越障 Action，不生成
关节/足端轨迹，也不判断越障完成。高度和粗糙度单位为米，旧数组坡度为 ``dz/dx``，速度
上限为 0～1 比例；证据不足、字段非法或超时均 fail-closed 为 STOP。

真机标定入口
------------
导航风险参数只在 ``config/terrain_navigation.yaml``。必须先完成 ``vision.yaml`` 与
``terrain.yaml`` 的传感器标定，再用已标注 rosbag 调整 STEP/CLIMB/STOP 高度、迟滞和感知
超时；最后结合实测 footprint、Nav2 inflation 和运动团队可接管距离调整入口停车参数。
比赛专名的米制观测包络目前仍是依据 2026 V2.0 规则名义尺寸和 Gazebo
样本建立的初始值，不是已完成的真机标定。确定相机/雷达安装后，应用带真值标签的
rosbag 重新估计这些包络，并将它们迁移到单一的版本化配置；在此之前不要把
代码中的观测上下限误解为规则精确尺寸。
离线工具只会建议视觉最低置信度及三个高度阈值，不会替你决定真机可跨越能力，具体流程
见 ``instruction.txt`` 第五节。

安全边界
--------
STOP 第一帧生效；重复/乱序融合帧不推进风险或名称投票。视觉证据不得绕过点云尺度确认，
感知阈值也不能代替运动控制器的姿态、接触、关节和硬件安全判定。
"""

from dataclasses import dataclass
from math import atan, degrees, isfinite
from typing import Sequence, Tuple

import rclpy
from quadruped_interfaces.msg import FusedObstacle, NavigationSafety, TerrainFeatures

from quadruped_planning.parameter_validation import (
    SAFETY_PARAMETER_NAMES,
    validate_safety_parameters,
)
from quadruped_planning.time_utils import (
    ros_age_is_fresh,
    ros_age_seconds,
    ros_clock_moved_backward,
)
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


@dataclass(frozen=True)
class ObstacleMeasurements:
    """一帧障碍语义分类所需的量测快照。

    字段名明确标出米或度，避免在长条件表达式里反复出现 ``float(msg.field)``，也避免
    把消息中的弧度坡角与规则书中的角度数混用。这个结构只提高可读性，不保存历史、
    不修改消息，也不包含阈值。传感器几何/地形安全参数分别归属
    ``terrain.yaml`` 和 ``terrain_navigation.yaml``；比赛专名分类器中暂存的 2026 V2.0/Gazebo
    观测包络尚待真机 rosbag 标定后迁入单一版本化配置。
    """

    height_m: float
    pit_depth_m: float
    pitch_deg: float
    roll_deg: float
    roughness_m: float
    width_m: float
    clearance_m: float

    @classmethod
    def from_safety(cls, safety: NavigationSafety) -> "ObstacleMeasurements":
        """从强类型消息一次性读取量测；不在这里修正非法数据。"""
        return cls(
            height_m=float(safety.obstacle_height),
            pit_depth_m=float(safety.pit_depth),
            pitch_deg=degrees(float(safety.slope_pitch)),
            roll_deg=degrees(float(safety.slope_roll)),
            roughness_m=float(safety.roughness),
            width_m=float(safety.width),
            clearance_m=float(safety.clearance_height),
        )

    def is_finite(self) -> bool:
        """只有全部量测为有限数时才允许产生可执行比赛专名。"""
        return all(
            isfinite(value)
            for value in (
                self.height_m,
                self.pit_depth_m,
                self.pitch_deg,
                self.roll_deg,
                self.roughness_m,
                self.width_m,
                self.clearance_m,
            )
        )


@dataclass(frozen=True)
class ObstacleClassification:
    """One atomic classifier result for machines and people.

    ``semantic_id`` is the stable English control identity. ``display_name`` is only
    a Chinese UI label and may be reworded or decorated without changing that identity.
    An empty ID deliberately means that the available geometry has not uniquely
    identified a task obstacle.
    """

    semantic_id: str
    display_name: str


def obstacle_measurements_are_valid(safety: NavigationSafety) -> bool:
    """检查名称分类所需字段，供分类器和时序稳定器共用同一失效条件。"""
    return bool(safety.perception_valid) and ObstacleMeasurements.from_safety(
        safety
    ).is_finite()


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
        self.initial = initial
        self.current = initial
        self.pending_mode = None
        self.pending_count = 0

    def reset(self) -> Assessment:
        """Discard pending votes after a sensor session or ROS-clock reset."""
        self.current = self.initial
        self.pending_mode = None
        self.pending_count = 0
        return self.current

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


class ObstacleClassificationStabilizer:
    """Stabilize one complete ``semantic_id``/display pair.

    The immutable pair is voted and replaced atomically. No code reconstructs the
    machine ID from the Chinese label after stabilization. Invalid geometry immediately
    clears both fields so an old task identity cannot survive a sensor fault.
    """

    INVALID = ObstacleClassification("", "感知数据无效")

    def __init__(self, confirmation_frames: int = 3, clear_frames: int = 4):
        self.confirmation_frames = max(1, int(confirmation_frames))
        self.clear_frames = max(1, int(clear_frames))
        self.current = self.INVALID
        self.pending = None
        self.pending_count = 0

    def reset(self) -> ObstacleClassification:
        """Clear both the machine identity and its optional UI representation."""
        self.current = self.INVALID
        self.pending = None
        self.pending_count = 0
        return self.current

    def update(
        self, candidate: ObstacleClassification, perception_valid: bool
    ) -> ObstacleClassification:
        """Return the stable atomic result; invalid input revokes it immediately."""
        if not perception_valid:
            return self.reset()
        value = ObstacleClassification(
            str(candidate.semantic_id).strip(),
            str(candidate.display_name).strip() or "未知障碍",
        )
        if value == self.current:
            self.pending = None
            self.pending_count = 0
            return self.current
        if value == self.pending:
            self.pending_count += 1
        else:
            self.pending = value
            self.pending_count = 1
        required = (
            self.clear_frames
            if not value.semantic_id and value.display_name == "无障碍"
            else self.confirmation_frames
        )
        if self.pending_count >= required:
            self.current = value
            self.pending = None
            self.pending_count = 0
        return self.current


def validate_height_thresholds(
    step: float, climb: float, stop: float
) -> Tuple[float, float, float]:
    """验证米制高度阈值严格递增，否则恢复保守初值。"""
    values = (step, climb, stop)
    if all(isfinite(value) for value in values) and 0.0 <= step < climb < stop:
        return values
    return 0.07, 0.18, 0.32


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


def classify_front_obstacle(
    safety: NavigationSafety, visual_hint_name: str = ""
) -> ObstacleClassification:
    """Classify one geometry snapshot into an atomic machine/UI result.

    Every geometry branch writes the stable English ``semantic_id`` and its Chinese
    ``display_name`` together. Generic/ambiguous geometry deliberately gets an empty
    ID. This function never derives control identity from display text and never reads
    Gazebo names or poses.

    The numeric envelopes below are initial recognition ranges derived from the 2026
    V2.0 nominal geometry and deterministic Gazebo observations. They include partial
    views, ground-fit bias and the current simulated sensor mounting, so they are not
    immutable rule dimensions. Once real hardware exists, calibrate them from labelled
    camera/depth rosbag data and move the resulting values into one versioned profile.
    This function remains the single authority in the meantime; the mission layer must
    re-use its result instead of maintaining another set of numeric limits.
    """
    if not obstacle_measurements_are_valid(safety):
        return ObstacleClassification("", "感知数据无效")

    obstacle_type = int(safety.obstacle_type)
    measured = ObstacleMeasurements.from_safety(safety)
    height_m = measured.height_m
    pit_depth_m = measured.pit_depth_m
    pitch_deg = abs(measured.pitch_deg)
    roll_deg = abs(measured.roll_deg)
    roughness_m = measured.roughness_m
    width_m = measured.width_m
    clearance_m = measured.clearance_m

    # 原始 OpenCV hint 没有经过几何尺度和跨类一致性确认，因此它不参与
    # 下方机器语义分支。即使 raw hint 与 STEP/WALL 冲突，权威点云仍照常
    # 确认 T 台/高墙；hint 只会在 ``format_front_obstacle_status`` 的 UI 辅助字段显示。
    if obstacle_type == NavigationSafety.OBSTACLE_POLE:
        # 限高杆的横梁很细，单帧深度云常先只看到一侧 0.32 m 支柱而归入 POLE；规则
        # 绕杆立柱高度不低于 0.50 m。用量测高度给出接近阶段名称，比直接误报绕杆可靠。
        if (
            0.25 <= height_m <= 0.42
            and width_m <= 0.22
        ):
            return ObstacleClassification("height_bar", "限高杆（支柱结构）")
        return ObstacleClassification("right_angle_poles", "直角绕杆区（立柱）")
    if obstacle_type == NavigationSafety.OBSTACLE_PIT:
        # PIT 只说明 ROI 中存在真实低回波；桥板缝、台阶侧面、场地边缘和墙面遮挡都
        # 可能产生同一粗类型。以下规则从最具特征的轮廓到最保守回退依次判断。
        # 低矮高墙完全遮住墙后近地面后，地面拟合可能跨过墙顶并把墙前区域解释为
        # 约 0.40 m 的负落差。该轮廓比 T 台的踏面落差更深，横向连续宽度接近规则
        # 中的 1 m，且保留中等边缘残差；因此可恢复高墙语义。这里仍只看实时几何，
        # 不读取 Gazebo 中的障碍名称或位置。
        if (
            12.0 <= pitch_deg <= 22.0
            and pit_depth_m > 0.36
            and height_m < 0.10
            and 0.035 <= roughness_m <= 0.070
            and 0.80 <= width_m <= 1.20
        ):
            return ObstacleClassification("high_wall", "高墙（遮挡轮廓）")
        # 正对 T 字台阶的近场点云会用较高踏面拟合参考平面，较低踏面因此暂时落入
        # ``negative`` 区域并被粗分类为 PIT。它与真正坑洞的关键差异是 16～24° 的
        # 连续阶梯趋势、约 0.3 m 高差和接近 1 m 的规则宽度。先恢复为 T 台语义，
        # 否则任务会把台阶错误交给坑洞流程。
        if (
            16.0 <= pitch_deg <= 24.0
            and roll_deg <= 6.0
            and 0.20 <= pit_depth_m <= 0.36
            and height_m < 0.12
            and 0.025 <= roughness_m <= 0.065
            and 0.75 <= width_m <= 1.25
        ):
            return ObstacleClassification("t_shaped_stairs", "T 字形台阶")
        # 木桥 A 的 14° 入口坡只占深度 ROI 的一部分时，地面先验会把坡脚解释成浅坑，
        # 但坡向、低残差和约 1 m 通道宽仍然稳定。这里输出比赛语义而不读取 world pose。
        if (
            7.0 <= pitch_deg <= 16.0
            and 0.08 <= pit_depth_m <= 0.22
            and height_m < 0.08
            and roughness_m < 0.035
            and 0.75 <= width_m <= 1.25
        ):
            return ObstacleClassification("wooden_bridge_a", "木桥 A（14°入口坡）")
        # 障碍赛场地边缘后的低层支撑/地面，在深度云中同样会表现为约 0.10～0.13 m
        # 的负台阶。真正砂砾坑还有 0.15 m 护栏、碎石/木料起伏；若前缘几乎没有凸起、
        # 表面也很平整，就只能确认“场地边界”，不得把它计成砂砾坑并执行越障。
        if (
            height_m < 0.06
            # 深度相机越靠近赛台边缘，部分射线会直接看向更低的参考地面甚至无回波，
            # 量得的“坑深”没有可靠上限。真实砂砾坑入口必须同时出现 0.15 m 护栏、
            # 填料正凸起或明显粗糙度；因此平整、无凸起轮廓只要求负落差下限。
            and pit_depth_m >= 0.07
            and roughness_m < 0.045
            # 靠近边界并带横偏时，前向 ROI 只能截到约 0.4 m 的边缘，不能要求一帧
            # 必须看到完整 0.8 m 宽度。真正砂砾坑的 0.15 m 护栏会产生更高正凸起，
            # 且坑底/碎料通常更深或更粗糙；联合深度、平整度后放宽宽度不会把它误杀。
            and width_m >= 0.35
            and pitch_deg < 5.0
        ):
            return ObstacleClassification("arena_boundary", "场地边界（禁止越界）")
        # 若同一帧同时含有 0.30 m 以上的大块正凸起，它就不是规则中仅 0.15 m 护栏、
        # 0.10 m 深的砂砾坑。常见来源是从木桥/T 台侧面观察时，地面拟合跨过高平台，
        # 低处被粗分为 PIT。保持“待结构确认”可让机器人换角度继续观察，绝不能把桥
        # 误记成坑并执行一次错误的比赛任务。桥 B 正常入口平台约 0.20 m，不受影响。
        if (
            # 规则坑入口护栏约 0.15 m、木桥 B 桥板约 0.20 m；达到 0.22 m 后更像
            # 高踏板/台阶的混合视图，不能继续向坑或桥板缝规则下落。
            height_m >= 0.22
            and width_m >= 0.55
        ):
            return ObstacleClassification("", "台阶或木桥踏板（待结构确认）")
        # 木桥 B 的 0.40 m 周期性板间隙也会形成真实负高度回波。若同时看到较宽、
        # 约 0.20 m 高且表面较平整的桥板，就不能把它叫作砂砾坑。阈值来自规则尺寸，
        # 并已用 Gazebo 点云联调；真机仍需用 rosbag 校准，而不是读取场地坐标。
        if (
            pit_depth_m >= 0.14
            and width_m >= 0.65
            # 板缝后必须同时看到高于地面的桥板。只凭“平整负落差”会把黄色赛台外沿
            # （实测高度约 0、负落差约 0.15 m）误报成木桥 B。
            and height_m >= 0.12
        ):
            return ObstacleClassification("wooden_bridge_b", "木桥 B（桥板间隙）")
        # 不能把通用 PIT 粗分类直接等同于比赛中的砂砾/碎木坑。PIT 只说明 ROI 内存在
        # 负高度回波；桥板缝、台阶侧面、赛台边缘以及被高平台遮挡的地面都可能产生
        # 相同结果。比赛坑必须额外看到约 0.15 m 的入口护栏/碎料正凸起和粗糙表面。
        # 这里采用规则尺寸附近的联合门限；不满足时保留“待确认”，让任务换角度观察，
        # 而不是错误执行一次越障并把它计入八项任务。
        if (
            0.10 <= height_m <= 0.21
            and 0.07 <= pit_depth_m <= 0.22
            # Gazebo 的确定性砂砾/碎木样件在 5 Hz 体素化后实测残差约 0.025 m；
            # 真坑的护栏高、坑深和正凸起已经由其余条件共同确认，因此不应再要求
            # 4.5 cm 的单一粗糙度。平整场地边界没有 0.10 m 以上正凸起，不会通过。
            and roughness_m >= 0.020
            and 0.35 <= width_m <= 1.40
            and pitch_deg < 7.0
        ):
            return ObstacleClassification("gravel_wood_pit", "砂砾与碎木坑")
        return ObstacleClassification("", "坑洞（结构待确认）")
    if obstacle_type == NavigationSafety.OBSTACLE_BAR:
        # 坑区 0.15 m 护栏在斜视点云中可能产生悬空外观；高度明显低于 0.30 m
        # 限高横杆时只标为坑区入口线索，等待后续负高度回波确认。
        if (
            height_m < 0.28
            and clearance_m >= 0.15
        ):
            # 坑护栏是约 0.60 m 的局部窄结构；1 m 宽的高墙在雷达从侧面扫到
            # 顶边时也可能暂时具有“下方有空间”的 BAR 外观。横向连续宽度可在
            # 不依赖场地坐标的前提下区分二者。
            if width_m < 0.80:
                return ObstacleClassification(
                    "gravel_wood_pit", "坑区护栏（后方地形待确认）"
                )
            return ObstacleClassification("high_wall", "高墙（顶边轮廓）")
        return ObstacleClassification("height_bar", "限高杆")
    if obstacle_type == NavigationSafety.OBSTACLE_WALL:
        # 相机斜向看出黄色赛台时，会把台面到场外地面的落差重建成约 0.25 m 的平整
        # 立面。高墙规则高度约 0.30 m，且墙体边缘不会同时满足“低于 0.28 m、近水平、
        # 低残差”的组合；先识别边界，避免在赛台东/西侧反复调用高墙 Action。
        if (
            height_m < 0.28
            and width_m >= 0.55
            and roughness_m < 0.045
            and pitch_deg < 5.0
        ):
            return ObstacleClassification("arena_boundary", "场地边界（禁止越界）")
        # 砂砾坑 0.15 m 护栏在斜视时会与碎料合成约 0.25 m 的窄粗糙立面；高墙则有
        # 约 1 m 连续横宽。先保留坑区入口语义，等待后方负高度确认。
        if (
            height_m < 0.28
            and width_m < 0.80
            and roughness_m >= 0.045
        ):
            return ObstacleClassification(
                "gravel_wood_pit", "坑区护栏（后方地形待确认）"
            )
        return ObstacleClassification("high_wall", "高墙")
    if obstacle_type == NavigationSafety.OBSTACLE_STEP:
        # STEP 覆盖低台阶、桥板、平台和阶梯总体轮廓，是最容易混淆的粗类型。按
        # “周期桥板→坑区填料→平整桥台→T 台→通用待确认”的顺序匹配；前面的专用
        # 规则必须比后面的宽松回退同时使用更多独立几何特征。
        # 木桥 B 的分段踏板实测为约 0.26 m 高、1.1 m 宽；周期板缝会把稳健残差提高到
        # 0.09 m 左右。它比砂砾坑 0.15 m 护栏更高、更宽，整体坡向仍近水平，因此应
        # 在通用“粗糙低台阶=坑区”规则之前确认，防止把桥板间隙误叫成砂砾坑。
        if (
            # 全场标定中，真实分段板的“高度/残差”为 0.20/0.080 m 或
            # 0.261/0.093 m；曾误触发桥 B 的 11.3° 主坡侧面为 0.277/0.059 m，且没有
            # 周期板缝。因此联合收紧高度和残差，无需读取 world 坐标。
            0.19 <= height_m <= 0.27
            # 真桥宽 1 m；机器人偏向通道一侧且 ROI 半宽 0.55 m 时，连通桥板只
            # 保留约 0.71 m。0.65 m 下限仍明显大于孤立立柱/细杆，并继续受高度、
            # 周期残差和近水平坡向三项约束。若真机点云横向裁切更窄，应先增大
            # terrain.yaml:lateral_half_width，再按 rosbag 调本门限。
            and width_m >= 0.65
            # 完整场地样本中桥板周期残差为 0.080～0.093 m，砂砾/碎木填料约
            # 0.070 m；保留小间隔，避免粗糙坑区被提升为木桥 B。
            and roughness_m >= 0.078
            and pitch_deg < 5.0
        ):
            return ObstacleClassification("wooden_bridge_b", "木桥 B（分段桥板）")
        # 进入砂砾/碎木区后，护栏与低洼填料会在单帧栅格中表现成约 0.15～0.25 m
        # 的低台阶，同时粗糙度显著升高。该组合接续上面的“坑区护栏”接近提示。
        if (
            0.12 <= height_m <= 0.19
            and roughness_m >= 0.05
            and width_m >= 0.40
        ):
            return ObstacleClassification(
                "gravel_wood_pit", "砂砾与碎木坑（入口/填料区）"
            )
        # 近场可能看不到填料后的 0.10 m 坑底，只剩 0.19～0.23 m 粗糙 STEP 和浅负
        # 残差；木桥 B 已由上方更强的周期残差门（>=0.078 m）提前分离。
        if (
            0.19 <= height_m <= 0.23
            and 0.02 <= pit_depth_m <= 0.08
            and 0.045 <= roughness_m < 0.078
            and 0.40 <= width_m <= 1.40
        ):
            return ObstacleClassification("gravel_wood_pit", "砂砾与碎木坑（填料区）")
        # 两座木桥的起终平台都是约 0.20 m 高、1 m 宽的平整结构。低残差可将它们与
        # T 台的多级踏面区分；A/B 在看见坡或板间隙前仍保持 unknown 语义。
        if (
            0.17 <= height_m <= 0.27
            and roughness_m < 0.025
            and width_m >= 0.80
        ):
            return ObstacleClassification(
                "wooden_bridge_unknown", "木桥平台（A/B 待结构确认）"
            )
        # 高墙已有独立 WALL 分支。正对 T 台时，近场平面会穿过多级踏面，使“相对平面
        # 高度”小于总高；但它仍表现为宽障碍、7～18° 的阶梯总体趋势和明显离散残差。
        # 同时支持直接看到 0.40 m 顶部与只看到多级踏面的两种距离，避免再误叫坑区。
        stepped_profile = (
            # 规则 T 台的 0.10 m 级高/0.30 m 踏面在完整 Gazebo 深度云中得到
            # 16.23°，并非理想几何的单一级面角。上限保留到 18°，覆盖约 2° 的
            # 深度/外参误差；主斜坡和木桥 A 的连续平面会在 terrain_geometry 中
            # 先归为 CLEAR，侧看主斜坡还会被下面的 roll 门拒绝，因此不会因这次
            # 放宽被误记为 T 台。真机若反复把粗糙坡误报为 T 台，应先校准外参和
            # 地面拟合，再结合 rosbag 收紧该上限，不能只提高高度阈值。
            7.0 <= pitch_deg <= 18.0
            and roughness_m >= 0.02
        )
        # T 台总高 0.40 m，但前一级踏面在近场常只量到约 0.30 m，且相机斜视只能覆盖
        # 约 0.5 m 横宽。它仍比桥面高、表面离散残差也更大；这组联合条件避免要求
        # “一帧必须看全 1.9 m 台面”而漏检真实入口。
        partial_t_stair = (
            width_m >= 0.45
            and height_m >= 0.28
            and roughness_m >= 0.04
        )
        if (
            # 规则 T 台总高 0.40 m；保留 3 cm 测量余量。全场回归证明主斜坡长侧面
            # 会连续输出 0.44--0.45 m、低残差且 roll 近零的 STEP，过去的 0.48 m
            # 上限会把它稳定误报为 T 台并执行错误长距离 Action。高于 0.43 m 必须
            # 换视角看到真实多级踏面后再确认，不能用“表面平整”作为放宽理由。
            height_m <= 0.43
            # 主斜坡从长侧观察时会形成与高台阶相似的橘色立面，但坡面法向在机体
            # 横向留下接近 11.3° 的 roll；真正从入口对准 T 台时 roll 应接近零。
            # 这是坐标无关的几何校验，不读取 Gazebo 障碍名称或位置。
            and roll_deg <= 6.0
            and (
                (
                    width_m >= 0.60
                    and (height_m >= 0.32 or stepped_profile)
                )
                or partial_t_stair
            )
        ):
            return ObstacleClassification("t_shaped_stairs", "T 字形台阶")
        return ObstacleClassification("", "台阶或木桥踏板（待结构确认）")

    if obstacle_type == NavigationSafety.OBSTACLE_CLEAR:
        # 只在横滚较小时把前后坡度解释成赛道坡面；侧向倾斜仍保留通用地形名称。
        if roll_deg <= 6.0 and 7.0 <= pitch_deg <= 12.0:
            return ObstacleClassification("main_slope", "主斜坡（11.3°坡面）")
        if roll_deg <= 6.0 and 12.0 < pitch_deg <= 17.0:
            # 规则中的连续 14° 入口坡属于木桥 A；木桥 B 由周期分段桥板识别。这里仍
            # 只使用坡角和横滚，不读取仿真名称或场地位置，因此可直接迁移到真机。
            return ObstacleClassification("wooden_bridge_a", "木桥 A（14°入口坡）")
        # 有色视觉候选没有尺度，不能改变速度或声称具体障碍；但 UI 也不能把它吞掉后
        # 错报“无障碍”。名称明确表达点云尚未完成结构分类。
        if visual_hint_name:
            return ObstacleClassification(
                "", f"视觉检测到{visual_hint_name}（点云待分类）"
            )
        return ObstacleClassification("", "无障碍")
    return ObstacleClassification("", obstacle_name_zh(obstacle_type, True))


def front_obstacle_name_zh(
    safety: NavigationSafety, visual_hint_name: str = ""
) -> str:
    """Return only the optional Chinese UI label from the atomic classification."""
    return classify_front_obstacle(safety, visual_hint_name).display_name


def format_front_obstacle_status(
    safety: NavigationSafety,
    visual_hint_name: str = "",
    resolved_name: str = "",
) -> str:
    """生成一行可读状态，明确这是前向 ROI 的融合判断而非全场物体列表。"""
    name = str(resolved_name) or front_obstacle_name_zh(safety, visual_hint_name)
    mode = MODE_NAMES.get(int(safety.mode), "UNKNOWN")
    geometry_is_clear = (
        int(safety.obstacle_type) == NavigationSafety.OBSTACLE_CLEAR
    )
    if (
        not resolved_name
        and safety.visual_assist_active
        and visual_hint_name
        and geometry_is_clear
    ):
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
    """拒绝零时间戳、陈旧重放帧和超出跨设备容差的未来观测。

    Header 是传感器采样时钟而非本节点接收时钟，允许已标定的微小时钟偏差。本机
    receive-time 心跳仍严格要求非负年龄；模拟器/rosbag 回拨由节点的 epoch reset 处理。
    """
    values = (now_seconds, stamp_seconds, maximum_age, future_tolerance)
    if not all(isfinite(float(value)) for value in values):
        return False
    age = float(now_seconds) - float(stamp_seconds)
    return (
        stamp_seconds > 0.0
        and maximum_age > 0.0
        and -max(0.0, future_tolerance) <= age <= maximum_age
    )


def terrain_features_to_fused_observation(msg: TerrainFeatures) -> FusedObstacle:
    """Losslessly adapt one typed terrain frame to the common safety snapshot.

    No field is repaired or inferred here.  In particular, ``valid=false`` maps to
    ``geometry_confirmed=false`` so the common validator returns STOP even if every
    numeric field happens to look plausible.
    """
    observation = FusedObstacle()
    observation.header = msg.header
    observation.obstacle_type = int(msg.obstacle_type)
    observation.confidence = float(msg.confidence)
    observation.geometry_confirmed = bool(msg.valid)
    observation.vision_confirmed = False
    for field in (
        "obstacle_height",
        "pit_depth",
        "slope_pitch",
        "slope_roll",
        "roughness",
        "distance",
        "lateral_offset",
        "width",
        "structure_heading",
        "structure_heading_confidence",
        "clearance_height",
        "valid_points",
    ):
        setattr(observation, field, getattr(msg, field))
    return observation


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

    坑洞、墙和横杆需要未来越障控制器接管，因此在交接区速度上限为零。对于 POLE，
    本安全层始终只给出低速 Nav2 约束：普通/矮柱由 Nav2 绕行；高度和语义都符合规则
    绕杆赛项时，由任务层另行进入 TraverseObstacle Action。这里不决定任务语义，也不
    发出任何动作指令。
    """
    if obstacle_type == GEOMETRY_PIT and pit_depth > 0.0:
        return "STOP", 0.0
    if obstacle_type in (GEOMETRY_WALL, GEOMETRY_BAR):
        return "STOP", 0.0
    if obstacle_type == GEOMETRY_POLE:
        # 高立柱可能先被高度阈值评成 STOP。显式 POLE 类别优先：安全层先保留低速
        # Nav2 接近/绕行能力，具体碰撞边界仍由 scan/costmap 保证；任务层若确认它是
        # 规则绕杆赛项，会在独立的语义与 Action 闸门处取得控制权。
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
        msg.structure_heading,
        msg.structure_heading_confidence,
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
        and 0.0 <= float(msg.structure_heading_confidence) <= 1.0
        and float(msg.clearance_height) >= 0.0
        and int(msg.valid_points) >= max(1, int(min_points))
    )


def observation_stamp_strictly_advances(
    previous_stamp: float | None, current_stamp: float
) -> bool:
    """Reject duplicate/out-of-order fused frames from temporal confirmation.

    Freshness relative to ``now`` is necessary but insufficient: two packets can both
    be recent while the second is a DDS duplicate or an older driver-buffered sample.
    Such a packet must not refresh the perception heartbeat or count as another
    obstacle vote. A caller resets ``previous_stamp`` to ``None`` when ROS time moves
    backward and a new bag/simulator session begins.
    """
    if not isfinite(float(current_stamp)) or float(current_stamp) <= 0.0:
        return False
    if previous_stamp is None:
        return True
    return (
        isfinite(float(previous_stamp))
        and float(current_stamp) > float(previous_stamp) + 1e-9
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
    hard_stop_distance: float = 1.20,
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

    def __init__(self, **node_kwargs):
        """加载安全阈值并连接强类型融合接口与旧数组兼容接口。

        ``prefer_fused_obstacle`` 为真时融合消息是唯一权威输入；旧数组仍被订阅只是为了
        兼容关闭视觉的点云路径，不能让两条路径同时争抢当前状态。
        """
        super().__init__("terrain_safety_assessor", **node_kwargs)
        for name, default in (
            ("step_threshold", 0.07),
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
            ("hard_stop_distance", 1.20),
            ("hazard_approach_speed", 0.25),
            ("future_stamp_tolerance", 0.10),
            ("status_log_period", 1.0),
        ):
            self.declare_parameter(name, default)
        self.declare_parameter("min_points", 30)
        self.declare_parameter("prefer_fused_obstacle", True)
        self.declare_parameter("clear_confirmation_frames", 5)
        self.declare_parameter("hazard_confirmation_frames", 3)
        self.declare_parameter("name_confirmation_frames", 3)
        self.declare_parameter("name_clear_frames", 4)
        self.declare_parameter("vision_assist_enabled", True)
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("legacy_features_enabled", False)

        validate_safety_parameters(
            {name: self.get_parameter(name).value for name in SAFETY_PARAMETER_NAMES}
        )

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
                "Invalid height thresholds; restored 0.07/0.18/0.32 m"
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
            "hard_stop_distance", 1.20
        )
        self.hazard_approach_speed = self._unit_parameter(
            "hazard_approach_speed"
        )
        self.output_frame = (
            str(self.get_parameter("output_frame").value) or "base_link"
        )
        self.legacy_features_enabled = bool(
            self.get_parameter("legacy_features_enabled").value
        )
        self.status_log_period = self._positive_parameter("status_log_period", 1.0)
        self.assessment_filter = ConservativeAssessmentFilter(
            int(self.get_parameter("clear_confirmation_frames").value),
            ("STOP", 0.0),
            int(self.get_parameter("hazard_confirmation_frames").value),
        )
        self.classification_stabilizer = ObstacleClassificationStabilizer(
            int(self.get_parameter("name_confirmation_frames").value),
            int(self.get_parameter("name_clear_frames").value),
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
        # 无视觉模式也使用强类型、带 Header/valid 的在线合同。旧数组没有采样时刻、
        # frame 或有效位，默认不能影响安全链；仅在回放无法迁移的历史 rosbag 时显式打开。
        self.create_subscription(
            TerrainFeatures,
            "/terrain/features_stamped",
            self.typed_features_callback,
            10,
        )
        if self.legacy_features_enabled:
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
        self.last_features_time = None
        self.last_fused_stamp = None
        self.last_typed_stamp = None
        self.last_vision_time = None
        self.visual_target = False
        self.visual_hint_name = ""
        self.visual_assist_active = False
        self.perception_valid = False
        self.latest_observation = None
        self.last_assessment = None
        self.last_obstacle_status = None
        self.last_status_log_time = None
        self.last_clock_time = self.get_clock().now()
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

    def _reset_temporal_state(self, now) -> None:
        """Clear every observation, vote and heartbeat tied to an old ROS epoch."""
        self.last_features_time = None
        self.last_fused_stamp = None
        self.last_typed_stamp = None
        self.last_vision_time = None
        self.visual_target = False
        self.visual_hint_name = ""
        self.visual_assist_active = False
        self.perception_valid = False
        self.latest_observation = None
        self.assessment_filter.reset()
        self.classification_stabilizer.reset()
        self.last_assessment = None
        self.last_obstacle_status = None
        self.last_status_log_time = None
        self.last_clock_time = now

    def _handle_clock_rewind(self, now) -> bool:
        """Fail closed once when ``/clock`` starts a new simulator/rosbag epoch."""
        if not ros_clock_moved_backward(now, self.last_clock_time):
            self.last_clock_time = now
            return False
        self._reset_temporal_state(now)
        self.get_logger().warning(
            "ROS clock moved backward; reset terrain observations and temporal votes"
        )
        return True

    def _reject_metric_observation(self) -> None:
        """Invalidate the current geometry immediately without refreshing its heartbeat."""
        self.perception_valid = False
        self.visual_assist_active = False
        self.latest_observation = None
        self._publish_candidate(("STOP", 0.0))

    def _process_metric_observation(self, observation: FusedObstacle) -> None:
        """Run fused or typed point-cloud geometry through one identical safety path."""
        self.latest_observation = observation
        self.perception_valid = fused_observation_valid(
            observation, self.fused_min_confidence, self.min_points
        )
        assessment = select_fused_assessment(
            observation,
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
        visual_active = bool(observation.vision_confirmed) and assessment[0] == "WALK"
        self.visual_active_pub.publish(Bool(data=visual_active))
        self.visual_assist_active = visual_active
        self.publish_assessment(*assessment)

    def typed_features_callback(self, msg: TerrainFeatures) -> None:
        """Use typed point-cloud geometry as the no-vision online authority.

        The producer must supply a current, strictly advancing Header in ``output_frame``
        and set ``valid`` only after TF/point-count/geometry checks.  A live ``valid=false``
        frame refreshes the transport heartbeat but still produces STOP; a malformed,
        stale, future, duplicate or wrong-frame message cannot refresh the heartbeat.
        """
        if self.prefer_fused:
            return
        now = self.get_clock().now()
        self._handle_clock_rewind(now)
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if (
            str(msg.header.frame_id) != self.output_frame
            or not observation_stamp_is_current(
                now.nanoseconds * 1e-9,
                stamp,
                self.sensor_timeout,
                self.future_stamp_tolerance,
            )
            or not observation_stamp_strictly_advances(self.last_typed_stamp, stamp)
        ):
            self._reject_metric_observation()
            return
        self.last_typed_stamp = stamp
        self.last_features_time = now
        self._process_metric_observation(terrain_features_to_fused_observation(msg))

    def features_callback(self, msg: Float32MultiArray) -> None:
        """处理无相机模式下的旧数组特征，保留既有 rosbag 可回放性。"""
        if self.prefer_fused or not self.legacy_features_enabled:
            return
        now = self.get_clock().now()
        self._handle_clock_rewind(now)
        if len(msg.data) < 4:
            self._reject_metric_observation()
            return
        self.last_features_time = now
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
            stamp=now.to_msg(), frame_id=self.output_frame
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
        self._handle_clock_rewind(now)
        now_seconds = now.nanoseconds * 1e-9
        stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        if (
            str(msg.header.frame_id) != self.output_frame
            or not observation_stamp_is_current(
                now_seconds,
                stamp,
                self.sensor_timeout,
                self.future_stamp_tolerance,
            )
        ):
            self._reject_metric_observation()
            return
        if not observation_stamp_strictly_advances(
            self.last_fused_stamp, stamp
        ):
            # 直接忽略而不重发：若生产者持续重复一个时间戳，timeout_callback 会按
            # 心跳超时停车。这样重复 DDS 流量不能冒充健康 5 Hz 传感器或累计语义票。
            self.get_logger().warning(
                "Ignoring duplicate/out-of-order fused obstacle frame",
                throttle_duration_sec=2.0,
            )
            return
        self.last_fused_stamp = stamp
        self.last_features_time = now
        self._process_metric_observation(msg)

    def vision_callback(self, msg: Float32MultiArray) -> None:
        """缓存视觉辅助证据；超时后自动失效，不能持续限制新场景。"""
        now = self.get_clock().now()
        self._handle_clock_rewind(now)
        self.last_vision_time = now
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
        now = self.get_clock().now()
        if not ros_age_is_fresh(now, self.last_vision_time, self.vision_timeout):
            self.last_vision_time = None
            self.visual_target = False
            self.visual_hint_name = ""
            return False
        return self.visual_target

    def _fresh_visual_hint_name(self) -> str:
        """仅返回仍在超时窗口内的视觉名称，防止终端残留上一处障碍。"""
        return self.visual_hint_name if self._fresh_visual_target() else ""

    def timeout_callback(self) -> None:
        """独立检查感知心跳；断流时持续发布零速度上限。"""
        now = self.get_clock().now()
        if self._handle_clock_rewind(now):
            self._publish_candidate(("STOP", 0.0))
            return
        if not ros_age_is_fresh(now, self.last_features_time, self.sensor_timeout):
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
            safety.structure_heading = finite_or_zero(
                observation.structure_heading
            )
            safety.structure_heading_confidence = min(
                1.0,
                nonnegative_finite_or_zero(
                    observation.structure_heading_confidence
                ),
            )
            safety.clearance_height = nonnegative_finite_or_zero(
                observation.clearance_height
            )
            safety.valid_points = nonnegative_integer_or_zero(
                observation.valid_points
            )
        visual_hint_name = self._fresh_visual_hint_name()
        geometry_valid = obstacle_measurements_are_valid(safety)
        classification = classify_front_obstacle(safety, visual_hint_name)
        # 该文本话题面向终端/UI；明确区分“点云几何确认”和“视觉疑似”，避免把
        # OpenCV 单目分类误当成已量测高度的安全结论。几何字段非法时不能进入这个
        # 覆盖分支，否则一帧 NaN 点云可能被视觉文本包装成有效目标并继续累计语义票。
        if (
            geometry_valid
            and safety.visual_assist_active
            and visual_hint_name
            and int(safety.obstacle_type) == NavigationSafety.OBSTACLE_CLEAR
        ):
            classification = ObstacleClassification(
                "", f"视觉疑似{visual_hint_name}（点云未确认）"
            )
        classification = self.classification_stabilizer.update(
            classification, geometry_valid
        )
        # Publish the ID and display name from the same stabilized classification.  The
        # Header and all metric fields therefore remain one atomic observation; a DDS
        # scheduling difference between this message and the optional UI String can no
        # longer attach the previous obstacle's name to new geometry.
        safety.semantic_id = classification.semantic_id
        self.navigation_safety_pub.publish(safety)
        self.front_obstacle_name_pub.publish(
            String(data=classification.display_name)
        )

        # 类别/模式变化立即打印；稳定状态每秒刷新一次。这样终端始终能看到当前结果，
        # 又不会按 10 Hz 感知帧率刷屏。数值不参与变化签名，由周期日志展示最新测量。
        status_signature = (
            bool(safety.perception_valid),
            classification.display_name,
            safety.semantic_id,
            int(safety.mode),
            bool(safety.visual_assist_active),
            visual_hint_name,
        )
        now = self.get_clock().now()
        log_age = ros_age_seconds(now, self.last_status_log_time)
        if log_age < 0.0:
            # This path is defensive for direct publish calls outside the normal callbacks;
            # a negative logging age must not suppress status indefinitely after /clock reset.
            self.last_status_log_time = None
            self.last_obstacle_status = None
            log_age = float("inf")
        if (
            status_signature != self.last_obstacle_status
            or log_age >= self.status_log_period
        ):
            self.get_logger().info(
                format_front_obstacle_status(
                    safety,
                    visual_hint_name,
                    resolved_name=classification.display_name,
                )
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

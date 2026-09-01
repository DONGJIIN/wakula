"""在 ``base_link`` 中提取点云地形几何。

职责
----
节点选择一路稳定 PointCloud2，变换到机器人坐标系，在有界前向 ROI 内估计地面、坡度、
粗糙度、障碍高度、坑深、横杆净空和结构方向，并复用凸起点给 Nav2 标障。这里输出上层
通行特征，不生成足端轨迹。长度单位为米；旧数组坡度字段为 ``dz/dx``，强类型消息角度为
弧度，字段合同见根目录 ``connect.txt``。

真机标定入口
------------
所有可调值只在 ``config/terrain.yaml``。接入深度相机或 3D 雷达后按以下顺序处理：
1. 校验点云单位、Header、``sensor_frame -> base_link`` 外参和静态平地朝向；
2. 用已知尺寸标定物采集平地、5/7/10 cm 台阶、坡、坑、墙、横杆及大量无障碍 bag；
3. 先调 ROI 与地面，再调台阶/坑/墙/杆和连通回波门，最后才压缩点数与处理频率；
4. 将 ``TerrainFeatures`` 米制输出与卷尺/角度仪真值比较，并按 ``instruction.txt`` 第五节
   生成分类报告；每次只改一组参数，使用独立 bag 复验。

安全与移植边界
--------------
无回波不等于坑洞；无效、重复、乱序或过期点云不刷新来源心跳。不得把安装高度、量程或
噪声补偿硬编码进 Python，否则同一算法无法无修改移植到另一传感器或机器人。
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
from tf2_ros import Buffer, TransformException, TransformListener

from quadruped_interfaces.msg import TerrainFeatures

from quadruped_perception.parameter_validation import (
    TERRAIN_PARAMETER_NAMES,
    validate_terrain_parameters,
)
from quadruped_perception.sensor_contracts import (
    point_cloud_message_contract_valid,
    source_stamp_strictly_advances,
    source_stamp_is_plausible,
)
from quadruped_perception.terrain_geometry import (
    CLEAR,
    analyze_terrain_geometry,
    navigation_obstacle_points,
)

from quadruped_perception.topic_selection import should_accept_source


TerrainResult = Tuple[list, int]
# Float32MultiArray 没有字段名，因此在代码中集中声明下标，避免各节点散落魔法数字。
# 修改该顺序属于通信接口变更，必须同步 planning、tools 和 connect.txt。
FEATURE_GROUND_Z = 0
FEATURE_HIGH_Z = 1
FEATURE_OBSTACLE_HEIGHT = 2
FEATURE_VALID_POINTS = 3
FEATURE_GROUND_SLOPE = 4
FEATURE_ROUGHNESS = 5
FEATURE_FRONTAL_HEIGHT = 6
FEATURE_LOOKAHEAD = 7
FEATURE_TRAVERSABILITY = 8
FEATURE_PIT_DEPTH = 9
FEATURE_SLOPE_ROLL = 10
FEATURE_OBSTACLE_TYPE = 11
FEATURE_CONFIDENCE = 12
FEATURE_WIDTH = 13
FEATURE_CLEARANCE_HEIGHT = 14
FEATURE_COUNT = 15
DEFAULT_POINT_CLOUD_TOPICS = [
    "/camera/depth/points",
    "/camera/depth/color/points",
    "/camera/points",
    "/points",
]


def bounded_point_sample(xyz: np.ndarray, maximum_points: int) -> np.ndarray:
    """在坐标变换前确定性限制原始点数，避免高分辨率 RGB-D 云耗尽算力。

    640x480 深度相机一帧可包含 30 万点，而地形 ROI 最终只有约 500 个 5 cm 栅格。
    对完整点云逐点做 TF 既没有增加有效空间分辨率，也会在 Gazebo 与 Nav2 同机运行时
    造成秒级调度抖动。等间隔索引覆盖整幅有序点云，结果可复现，且不会像截取数组前段
    那样只保留图像顶部。非正上限表示不采样，便于离线精度对照。
    """
    points = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    limit = int(maximum_points)
    if limit <= 0 or len(points) <= limit:
        return points
    indices = np.linspace(0, len(points) - 1, limit, dtype=np.int64)
    return points[indices]


def transform_xyz(xyz: np.ndarray, translation, quaternion) -> np.ndarray:
    """只变换 XYZ，同时允许 PointCloud2 携带任意其他字段。

    不同 RGB-D/雷达驱动可能追加 packed RGB、intensity、ring 或对齐填充。用
    ``tf2_sensor_msgs`` 重建完整结构时可能因厂商字段布局失败，而地形分析只需要 XYZ，
    因此这里用有界 NumPy 变换主动解除几何算法与无关字段的耦合。
    """
    points = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    # GPU 深度相机会用 +/-Inf 表示无回波像素；矩阵乘法前剔除，避免告警和无效计算。
    points = points[np.isfinite(points).all(axis=1)]
    tx, ty, tz = (float(value) for value in translation)
    qx, qy, qz, qw = (float(value) for value in quaternion)
    values = (tx, ty, tz, qx, qy, qz, qw)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not all(np.isfinite(value) for value in values) or norm < 1e-9:
        raise ValueError("point-cloud transform is non-finite or degenerate")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    rotation = np.asarray(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qz * qw),
                2 * (qx * qz + qy * qw),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )
    return (points @ rotation.T + np.asarray((tx, ty, tz))).astype(np.float32)


def filter_roi_points(
    xyz: np.ndarray,
    x_min: float,
    x_max: float,
    y_half: float,
    max_points: int,
    z_min: float = -1.0,
    z_max: float = 2.0,
) -> np.ndarray:
    """返回前向 ROI 内有限且数量有上限的 ``Nx3`` 点数组。

    等间隔抽样是确定性的：相同输入总会得到相同输出，便于 rosbag 回放复现结果。
    它不是体素滤波，不能替代真机阶段对点云密度和盲区的检查。
    """
    points = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    # ROI 只保留机器人正前方区域，既减少计算量，也避免腿部点云干扰。
    valid = np.isfinite(points).all(axis=1)
    valid &= (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
    valid &= np.abs(points[:, 1]) <= y_half
    # 深度相机无回波像素有时不是 NaN/Inf，而是驱动定义的极大有限值。经过相机到
    # base_link 的轴变换后，这些值可能落在前向 XY ROI 内，却产生几十米高的伪障碍。
    # 垂向物理窗口既过滤这类哨兵值，也避免天空/机器人自身进入地形拟合；上下界作为
    # 参数暴露，换真机或大型机器人时可按安装高度修改。
    lower_z, upper_z = sorted((float(z_min), float(z_max)))
    valid &= (points[:, 2] >= lower_z) & (points[:, 2] <= upper_z)
    points = points[valid]
    if len(points) > max_points:
        indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
        points = points[indices]
    return points


def fit_ground_envelope(
    points: np.ndarray, ground_percentile: float, bin_count: int = 12
) -> Tuple[float, float, np.ndarray, float]:
    """用纵向分箱低分位点拟合地面包络。

    直接对全部点拟合会让台阶顶面或墙面把“水平地面”拉成斜坡。这里先沿 x 分箱，
    每箱只取较低的 z 分位数作为候选地面，再拟合 ``z = slope*x + intercept``。

    Returns:
        ``(slope, intercept, relative_z, roughness)``。``relative_z`` 是每个点
        相对拟合地面的高度；roughness 同时考虑包络起伏和低层点离散度。
    """
    x_values = points[:, 0].astype(np.float64)
    z_values = points[:, 2].astype(np.float64)
    quantile = float(np.clip(ground_percentile, 0.02, 0.40))
    edges = np.linspace(float(x_values.min()), float(x_values.max()), bin_count + 1)
    sample_x = []
    sample_z = []
    for index in range(bin_count):
        upper_inclusive = index == bin_count - 1
        selected = (x_values >= edges[index]) & (
            (x_values <= edges[index + 1])
            if upper_inclusive
            else (x_values < edges[index + 1])
        )
        if np.count_nonzero(selected) < 3:
            continue
        sample_x.append(float(np.median(x_values[selected])))
        sample_z.append(float(np.quantile(z_values[selected], quantile)))

    if len(sample_x) >= 2:
        # 手写一元最小二乘可避免构造较大的设计矩阵，且退化条件清晰可控。
        centered = np.asarray(sample_x) - np.mean(sample_x)
        denominator = float(np.dot(centered, centered))
        slope = (
            0.0
            if denominator < 1e-9
            else float(
                np.dot(centered, np.asarray(sample_z) - np.mean(sample_z))
                / denominator
            )
        )
        intercept = float(np.mean(sample_z) - slope * np.mean(sample_x))
        profile_residuals = np.asarray(sample_z) - (
            slope * np.asarray(sample_x) + intercept
        )
        profile_roughness = float(np.sqrt(np.mean(np.square(profile_residuals))))
    else:
        # 有效分箱不足时不能可靠估坡，退化为水平地面；后续仍受 min_points 约束。
        slope = 0.0
        intercept = float(np.quantile(z_values, quantile))
        profile_roughness = 0.0

    relative_z = z_values - (slope * x_values + intercept)
    # 只用相对高度较低的一半估计点噪声，避免把真实障碍表面计入地面粗糙度。
    lower_half = relative_z[relative_z <= np.quantile(relative_z, 0.50)]
    point_roughness = float(np.std(lower_half)) if len(lower_half) else 0.0
    roughness = max(profile_roughness, point_roughness)
    return slope, intercept, relative_z, roughness


def compute_terrain_features(
    xyz: np.ndarray,
    x_min: float,
    x_max: float,
    y_half: float,
    max_points: int,
    ground_percentile: float,
    critical_height: float,
    max_slope: float,
    max_roughness: float,
    min_points: int,
    z_min: float = -1.0,
    z_max: float = 2.0,
) -> Optional[TerrainResult]:
    """从 ``base_link`` 下的 ``Nx3`` 点云计算固定九字段地形特征。

    返回 ``None`` 表示证据不足，调用者不得将其解释为平地；在线节点会停止发布新特征，
    规划节点随后依靠传感器超时进入 STOP。该 fail-closed 行为是安全合同的一部分。
    """
    points = filter_roi_points(
        xyz, x_min, x_max, y_half, max_points, z_min=z_min, z_max=z_max
    )
    if len(points) < min_points:
        return None

    x_values = points[:, 0].astype(np.float64)
    slope, intercept, relative_z, roughness = fit_ground_envelope(
        points, ground_percentile
    )
    ground_offset = float(
        np.quantile(relative_z, np.clip(ground_percentile, 0.02, 0.40))
    )
    obstacle_height = max(0.0, float(np.quantile(relative_z, 0.98) - ground_offset))
    ground = float(slope * np.median(x_values) + intercept + ground_offset)
    high = ground + obstacle_height

    # 中央走廊比完整 ROI 更接近视觉画面中心和机器人实际落脚通道。
    central = np.abs(points[:, 1]) <= y_half * 0.50
    frontal_height = obstacle_height
    if np.count_nonzero(central) >= max(5, min_points // 3):
        frontal_height = max(
            0.0,
            float(np.quantile(relative_z[central], 0.98) - ground_offset),
        )

    # lookahead 表示最近成片障碍的 x 距离；稀疏单点不会决定距离。
    height_gate = max(0.04, min(0.08, critical_height * 0.30))
    obstacle_x = x_values[relative_z - ground_offset >= height_gate]
    minimum_obstacle_points = max(3, int(len(points) * 0.005))
    lookahead = (
        float(np.quantile(obstacle_x, 0.10))
        if len(obstacle_x) >= minimum_obstacle_points
        else float(x_max)
    )
    # traversability 仅供监控/排序，真正动作等级仍由 planning 中显式阈值决定。
    # 保持动作判定离散透明，便于真机安全审查和离线复现。
    height_penalty = obstacle_height / max(critical_height, 1e-3)
    slope_penalty = abs(slope) / max(max_slope, 1e-3) * 0.35
    roughness_penalty = roughness / max(max_roughness, 1e-3) * 0.35
    penalty = height_penalty + slope_penalty + roughness_penalty
    traversability = float(np.clip(1.0 - penalty, 0.0, 1.0))
    features = [0.0] * FEATURE_COUNT
    features[FEATURE_GROUND_Z] = ground
    features[FEATURE_HIGH_Z] = high
    features[FEATURE_OBSTACLE_HEIGHT] = obstacle_height
    features[FEATURE_VALID_POINTS] = float(len(points))
    features[FEATURE_GROUND_SLOPE] = slope
    features[FEATURE_ROUGHNESS] = roughness
    features[FEATURE_FRONTAL_HEIGHT] = frontal_height
    features[FEATURE_LOOKAHEAD] = float(np.clip(lookahead, x_min, x_max))
    features[FEATURE_TRAVERSABILITY] = traversability
    return features, len(points)


class TerrainAnalyzer(Node):
    """限频处理最新点云并发布地形特征、Nav2 障碍点和诊断信息。"""

    def __init__(self, **node_kwargs):
        """声明地形参数并建立“最新帧覆盖”式点云处理流水线。

        订阅回调只保存最新消息，定时器才执行 TF 和几何分析。这种结构会主动丢弃处理不过来
        的旧帧，避免 RK3588 在高频点云下积压并输出过时的安全判断。
        """
        super().__init__("terrain_analyzer", **node_kwargs)
        self.declare_parameter("input_topic", "")
        self.declare_parameter(
            "input_topic_candidates", DEFAULT_POINT_CLOUD_TOPICS
        )
        self.declare_parameter("target_frame", "base_link")
        # 声明默认值与正式 terrain.yaml 保持一致；这样直接 ros2 run 调试时不会
        # 悄悄退回高负载旧档。launch 参数仍可按真机 rosbag 覆盖这些初值。
        self.declare_parameter("processing_hz", 5.0)
        self.declare_parameter("transform_timeout", 0.05)
        self.declare_parameter("transform_max_points", 40000)
        self.declare_parameter("max_points", 12000)
        self.declare_parameter("nav2_cloud_max_points", 1800)
        self.declare_parameter("nav2_obstacle_min_height_above_ground", 0.05)
        self.declare_parameter("front_x_min", 0.10)
        self.declare_parameter("front_x_max", 2.50)
        self.declare_parameter("lateral_half_width", 0.55)
        self.declare_parameter("front_z_min", -1.0)
        self.declare_parameter("front_z_max", 2.0)
        self.declare_parameter("ground_percentile", 0.10)
        self.declare_parameter("warning_height", 0.07)
        self.declare_parameter("critical_height", 0.28)
        self.declare_parameter("max_slope", 0.45)
        self.declare_parameter("max_roughness", 0.06)
        self.declare_parameter("min_valid_points", 30)
        self.declare_parameter("source_switch_timeout", 2.0)
        self.declare_parameter("source_failure_cooldown", 2.0)
        self.declare_parameter("source_future_tolerance", 0.10)
        self.declare_parameter("source_geometry_failure_frames", 5)
        self.declare_parameter("ground_prior_max_age", 3.0)
        self.declare_parameter("ground_prior_max_consecutive_conflicts", 3)
        self.declare_parameter("ground_prior_max_height_shift", 0.10)
        self.declare_parameter("grid_cell_size", 0.05)
        self.declare_parameter("ground_height_bin", 0.03)
        self.declare_parameter("pit_depth_threshold", 0.07)
        self.declare_parameter("wall_height_threshold", 0.23)
        self.declare_parameter("bar_min_clearance", 0.18)
        self.declare_parameter("min_connected_region_cells", 4)
        self.declare_parameter("min_connected_region_points", 16)
        self.declare_parameter("clear_ground_corridor_half_width", 0.25)
        self.declare_parameter("clear_ground_required_distance", 0.80)
        self.declare_parameter("clear_ground_max_gap", 0.15)
        self.declare_parameter("clear_ground_min_lateral_fraction", 0.25)

        # Validate the configured geometry as one contract.  Continuing with a corrected 10 cm
        # ROI after a typo is unsafe because downstream nodes cannot tell that the requested
        # field of view was not actually used.
        validate_terrain_parameters(
            {name: self.get_parameter(name).value for name in TERRAIN_PARAMETER_NAMES}
        )

        override_topic = str(self.get_parameter("input_topic").value)
        candidates = list(self.get_parameter("input_topic_candidates").value)
        self.topics = (
            [override_topic]
            if override_topic
            else list(dict.fromkeys(candidates))
        )
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.transform_timeout = max(
            0.0, float(self.get_parameter("transform_timeout").value)
        )
        self.transform_max_points = int(
            self.get_parameter("transform_max_points").value
        )
        self.max_points = max(1, int(self.get_parameter("max_points").value))
        self.nav2_cloud_max_points = max(
            1, int(self.get_parameter("nav2_cloud_max_points").value)
        )
        self.nav2_obstacle_min_height = max(
            0.0,
            float(
                self.get_parameter(
                    "nav2_obstacle_min_height_above_ground"
                ).value
            ),
        )
        self.x_min = float(self.get_parameter("front_x_min").value)
        self.x_max = float(self.get_parameter("front_x_max").value)
        self.y_half = max(
            0.0, float(self.get_parameter("lateral_half_width").value)
        )
        self.z_min = float(self.get_parameter("front_z_min").value)
        self.z_max = float(self.get_parameter("front_z_max").value)
        if self.z_max <= self.z_min:
            self.get_logger().warning(
                "front_z_max must exceed front_z_min; using a 3.0 m vertical ROI"
            )
            self.z_max = self.z_min + 3.0
        self.ground_percentile = float(
            self.get_parameter("ground_percentile").value
        )
        self.warning_height = float(self.get_parameter("warning_height").value)
        self.critical_height = float(self.get_parameter("critical_height").value)
        self.max_slope = float(self.get_parameter("max_slope").value)
        self.max_roughness = float(self.get_parameter("max_roughness").value)
        self.min_points = max(
            1, int(self.get_parameter("min_valid_points").value)
        )
        self.source_switch_timeout = max(
            0.1, float(self.get_parameter("source_switch_timeout").value)
        )
        self.source_failure_cooldown = max(
            0.1, float(self.get_parameter("source_failure_cooldown").value)
        )
        self.source_future_tolerance = float(
            self.get_parameter("source_future_tolerance").value
        )
        self.source_geometry_failure_frames = int(
            self.get_parameter("source_geometry_failure_frames").value
        )
        # ``ground_height_prior`` is only a short-lived aid for frames where a wall or platform
        # hides the floor.  It is not a calibration constant: body-height changes, a new TF, or a
        # replay seek make an old base_link-relative height physically wrong.  Age bounds its
        # lifetime; the conflict count bounds how many consecutive translated floors may remain
        # UNKNOWN before the node deliberately reacquires the new level.
        self.ground_prior_max_age = float(
            self.get_parameter("ground_prior_max_age").value
        )
        self.ground_prior_max_consecutive_conflicts = int(
            self.get_parameter("ground_prior_max_consecutive_conflicts").value
        )
        self.ground_prior_max_height_shift = float(
            self.get_parameter("ground_prior_max_height_shift").value
        )
        self.grid_cell_size = max(
            0.02, float(self.get_parameter("grid_cell_size").value)
        )
        self.ground_height_bin = max(
            0.01, float(self.get_parameter("ground_height_bin").value)
        )
        self.pit_depth_threshold = max(
            0.03, float(self.get_parameter("pit_depth_threshold").value)
        )
        self.wall_height_threshold = max(
            0.10, float(self.get_parameter("wall_height_threshold").value)
        )
        self.bar_min_clearance = max(
            self.warning_height,
            float(self.get_parameter("bar_min_clearance").value),
        )
        self.min_connected_region_cells = max(
            2, int(self.get_parameter("min_connected_region_cells").value)
        )
        self.min_connected_region_points = max(
            4, int(self.get_parameter("min_connected_region_points").value)
        )
        # CLEAR 是允许 Nav2 继续前进的安全结论，因此不能仅凭“整帧总点数很多”批准。
        # 下列四项描述中央落脚通道必须被深度/三维雷达连续看见多远，以及最多容忍多宽
        # 的回波缺带。缺带返回 UNKNOWN，而不是凭空把无回波猜成 PIT。
        self.clear_ground_corridor_half_width = float(
            self.get_parameter("clear_ground_corridor_half_width").value
        )
        self.clear_ground_required_distance = float(
            self.get_parameter("clear_ground_required_distance").value
        )
        self.clear_ground_max_gap = float(
            self.get_parameter("clear_ground_max_gap").value
        )
        self.clear_ground_min_lateral_fraction = float(
            self.get_parameter("clear_ground_min_lateral_fraction").value
        )
        if self.x_max <= self.x_min:
            self.get_logger().warning(
                "front_x_max must exceed front_x_min; using a 0.10 m ROI"
            )
            self.x_max = self.x_min + 0.10

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # ``latest_cloud`` keeps its source beside the message.  A candidate may provisionally own
        # this slot after cheap metadata validation, but it only refreshes the healthy-source clock
        # after XYZ decoding and the timestamped TF both succeed in ``processing_callback``.
        self.latest_cloud = None
        self.last_processed_stamp = None
        # Header watermark belongs to the currently selected source. Duplicate or
        # out-of-order clouds must not replace ``latest_cloud`` or refresh ownership,
        # otherwise one physical scan can be counted as several geometry frames.
        self.last_received_source_stamp = None
        self.active_topic = None
        self.last_active_cloud_time = None
        self.last_candidate_cloud_time = None
        # Source arbitration, header watermarks, cooldown deadlines and the ground prior all use
        # ROS time.  Remember the last observed epoch independently of source health so a bag seek
        # is detected *before* an old future timestamp/cooldown can reject the first new frame.
        self.last_ros_time_ns = None
        # Decode/TF failures quarantine only the offending candidate.  This lets a correctly wired
        # backup camera/lidar take over instead of being suppressed by a high-rate broken topic.
        self.source_cooldown_until = {}
        # Finite XYZ and a valid TF still do not prove that a sensor sees the forward traversal
        # corridor.  Count consecutive sparse/UNKNOWN geometry frames for the active session; the
        # YAML threshold deliberately exceeds one so body pitch or one dropped depth frame cannot
        # flap between otherwise healthy sensors.
        self.consecutive_geometry_failures = 0
        # 只由明确 CLEAR 帧更新，用于近场被墙/平台完全遮挡时防止地面层翻转。
        # 它属于同一传感器会话的短期几何先验，不含场地坐标或障碍标签。
        self.ground_height_prior = None
        self.ground_height_prior_time = None
        self.ground_prior_conflict_count = 0
        self.features_pub = self.create_publisher(
            Float32MultiArray, "/terrain/features", 10
        )
        self.typed_features_pub = self.create_publisher(
            TerrainFeatures, "/terrain/features_stamped", 10
        )
        self.obstacle_cloud_pub = self.create_publisher(
            PointCloud2, "/perception/obstacle_points", qos_profile_sensor_data
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._cloud_subscriptions = [
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, source=topic: self.cloud_callback(msg, source),
                qos_profile_sensor_data,
            )
            for topic in self.topics
        ]
        processing_hz = min(
            30.0, max(0.5, float(self.get_parameter("processing_hz").value))
        )
        self.create_timer(1.0 / processing_hz, self.processing_callback)
        self.get_logger().info(
            f"Terrain analyzer: {self.topics} -> {self.target_frame} at "
            f"{processing_hz:.1f} Hz"
        )

    def cloud_callback(self, msg: PointCloud2, source: str) -> None:
        """只缓存最新帧；处理速度落后时主动丢旧帧，避免决策使用过期环境。"""
        now = self.get_clock().now()
        self._observe_ros_epoch(now)
        now_seconds = now.nanoseconds * 1e-9
        cooldown_until = self.source_cooldown_until.get(source, float("-inf"))
        if now_seconds < cooldown_until:
            return
        # Drop expired entries so a repeatedly unplugged/replugged sensor cannot grow this mapping.
        self.source_cooldown_until = {
            topic: deadline
            for topic, deadline in self.source_cooldown_until.items()
            if deadline > now_seconds
        }
        # Validate metadata before source arbitration.  Otherwise a continuously published
        # empty cloud or a cloud without floating XYZ fields can monopolize the preferred topic
        # and prevent a healthy secondary depth camera/3-D lidar from ever taking over.
        if not point_cloud_message_contract_valid(msg):
            self.get_logger().warning(
                f"Ignoring invalid PointCloud2 contract from {source}",
                throttle_duration_sec=2.0,
            )
            # If a once-healthy driver changes to a corrupt field layout, waiting for the ordinary
            # silence timeout unnecessarily blocks a backup sensor.  Quarantine malformed traffic
            # immediately; a single-sensor installation retries it after the configured cooldown.
            self._mark_source_unhealthy(source)
            return
        if not source_stamp_is_plausible(
            msg.header,
            now_seconds,
            self.source_switch_timeout,
            self.source_future_tolerance,
        ):
            self.get_logger().warning(
                f"Ignoring stale/future PointCloud2 from {source}",
                throttle_duration_sec=2.0,
            )
            return
        ownership_reference = self.last_active_cloud_time or self.last_candidate_cloud_time
        active_age = (
            float("inf")
            if ownership_reference is None
            else (now - ownership_reference).nanoseconds / 1e9
        )
        if not should_accept_source(
            self.active_topic,
            source,
            active_age,
            self.source_switch_timeout,
        ):
            return
        # 锁定首个有效点云源；当前源超时后允许其他默认话题接管。
        if source != self.active_topic:
            self.active_topic = source
            self.last_processed_stamp = None
            self.last_received_source_stamp = None
            self.last_active_cloud_time = None
            self.last_candidate_cloud_time = None
            self.consecutive_geometry_failures = 0
            # A replay/simulator clock reset starts a new physical observation
            # session. Keeping the previous ground prior could shift all new relative
            # heights and turn flat ground into a step until many CLEAR frames arrive.
            self._reset_ground_prior()
            self.get_logger().info(f"Using point-cloud topic {source}")
        if not source_stamp_strictly_advances(
            msg.header, self.last_received_source_stamp
        ):
            self.get_logger().warning(
                f"Ignoring duplicate/out-of-order PointCloud2 from {source}",
                throttle_duration_sec=2.0,
            )
            return
        self.last_received_source_stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        self.last_candidate_cloud_time = now
        self.latest_cloud = (msg, source)

    def processing_callback(self) -> None:
        """处理一帧未见过的点云；每个传感器时间戳最多处理一次。"""
        # A /clock rewind may occur between the last subscriber callback and this timer.  Observe
        # it before reading ``latest_cloud`` so an old-epoch pending frame is never processed.
        if self._observe_ros_epoch(self.get_clock().now()):
            return
        pending = self.latest_cloud
        if pending is None:
            return
        msg, source = pending
        stamp = (msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp
        transformed = self._xyz_in_target_frame(msg)
        if transformed is None:
            self._mark_source_unhealthy(source)
            return
        # Decoding and TF are necessary but not sufficient source-health evidence.  The sensor must
        # also contain enough forward geometry for a conservative valid classification below.
        processing_time = self.get_clock().now()
        if self._observe_ros_epoch(processing_time):
            return
        self._expire_ground_prior(processing_time)
        header, xyz = transformed
        # 先保留较完整的分析 ROI。这里使用 max_points 而不是 Nav2 的发布上限，避免为了
        # 节省 costmap 带宽而同时降低地面拟合、细横杆和窄立柱的分类支撑点数。
        analysis_points = filter_roi_points(
            xyz,
            self.x_min,
            self.x_max,
            self.y_half,
            self.max_points,
            z_min=self.z_min,
            z_max=self.z_max,
        )
        result = compute_terrain_features(
            xyz,
            self.x_min,
            self.x_max,
            self.y_half,
            self.max_points,
            self.ground_percentile,
            self.critical_height,
            self.max_slope,
            self.max_roughness,
            self.min_points,
            z_min=self.z_min,
            z_max=self.z_max,
        )
        if result is None:
            self._record_geometry_failure(source, "insufficient forward ROI points")
            self._publish_diagnostic(
                DiagnosticStatus.WARN,
                "Insufficient terrain points",
                len(analysis_points),
                {},
            )
            return
        features, valid_points = result
        geometry = analyze_terrain_geometry(
            analysis_points,
            cell_size=self.grid_cell_size,
            ground_bin_size=self.ground_height_bin,
            step_height=self.warning_height,
            pit_depth=self.pit_depth_threshold,
            wall_height=self.wall_height_threshold,
            bar_min_clearance=self.bar_min_clearance,
            min_cells=max(8, self.min_points // 3),
            min_region_cells=self.min_connected_region_cells,
            min_region_points=self.min_connected_region_points,
            ground_height_prior=self.ground_height_prior,
            ground_prior_max_height_shift=self.ground_prior_max_height_shift,
            clear_ground_corridor_half_width=self.clear_ground_corridor_half_width,
            clear_ground_start_x=self.x_min,
            clear_ground_required_distance=self.clear_ground_required_distance,
            clear_ground_max_gap=self.clear_ground_max_gap,
            clear_ground_min_lateral_fraction=self.clear_ground_min_lateral_fraction,
        )
        if geometry.valid:
            self.consecutive_geometry_failures = 0
            self.last_active_cloud_time = processing_time
            self.last_candidate_cloud_time = None
        self._update_ground_prior(geometry, processing_time)
        # 代价地图只接收相对局部地面凸起的点。平地和可通行坡面不会再因为 base_link
        # 高度或坡度而被标成障碍；几何无效时输出空云，同时安全评估保持 STOP。
        nav2_points = navigation_obstacle_points(
            analysis_points,
            geometry,
            minimum_height_above_ground=self.nav2_obstacle_min_height,
            maximum_points=self.nav2_cloud_max_points,
        )
        self.obstacle_cloud_pub.publish(
            # sensor_msgs_py 原生接受 float32 ndarray。不要先 ``tolist()``：3000 个点会
            # 额外构造约 12000 个 Python list/float 对象，增加 RK3588 的瞬时内存、GC
            # 和序列化延迟；直接传连续数组还能走 create_cloud 的快速内存视图路径。
            point_cloud2.create_cloud_xyz32(
                header, np.ascontiguousarray(nav2_points, dtype=np.float32)
            )
        )
        if geometry.valid:
            # 旧九字段继续由原算法提供，扩展字段和强类型话题承载新几何合同。
            features[FEATURE_PIT_DEPTH] = geometry.pit_depth
            features[FEATURE_SLOPE_ROLL] = geometry.slope_roll
            features[FEATURE_OBSTACLE_TYPE] = float(geometry.obstacle_type)
            features[FEATURE_CONFIDENCE] = geometry.confidence
            features[FEATURE_WIDTH] = geometry.width
            features[FEATURE_CLEARANCE_HEIGHT] = geometry.clearance_height
        self.features_pub.publish(Float32MultiArray(data=features))
        typed = TerrainFeatures()
        typed.header = header
        typed.valid = bool(geometry.valid)
        typed.obstacle_type = int(geometry.obstacle_type)
        typed.confidence = float(geometry.confidence)
        typed.ground_height = float(geometry.ground_height)
        # 强类型接口必须保持同一几何估计内部自洽：类别、尺寸和距离均来自稳健栅格
        # 分割。旧数组继续保留历史算法字段，供已有 rosbag/工具兼容使用。
        typed.obstacle_height = float(
            geometry.obstacle_height
            if geometry.valid
            else features[FEATURE_FRONTAL_HEIGHT]
        )
        typed.pit_depth = float(geometry.pit_depth)
        typed.slope_pitch = float(geometry.slope_pitch)
        typed.slope_roll = float(geometry.slope_roll)
        typed.roughness = float(max(features[FEATURE_ROUGHNESS], geometry.roughness))
        typed.distance = float(
            geometry.distance if geometry.valid else features[FEATURE_LOOKAHEAD]
        )
        typed.lateral_offset = float(geometry.lateral_offset)
        typed.width = float(geometry.width)
        typed.structure_heading = float(geometry.structure_heading)
        typed.structure_heading_confidence = float(
            geometry.structure_heading_confidence
        )
        typed.clearance_height = float(geometry.clearance_height)
        typed.valid_points = int(valid_points)
        self.typed_features_pub.publish(typed)
        obstacle_height = features[FEATURE_OBSTACLE_HEIGHT]
        slope = features[FEATURE_GROUND_SLOPE]
        roughness = features[FEATURE_ROUGHNESS]
        level = DiagnosticStatus.OK
        message = "Terrain passable"
        if obstacle_height >= self.critical_height or abs(slope) > self.max_slope:
            level, message = DiagnosticStatus.ERROR, "Critical terrain"
        elif obstacle_height >= self.warning_height or roughness > self.max_roughness:
            level, message = DiagnosticStatus.WARN, "Step or rough terrain detected"
        self._publish_diagnostic(
            level,
            message,
            valid_points,
            {
                "obstacle_height_m": obstacle_height,
                "slope": slope,
                "roughness_m": roughness,
            },
        )
        if not geometry.valid:
            # Publish the typed UNKNOWN result first so downstream safety remains fail-closed, then
            # release this source only after the configured consecutive-frame debounce expires.
            self._record_geometry_failure(source, "no valid forward terrain geometry")

    def _observe_ros_epoch(self, now: Time) -> bool:
        """Reset every ROS-time-bound sensor state before accepting data after a rewind.

        Cooldown deadlines and receive/processing watermarks from a later rosbag/Gazebo epoch must
        not suppress the first frame after a seek.  The observer is called by both the subscription
        and timer entry points; equal time while simulation is paused is not a rewind.
        """
        current_ns = int(now.nanoseconds)
        rewound = (
            self.last_ros_time_ns is not None
            and current_ns < self.last_ros_time_ns
        )
        if rewound:
            self._reset_sensor_epoch()
            self.get_logger().warning(
                "ROS clock moved backward; reset point-cloud source epoch",
                throttle_duration_sec=2.0,
            )
        self.last_ros_time_ns = current_ns
        return rewound

    def _reset_sensor_epoch(self) -> None:
        """Forget state whose meaning is limited to one monotonic ROS-time epoch."""
        self.latest_cloud = None
        self.last_processed_stamp = None
        self.last_received_source_stamp = None
        self.active_topic = None
        self.last_active_cloud_time = None
        self.last_candidate_cloud_time = None
        self.source_cooldown_until.clear()
        self.consecutive_geometry_failures = 0
        self._reset_ground_prior()

    def _record_geometry_failure(self, source: str, reason: str) -> None:
        """Debounce geometrically unusable frames, then quarantine the active source.

        A finite cloud outside the configured ROI, or one that cannot establish valid ground/
        obstacle geometry, is not healthy merely because decoding and TF succeeded.  Several such
        frames in sequence identify a wrong optical axis, blocked lens or unsuitable fallback
        topic.
        """
        if source != self.active_topic:
            return
        self.consecutive_geometry_failures += 1
        if self.consecutive_geometry_failures < self.source_geometry_failure_frames:
            return
        self.get_logger().warning(
            f"Point-cloud source {source} produced "
            f"{self.consecutive_geometry_failures} unusable geometry frames "
            f"({reason}); trying a backup",
            throttle_duration_sec=2.0,
        )
        self._mark_source_unhealthy(source)

    def _mark_source_unhealthy(self, source: str) -> None:
        """Cooldown a source after decoding/TF failure and release it for healthy failover.

        ``source_failure_cooldown`` is longer than one processing period so a high-rate malformed
        topic cannot reacquire before the backup's next frame.  With one sensor,
        the same topic is automatically retried after cooldown.
        """
        now = self.get_clock().now()
        if self._observe_ros_epoch(now):
            # Rewind already discarded the old-epoch frame and all source ownership.  Starting a
            # cooldown in the new epoch would incorrectly punish whichever source arrives first.
            return
        self.source_cooldown_until[source] = (
            now.nanoseconds * 1e-9 + self.source_failure_cooldown
        )
        if source != self.active_topic:
            return
        self.get_logger().warning(
            f"Point-cloud source {source} is unusable; allowing backup source takeover",
            throttle_duration_sec=2.0,
        )
        self.active_topic = None
        self.last_active_cloud_time = None
        self.last_candidate_cloud_time = None
        self.last_processed_stamp = None
        self.last_received_source_stamp = None
        self.latest_cloud = None
        self.consecutive_geometry_failures = 0
        # Ground height belongs to the old source/TF calibration and must not cross a failover.
        self._reset_ground_prior()

    def _reset_ground_prior(self) -> None:
        """Forget all state tied to a previously observed base_link-relative floor."""
        self.ground_height_prior = None
        self.ground_height_prior_time = None
        self.ground_prior_conflict_count = 0

    def _expire_ground_prior(self, now: Time) -> None:
        """Expire a stale or future-dated floor prior before classifying the current cloud.

        ``ground_prior_max_age`` is measured in ROS time, not wall time, so paused simulation does
        not age the model.  A negative age indicates a rosbag/Gazebo clock rewind and is treated as
        a new observation session.  Increase the age only if a labelled bag shows a long, genuine
        floor occlusion; decrease it to limit how long changed TF/body height affects geometry.
        """
        if self.ground_height_prior is None:
            return
        if self.ground_height_prior_time is None:
            self._reset_ground_prior()
            return
        age = (now - self.ground_height_prior_time).nanoseconds * 1e-9
        if age < 0.0 or age > self.ground_prior_max_age:
            self.get_logger().warning(
                "Ground-height prior expired; reacquiring from current point cloud",
                throttle_duration_sec=2.0,
            )
            self._reset_ground_prior()

    def _update_ground_prior(self, geometry, now: Time) -> None:
        """Update, retain, or invalidate the short-term floor-height state.

        The geometry core deliberately does not mutate state.  A broad translated floor raises
        ``ground_reference_conflict`` and remains UNKNOWN; this method requires consecutive frames
        before dropping the old prior.  Raise ``ground_prior_max_consecutive_conflicts`` when
        isolated gait impacts cause false resets, and lower it when a commanded body-height change
        takes too long to recover.  Always tune it together with ``ground_prior_max_height_shift``.
        """
        if geometry.ground_reference_conflict:
            # The core raises this only for a broad, observed plane shifted as a whole.
            # Keep initial frames UNKNOWN so one vibration cannot erase a useful prior.
            # A persistent shift likely means commanded body height or recalibrated TF; discard the
            # prior after the configured count so the next CLEAR frame establishes the new height.
            self.ground_prior_conflict_count += 1
            if (
                self.ground_prior_conflict_count
                >= self.ground_prior_max_consecutive_conflicts
            ):
                self.get_logger().warning(
                    "Ground-height prior conflicts with consecutive full-floor observations; "
                    "reacquiring the current ground level",
                    throttle_duration_sec=2.0,
                )
                self._reset_ground_prior()
        elif geometry.valid and geometry.obstacle_type == CLEAR:
            if self.ground_height_prior is None:
                self.ground_height_prior = float(geometry.ground_height)
            else:
                self.ground_height_prior = (
                    0.90 * float(self.ground_height_prior)
                    + 0.10 * float(geometry.ground_height)
                )
            self.ground_height_prior_time = now
            self.ground_prior_conflict_count = 0
        else:
            # A real obstacle or a sparse/invalid frame says nothing about whether the stored floor
            # level is wrong.  It breaks the *consecutive translated-floor* sequence but leaves the
            # age-limited prior available for the next occluded wall frame.
            self.ground_prior_conflict_count = 0

    def _xyz_in_target_frame(self, msg: PointCloud2):
        """读取 XYZ 并按采样时刻变换，忽略不相关的厂商扩展字段。"""
        try:
            xyz = point_cloud2.read_points_numpy(
                msg, field_names=["x", "y", "z"], skip_nans=True
            )
        except (AssertionError, TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().warning(f"Invalid PointCloud2 XYZ layout: {exc}")
            return None
        xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
        # A non-empty cloud may encode every no-return as NaN/Inf.  Such a stream cannot
        # support terrain geometry and must not keep ownership over a usable backup sensor.
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if not len(xyz):
            self.get_logger().warning("PointCloud2 contains no finite XYZ samples")
            return None
        # 先降采样再做 TF。后续仍会按 base_link 前向 ROI 和 max_points 二次筛选；这一层
        # 只负责限制全图变换成本，不假设任何厂商坐标轴或图像宽高。
        xyz = bounded_point_sample(xyz, self.transform_max_points)
        if not self.target_frame or msg.header.frame_id == self.target_frame:
            return msg.header, np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.transform_timeout),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            transformed = transform_xyz(
                xyz,
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = self.target_frame
            return header, transformed
        except TransformException as exc:
            self.get_logger().warning(
                f"Waiting for point-cloud TF {msg.header.frame_id} -> "
                f"{self.target_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        except ValueError as exc:
            self.get_logger().warning(f"Invalid point-cloud transform: {exc}")
            return None

    def _publish_diagnostic(
        self, level: int, message: str, points: int, values: dict
    ) -> None:
        """把可机读状态发布到标准 ``/diagnostics``，供监控和 rosbag 记录。"""
        status = DiagnosticStatus()
        status.level = level
        status.name = "quadruped/terrain_analyzer"
        status.hardware_id = "terrain_sensor"
        status.message = message
        status.values = [KeyValue(key="valid_points", value=str(points))]
        status.values.extend(
            KeyValue(key=key, value=f"{value:.4f}")
            for key, value in values.items()
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)


def main(args=None):
    """运行有算力上限的点云地形分析节点。"""
    rclpy.init(args=args)
    node = TerrainAnalyzer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 避免 launch 转发的第二次 SIGINT 在资源销毁阶段打印 traceback。
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""相机—点云时间同步与保守证据融合节点。

职责
----
在有界小队列中按 Header 选择时间差最小的一对观测，并发布一条原子融合消息。点云始终
负责高度、坑深、坡度和净空；OpenCV 只能提高一致类别的置信度或辅助细分横杆/立柱，不能
凭单目图像批准越障。相机超时后明确降级为纯点云结果。

真机标定入口
------------
同步、队列和置信度参数位于 ``config/vision.yaml`` 的 ``perception_fusion`` 段。先检查
两路 Header 是否来自同一时钟，再从 rosbag 报告的 ``time_skew`` 分布选择 ``sync_slop``；
窗口应略高于正常抖动上界，而不是靠不断放大掩盖驱动时钟错误。修改后必须在运动 bag 中
确认视觉框与点云仍指向同一物体，完整步骤见 ``instruction.txt`` 第五节。

安全边界
--------
超过同步窗口、字段非法或空间不相交的证据不会互相确认。放大窗口会把运动前后的不同
障碍错配；降低视觉置信度会增加误确认，两者都必须用独立验证 bag 复测。
"""

from collections import deque
import math
import signal

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from quadruped_interfaces.msg import FusedObstacle, TerrainFeatures, VisionObstacle

from quadruped_perception.parameter_validation import (
    FUSION_PARAMETER_NAMES,
    validate_fusion_parameters,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


VISION_TO_GEOMETRY = {
    VisionObstacle.POLES: FusedObstacle.POLE,
    VisionObstacle.HEIGHT_BAR: FusedObstacle.BAR,
    VisionObstacle.WALL: FusedObstacle.WALL,
}


def stamp_seconds(header) -> float:
    """把 ROS Header 转为秒；零时间戳保留为 0，便于诊断错误驱动。"""
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def find_synchronized_pair(terrain_queue, vision_queue, sync_slop: float):
    """从两个小队列中选择全局时间差最小且未超窗的一对消息。

    ROS 2 不保证不同传感器回调严格按时间戳到达。只拿“最新点云”寻找图像会在网络
    抖动或驱动批量发布时漏掉本来可用的旧配对；全局搜索的上限由 queue_size 限制，
    默认仅 10×10 次比较，对 RK3588 可忽略。
    """
    limit = max(0.0, float(sync_slop))
    candidates = []
    for terrain in terrain_queue:
        terrain_time = stamp_seconds(terrain.header)
        if not math.isfinite(terrain_time) or terrain_time <= 0.0:
            continue
        for vision in vision_queue:
            vision_time = stamp_seconds(vision.header)
            if not math.isfinite(vision_time) or vision_time <= 0.0:
                continue
            skew = abs(vision_time - terrain_time)
            if skew <= limit:
                # 时间差相同时优先最新的一对，减少输出观测年龄。
                candidates.append(
                    (skew, -max(terrain_time, vision_time), terrain, vision)
                )
    if not candidates:
        return None
    skew, _, terrain, vision = min(
        candidates, key=lambda item: (item[0], item[1])
    )
    return terrain, vision, float(skew)


def terrain_fallback_ready(
    receive_seconds: float,
    now_seconds: float,
    maximum_wait: float,
) -> bool:
    """判断一帧点云是否已等待视觉足够久，可以按纯几何结果发布。

    相机是辅助证据，不应成为点云地形链的单点故障。等待窗口应略大于同步容差；非法
    时间或 ROS 时钟回拨时不贸然发布，由后续有效帧重新建立顺序。
    """
    values = (receive_seconds, now_seconds, maximum_wait)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    age = float(now_seconds) - float(receive_seconds)
    return maximum_wait > 0.0 and age >= maximum_wait


def terrain_observation_valid(terrain) -> bool:
    """独立复核点云生产者的有效位、类别、范围和全部连续字段。"""
    metrics = (
        terrain.confidence,
        terrain.ground_height,
        terrain.obstacle_height,
        terrain.pit_depth,
        terrain.slope_pitch,
        terrain.slope_roll,
        terrain.roughness,
        terrain.distance,
        terrain.lateral_offset,
        terrain.width,
        terrain.structure_heading,
        terrain.structure_heading_confidence,
        terrain.clearance_height,
    )
    return (
        bool(terrain.valid)
        and TerrainFeatures.CLEAR
        <= int(terrain.obstacle_type)
        <= TerrainFeatures.POLE
        and all(math.isfinite(float(value)) for value in metrics)
        and 0.0 <= float(terrain.confidence) <= 1.0
        and float(terrain.obstacle_height) >= 0.0
        and float(terrain.pit_depth) >= 0.0
        and float(terrain.roughness) >= 0.0
        and float(terrain.distance) >= 0.0
        and float(terrain.width) >= 0.0
        and 0.0 <= float(terrain.structure_heading_confidence) <= 1.0
        and float(terrain.clearance_height) >= 0.0
        and int(terrain.valid_points) > 0
    )


def vision_observation_valid(vision, minimum_confidence: float) -> bool:
    """验证视觉类别、置信度和归一化框，拒绝 NaN 与退化框。"""
    if vision is None:
        return False
    metrics = (
        vision.confidence,
        vision.center_x,
        vision.center_y,
        vision.width,
        vision.height,
    )
    threshold = float(minimum_confidence)
    if not math.isfinite(threshold):
        return False
    return (
        int(vision.obstacle_type) in VISION_TO_GEOMETRY
        and all(math.isfinite(float(value)) for value in metrics)
        and max(0.0, min(1.0, threshold)) <= float(vision.confidence) <= 1.0
        and 0.0 < float(vision.width) <= 1.0
        and 0.0 < float(vision.height) <= 1.0
        and float(vision.width) / 2.0
        <= float(vision.center_x)
        <= 1.0 - float(vision.width) / 2.0
        and float(vision.height) / 2.0
        <= float(vision.center_y)
        <= 1.0 - float(vision.height) / 2.0
    )


def vision_overlaps_forward_corridor(vision, center_margin: float) -> bool:
    """检查视觉框是否与机器人前向通行走廊相交。

    点云分析只覆盖 ``base_link`` 前方窄 ROI，而相机通常具有更宽视场。若画面最左侧的
    立柱与正前方台阶恰好同帧出现，仅凭时间同步就融合会把两个不同物体当作一个。
    在尚未获得相机内参/像素到点云投影前，采用归一化中心走廊是低成本保守约束：目标
    框只要与 ``[margin, 1-margin]`` 相交即可，不会苛刻要求中心完全位于走廊内。
    """
    if vision is None:
        return False
    margin = max(0.0, min(0.49, float(center_margin)))
    left = float(vision.center_x) - float(vision.width) / 2.0
    right = float(vision.center_x) + float(vision.width) / 2.0
    return right >= margin and left <= 1.0 - margin


def _finite_or_zero(value: float) -> float:
    """保持融合消息数值有限；有效性另由严格校验结果表达。"""
    numeric = float(value)
    return numeric if math.isfinite(numeric) else 0.0


def _nonnegative_finite_or_zero(value: float) -> float:
    """清理不可能为负的米制量，确保无效消息也适合记录和诊断。"""
    numeric = _finite_or_zero(value)
    return max(0.0, numeric)


def fuse_observations(
    terrain,
    vision,
    skew: float,
    vision_min_confidence: float,
    vision_center_margin: float = 0.15,
):
    """融合一对同步消息并返回强类型结果，保持点云几何的安全优先级。

    “时间接近”只证明两传感器看的是同一时刻，不证明看的是同一物体。因此视觉还要
    通过前向走廊约束；未来获得相机内外参后，可在此处替换为三维投影关联而不改变消息
    合同和下游规划接口。
    """
    result = FusedObstacle()
    result.header = terrain.header
    terrain_valid = terrain_observation_valid(terrain)
    result.obstacle_type = (
        int(terrain.obstacle_type) if terrain_valid else FusedObstacle.UNKNOWN
    )
    result.geometry_confirmed = terrain_valid
    result.vision_confirmed = False
    result.confidence = _finite_or_zero(
        terrain.confidence if terrain_valid else 0.0
    )
    result.obstacle_height = _nonnegative_finite_or_zero(
        terrain.obstacle_height
    )
    result.pit_depth = _nonnegative_finite_or_zero(terrain.pit_depth)
    result.slope_pitch = _finite_or_zero(terrain.slope_pitch)
    result.slope_roll = _finite_or_zero(terrain.slope_roll)
    result.roughness = _nonnegative_finite_or_zero(terrain.roughness)
    result.distance = _nonnegative_finite_or_zero(terrain.distance)
    # 横向偏移有方向，不能用 nonnegative 清洗；NaN/Inf 仍安全归零并由有效性校验拒绝。
    result.lateral_offset = _finite_or_zero(terrain.lateral_offset)
    result.width = _nonnegative_finite_or_zero(terrain.width)
    result.structure_heading = _finite_or_zero(terrain.structure_heading)
    result.structure_heading_confidence = _nonnegative_finite_or_zero(
        terrain.structure_heading_confidence
    )
    result.clearance_height = _nonnegative_finite_or_zero(
        terrain.clearance_height
    )
    result.time_skew = _nonnegative_finite_or_zero(abs(skew))
    result.valid_points = max(0, int(terrain.valid_points))
    if not vision_observation_valid(
        vision, vision_min_confidence
    ) or not vision_overlaps_forward_corridor(vision, vision_center_margin):
        return result
    visual_type = VISION_TO_GEOMETRY.get(int(vision.obstacle_type))
    if terrain_valid and terrain.obstacle_type in (
        TerrainFeatures.STEP,
        TerrainFeatures.WALL,
        TerrainFeatures.BAR,
        TerrainFeatures.POLE,
    ):
        # 视觉细分类还必须满足米制几何条件，不能把普通台阶仅凭像素外观改成横杆
        # 或立柱。横杆要求离地净空；立柱要求点云横向宽度较窄。
        compatible = visual_type == int(terrain.obstacle_type)
        if (
            visual_type == FusedObstacle.BAR
            and terrain.clearance_height >= 0.12
            and terrain.obstacle_height >= 0.10
        ):
            compatible = True
        elif (
            visual_type == FusedObstacle.POLE
            and 0.0 < terrain.width <= 0.25
            and terrain.obstacle_height >= 0.10
        ):
            compatible = True
        if compatible:
            # ``vision_confirmed`` 表示视觉和米制几何指向同一类别，而不只是“同一时刻
            # 有一个视觉框”。这个区别很重要：规划层会用该位决定视觉限速；冲突框若
            # 也置真，会把画面边缘的墙/立柱错误关联到正前方的 CLEAR 或台阶几何。
            result.vision_confirmed = True
            result.obstacle_type = visual_type
            result.confidence = min(
                1.0, 0.65 * terrain.confidence + 0.45 * vision.confidence
            )
        else:
            # 冲突视觉只能表示“未完成辅助复核”，不得惩罚已有米制支撑的点云。
            # 例如点云置信度 0.30 本来足以进入下游；乘 0.75 后会低于 0.25
            # 有效门，一个画面杂物就能把权威几何变成 STOP。保持原置信度并置
            # vision_confirmed=false，既不提升也不否决点云。
            result.confidence = float(terrain.confidence)
    return result


class PerceptionFusion(Node):
    """缓存少量消息并只融合时间差在阈值内的最近观测。"""

    def __init__(self, **node_kwargs):
        """建立两个有界输入队列以及融合结果/诊断发布器。

        节点不使用 message_filters，是为了明确控制乱序消息的配对、消费和丢弃规则；
        这样 rosbag 回放发生突发到达时，也不会重复使用同一帧证据。
        """
        super().__init__("perception_fusion", **node_kwargs)
        self.declare_parameter("sync_slop", 0.10)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("vision_min_confidence", 0.55)
        self.declare_parameter("vision_center_margin", 0.15)
        self.declare_parameter("terrain_only_timeout", 0.25)
        validate_fusion_parameters(
            {name: self.get_parameter(name).value for name in FUSION_PARAMETER_NAMES}
        )
        self.sync_slop = max(0.001, float(self.get_parameter("sync_slop").value))
        queue_size = max(2, int(self.get_parameter("queue_size").value))
        self.vision_min_confidence = min(
            1.0, max(0.0, float(self.get_parameter("vision_min_confidence").value))
        )
        self.vision_center_margin = min(
            0.49,
            max(0.0, float(self.get_parameter("vision_center_margin").value)),
        )
        # 必须至少给近似同步一个完整窗口；相机断流后仍能在安全评估超时前发布点云结果。
        self.terrain_only_timeout = max(
            self.sync_slop,
            float(self.get_parameter("terrain_only_timeout").value),
        )
        self.terrain_queue = deque(maxlen=queue_size)
        self.vision_queue = deque(maxlen=queue_size)
        self.terrain_receive_times = {}
        self.output_pub = self.create_publisher(
            FusedObstacle, "/perception/fused_obstacle", 10
        )
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(
            TerrainFeatures, "/terrain/features_stamped", self.terrain_callback, 10
        )
        self.create_subscription(
            VisionObstacle, "/vision/obstacle_stamped", self.vision_callback, 10
        )
        self.create_timer(0.05, self._publish_terrain_fallback)

    def terrain_callback(self, msg: TerrainFeatures) -> None:
        """缓存一条带采样时间的点云几何摘要并尝试配对。"""
        self.terrain_queue.append(msg)
        self.terrain_receive_times[id(msg)] = self.get_clock().now().nanoseconds * 1e-9
        self._prune_receive_times()
        self._try_pair()

    def vision_callback(self, msg: VisionObstacle) -> None:
        """缓存一条稳定视觉证据并尝试配对。"""
        self.vision_queue.append(msg)
        self._try_pair()

    def _try_pair(self) -> None:
        """发布至多一对同步观测，并丢弃已经消费的旧消息。"""
        if not self.terrain_queue or not self.vision_queue:
            return
        pair = find_synchronized_pair(
            self.terrain_queue, self.vision_queue, self.sync_slop
        )
        if pair is None:
            newest_terrain = stamp_seconds(self.terrain_queue[-1].header)
            newest_vision = stamp_seconds(self.vision_queue[-1].header)
            skew = abs(newest_terrain - newest_vision)
            self._diagnostic(
                DiagnosticStatus.WARN, "waiting for synchronized camera", skew
            )
            return
        terrain, vision, skew = pair
        fused = fuse_observations(
            terrain,
            vision,
            skew,
            self.vision_min_confidence,
            self.vision_center_margin,
        )
        self.output_pub.publish(fused)
        self._consume_through(terrain, vision)
        self._diagnostic(DiagnosticStatus.OK, "camera/cloud synchronized", skew)

    def _publish_terrain_fallback(self) -> None:
        """相机缺帧时延迟发布纯点云结果，保证视觉辅助不会阻断安全几何链。"""
        if not self.terrain_queue:
            return
        # 定时器与任一传感器回调可能交错；先再尝试一次配对，避免刚到达的图像被漏用。
        pair = find_synchronized_pair(
            self.terrain_queue, self.vision_queue, self.sync_slop
        )
        if pair is not None:
            self._try_pair()
            return
        terrain = self.terrain_queue[0]
        received = self.terrain_receive_times.get(id(terrain), float("nan"))
        now = self.get_clock().now().nanoseconds * 1e-9
        if not terrain_fallback_ready(received, now, self.terrain_only_timeout):
            return
        # vision=None 会保留点云几何和置信度，同时明确 vision_confirmed=false。
        self.output_pub.publish(
            fuse_observations(
                terrain, None, 0.0, self.vision_min_confidence
            )
        )
        self._consume_through(terrain, None)
        self._diagnostic(
            DiagnosticStatus.WARN,
            "camera unavailable; geometry-only fallback",
            0.0,
        )

    def _consume_through(self, terrain, vision) -> None:
        """消费已发布观测及更旧数据，保证同一帧不会重复贡献风险确认次数。"""
        terrain_time = stamp_seconds(terrain.header)
        self.terrain_queue = deque(
            (
                item
                for item in self.terrain_queue
                if stamp_seconds(item.header) > terrain_time
            ),
            maxlen=self.terrain_queue.maxlen,
        )
        if vision is not None:
            vision_cutoff = stamp_seconds(vision.header)
        else:
            # 已无可配图像；只保留未来仍可能与下一帧点云配对的较新视觉消息。
            vision_cutoff = terrain_time + self.sync_slop
        self.vision_queue = deque(
            (
                item
                for item in self.vision_queue
                if stamp_seconds(item.header) > vision_cutoff
            ),
            maxlen=self.vision_queue.maxlen,
        )
        self._prune_receive_times()

    def _prune_receive_times(self) -> None:
        """删除 deque 自动淘汰消息的接收时间，保持辅助字典有界。"""
        active_ids = {id(item) for item in self.terrain_queue}
        self.terrain_receive_times = {
            key: value
            for key, value in self.terrain_receive_times.items()
            if key in active_ids
        }

    def _diagnostic(self, level: int, message: str, skew: float) -> None:
        """通过标准 diagnostics 报告同步状态和当前时间差。"""
        status = DiagnosticStatus(
            level=level,
            name="quadruped/perception_fusion",
            hardware_id="camera_lidar_sync",
            message=message,
            values=[KeyValue(key="time_skew_s", value=f"{skew:.6f}")],
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)


def main(args=None):
    """启动相机—点云时间同步融合节点。"""
    rclpy.init(args=args)
    node = PerceptionFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

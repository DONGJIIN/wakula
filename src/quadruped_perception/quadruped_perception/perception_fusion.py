"""相机—点云近似时间同步与保守证据融合。

相机和深度驱动通常不是严格硬同步，本节点用消息头时间戳做有界近似同步。点云几何
始终是 STEP/PIT/WALL 的尺度依据；OpenCV 只能提高同类置信度或把已有高处几何细分为
横杆/立柱，不能凭单目图像独立批准越障。
"""

from collections import deque
import math
import signal

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from quadruped_interfaces.msg import FusedObstacle, TerrainFeatures, VisionObstacle
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


def fuse_observations(terrain, vision, skew: float, vision_min_confidence: float):
    """融合一对同步消息并返回强类型结果，保持点云几何的安全优先级。"""
    result = FusedObstacle()
    result.header = terrain.header
    result.obstacle_type = int(terrain.obstacle_type)
    result.geometry_confirmed = bool(terrain.valid)
    result.vision_confirmed = False
    result.confidence = float(terrain.confidence if terrain.valid else 0.0)
    result.obstacle_height = float(terrain.obstacle_height)
    result.pit_depth = float(terrain.pit_depth)
    result.slope_pitch = float(terrain.slope_pitch)
    result.slope_roll = float(terrain.slope_roll)
    result.roughness = float(terrain.roughness)
    result.distance = float(terrain.distance)
    result.width = float(terrain.width)
    result.clearance_height = float(terrain.clearance_height)
    result.time_skew = float(abs(skew))
    result.valid_points = int(terrain.valid_points)
    if vision is None or vision.confidence < vision_min_confidence:
        return result
    visual_type = VISION_TO_GEOMETRY.get(int(vision.obstacle_type))
    if visual_type is None:
        return result
    result.vision_confirmed = True
    if terrain.valid and terrain.obstacle_type in (
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
            and terrain.clearance_height >= 0.05
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
            result.obstacle_type = visual_type
            result.confidence = min(
                1.0, 0.65 * terrain.confidence + 0.45 * vision.confidence
            )
        else:
            # 冲突证据不改变几何类别，只降低置信度，交由后续帧继续确认。
            result.confidence = max(0.0, 0.75 * terrain.confidence)
    return result


class PerceptionFusion(Node):
    """缓存少量消息并只融合时间差在阈值内的最近观测。"""

    def __init__(self):
        super().__init__("perception_fusion")
        self.declare_parameter("sync_slop", 0.10)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("vision_min_confidence", 0.55)
        self.sync_slop = max(0.001, float(self.get_parameter("sync_slop").value))
        queue_size = max(2, int(self.get_parameter("queue_size").value))
        self.vision_min_confidence = min(
            1.0, max(0.0, float(self.get_parameter("vision_min_confidence").value))
        )
        self.terrain_queue = deque(maxlen=queue_size)
        self.vision_queue = deque(maxlen=queue_size)
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

    def terrain_callback(self, msg: TerrainFeatures) -> None:
        self.terrain_queue.append(msg)
        self._try_pair()

    def vision_callback(self, msg: VisionObstacle) -> None:
        self.vision_queue.append(msg)
        self._try_pair()

    def _try_pair(self) -> None:
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
            terrain, vision, skew, self.vision_min_confidence
        )
        self.output_pub.publish(fused)
        terrain_time = stamp_seconds(terrain.header)
        vision_time = stamp_seconds(vision.header)
        # 一对消息只能使用一次；同时丢弃更早观测，防止旧图重复增强后续点云。
        self.terrain_queue = deque(
            (
                item
                for item in self.terrain_queue
                if stamp_seconds(item.header) > terrain_time
            ),
            maxlen=self.terrain_queue.maxlen,
        )
        self.vision_queue = deque(
            (
                item
                for item in self.vision_queue
                if stamp_seconds(item.header) > vision_time
            ),
            maxlen=self.vision_queue.maxlen,
        )
        self._diagnostic(DiagnosticStatus.OK, "camera/cloud synchronized", skew)

    def _diagnostic(self, level: int, message: str, skew: float) -> None:
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

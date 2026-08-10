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


def fuse_observations(terrain, vision, skew: float, vision_min_confidence: float):
    """融合一对同步消息并返回强类型结果，保持点云几何的安全优先级。"""
    result = FusedObstacle()
    result.header = terrain.header
    result.obstacle_type = int(terrain.obstacle_type)
    result.geometry_confirmed = bool(terrain.valid)
    result.confidence = float(terrain.confidence if terrain.valid else 0.0)
    result.obstacle_height = float(terrain.obstacle_height)
    result.pit_depth = float(terrain.pit_depth)
    result.slope_pitch = float(terrain.slope_pitch)
    result.slope_roll = float(terrain.slope_roll)
    result.distance = float(terrain.distance)
    result.clearance_height = float(terrain.clearance_height)
    result.time_skew = float(abs(skew))
    if vision is None or vision.confidence < vision_min_confidence:
        return result
    visual_type = VISION_TO_GEOMETRY.get(int(vision.obstacle_type))
    if visual_type is None:
        return result
    if terrain.valid and terrain.obstacle_type in (
        TerrainFeatures.STEP,
        TerrainFeatures.WALL,
        TerrainFeatures.BAR,
        TerrainFeatures.POLE,
    ):
        # 横杆/立柱都必须已有正高度点云；视觉只负责细分类。
        if visual_type in (FusedObstacle.BAR, FusedObstacle.POLE):
            result.obstacle_type = visual_type
        if visual_type == int(terrain.obstacle_type):
            result.confidence = min(
                1.0, 0.65 * terrain.confidence + 0.45 * vision.confidence
            )
        else:
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
        terrain = self.terrain_queue[-1]
        terrain_time = stamp_seconds(terrain.header)
        candidates = [
            (abs(stamp_seconds(item.header) - terrain_time), item)
            for item in self.vision_queue
        ]
        skew, vision = min(candidates, key=lambda item: item[0])
        if not math.isfinite(skew) or skew > self.sync_slop:
            self._diagnostic(DiagnosticStatus.WARN, "waiting for synchronized camera", skew)
            return
        fused = fuse_observations(
            terrain, vision, skew, self.vision_min_confidence
        )
        self.output_pub.publish(fused)
        self.terrain_queue.clear()
        # 丢弃已配对时间之前的视觉帧，防止同一旧图重复增强多帧点云。
        self.vision_queue = deque(
            (item for item in self.vision_queue if stamp_seconds(item.header) > terrain_time),
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

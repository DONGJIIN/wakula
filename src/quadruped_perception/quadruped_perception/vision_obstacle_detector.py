"""Bounded OpenCV obstacle evidence for lidar/depth navigation fusion."""

from collections import Counter, deque
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from quadruped_perception.topic_selection import should_accept_source


DEFAULT_IMAGE_TOPICS = [
    "/camera/image_raw",
    "/camera/color/image_raw",
    "/camera/rgb/image_raw",
    "/camera/camera/color/image_raw",
    "/image_raw",
]
ColorFeature = Tuple[float, float, float, float, float]
ContourBox = Tuple[int, int, int, int, float]
HINT_CODES = {
    "none": 0.0,
    "poles": 1.0,
    "height_bar": 2.0,
    "wall": 3.0,
    "colored_obstacle": 4.0,
}


@dataclass(frozen=True)
class ObstacleEvidence:
    """One normalized image-space obstacle observation."""

    hint: str = "none"
    confidence: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0
    width: float = 0.0
    height: float = 0.0

    def as_array(self) -> List[float]:
        """Return a compact atomic ROS array representation."""
        return [
            HINT_CODES.get(self.hint, 0.0),
            self.confidence,
            self.center_x,
            self.center_y,
            self.width,
            self.height,
        ]


def contour_boxes(mask: np.ndarray, min_area: float) -> List[ContourBox]:
    """Extract valid external contour bounding boxes."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        boxes.append((x, y, width, height, area))
    return boxes


def largest_color_feature(mask: np.ndarray, min_area: float) -> ColorFeature:
    """Return area and normalized bounding box for the largest valid contour."""
    boxes = contour_boxes(mask, min_area)
    if not boxes:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    x, y, width, height, area = max(boxes, key=lambda box: box[4])
    image_height, image_width = mask.shape[:2]
    return (
        area,
        (x + width / 2.0) / image_width,
        (y + height / 2.0) / image_height,
        width / image_width,
        height / image_height,
    )


def evidence_from_boxes(
    hint: str,
    confidence: float,
    boxes: Sequence[ContourBox],
    image_width: int,
    image_height: int,
) -> ObstacleEvidence:
    """Create normalized evidence covering one or more related regions."""
    left = min(box[0] for box in boxes)
    top = min(box[1] for box in boxes)
    right = max(box[0] + box[2] for box in boxes)
    bottom = max(box[1] + box[3] for box in boxes)
    width = right - left
    height = bottom - top
    return ObstacleEvidence(
        hint=hint,
        confidence=float(np.clip(confidence, 0.0, 1.0)),
        center_x=(left + width / 2.0) / image_width,
        center_y=(top + height / 2.0) / image_height,
        width=width / image_width,
        height=height / image_height,
    )


def detect_obstacle_evidence(
    orange_mask: np.ndarray,
    blue_mask: np.ndarray,
    edge_mask: np.ndarray,
    min_area: float,
) -> ObstacleEvidence:
    """Combine color and contour geometry into a conservative raw hint."""
    image_height, image_width = orange_mask.shape[:2]
    image_area = float(image_height * image_width)
    orange_boxes = contour_boxes(orange_mask, min_area)
    blue_boxes = contour_boxes(blue_mask, min_area)

    # 优先使用颜色特征：比赛标志色比普通场景边缘更稳定。
    horizontal_blue = [
        box
        for box in blue_boxes
        if box[2] >= box[3] * 2.2 and box[4] / image_area >= 0.005
    ]
    if horizontal_blue:
        box = max(horizontal_blue, key=lambda item: item[4])
        confidence = min(0.98, 0.70 + box[4] / image_area * 4.0)
        return evidence_from_boxes(
            "height_bar", confidence, [box], image_width, image_height
        )

    tall_orange = [
        box
        for box in orange_boxes
        if box[3] >= box[2] * 1.5 and box[3] >= image_height * 0.15
    ]
    if len(tall_orange) >= 2:
        tall_orange.sort(key=lambda box: box[0])
        confidence = min(0.98, 0.72 + len(tall_orange) * 0.06)
        return evidence_from_boxes(
            "poles", confidence, tall_orange, image_width, image_height
        )

    # 当颜色受光照影响时，再使用 Canny 轮廓作为保守的几何补充。
    edge_boxes = contour_boxes(edge_mask, min_area)
    tall_edges = [
        box
        for box in edge_boxes
        if box[3] >= box[2] * 2.0
        and box[3] >= image_height * 0.20
        and image_width * 0.05 <= box[0] + box[2] / 2.0 <= image_width * 0.95
    ]
    if len(tall_edges) >= 2:
        tall_edges.sort(key=lambda box: box[0])
        separated = tall_edges[-1][0] - tall_edges[0][0] >= image_width * 0.12
        if separated:
            return evidence_from_boxes(
                "poles", 0.64, tall_edges, image_width, image_height
            )

    horizontal_edges = [
        box
        for box in edge_boxes
        if box[2] >= box[3] * 3.0
        and box[2] >= image_width * 0.30
        and box[1] + box[3] / 2.0 <= image_height * 0.65
    ]
    if horizontal_edges:
        box = max(horizontal_edges, key=lambda item: item[2])
        return evidence_from_boxes(
            "height_bar", 0.60, [box], image_width, image_height
        )

    wall_edges = [
        box
        for box in edge_boxes
        if box[2] >= image_width * 0.28
        and box[3] >= image_height * 0.15
        and box[1] + box[3] / 2.0 >= image_height * 0.45
    ]
    if wall_edges:
        box = max(wall_edges, key=lambda item: item[4])
        return evidence_from_boxes(
            "wall", 0.56, [box], image_width, image_height
        )

    colored_boxes = orange_boxes + blue_boxes
    if colored_boxes:
        box = max(colored_boxes, key=lambda item: item[4])
        if box[4] / image_area >= 0.03:
            return evidence_from_boxes(
                "colored_obstacle", 0.55, [box], image_width, image_height
            )
    return ObstacleEvidence()


def stabilize_evidence(
    history: Sequence[ObstacleEvidence], minimum_matches: int
) -> ObstacleEvidence:
    """Accept only a non-empty hint repeated across multiple recent frames."""
    # 多帧投票抑制反光、运动模糊和单帧噪声。
    hints = [item.hint for item in history if item.hint != "none"]
    if not hints:
        return ObstacleEvidence()
    hint, count = Counter(hints).most_common(1)[0]
    if count < minimum_matches:
        return ObstacleEvidence()
    matches = [item for item in history if item.hint == hint]
    confidence = float(np.mean([item.confidence for item in matches]))
    consistency = count / max(1, len(history))
    return ObstacleEvidence(
        hint=hint,
        # Repetition is already enforced by minimum_matches. Keep a small
        # consistency penalty without suppressing a valid 3-of-5 result.
        confidence=confidence * (0.8 + 0.2 * consistency),
        center_x=float(np.mean([item.center_x for item in matches])),
        center_y=float(np.mean([item.center_y for item in matches])),
        width=float(np.mean([item.width for item in matches])),
        height=float(np.mean([item.height for item in matches])),
    )


class VisionObstacleDetector(Node):
    """Publish temporally confirmed obstacle evidence from common camera topics."""

    def __init__(self):
        super().__init__("vision_obstacle_detector")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("image_topic_candidates", DEFAULT_IMAGE_TOPICS)
        self.declare_parameter("debug_mask_topic", "/vision/debug_mask")
        self.declare_parameter("publish_debug_mask", False)
        self.declare_parameter("processing_hz", 8.0)
        self.declare_parameter("resize_width", 640)
        self.declare_parameter("min_area_px", 300.0)
        self.declare_parameter("morphology_size", 5)
        self.declare_parameter("edge_low_threshold", 60)
        self.declare_parameter("edge_high_threshold", 160)
        self.declare_parameter("history_size", 5)
        self.declare_parameter("confirmation_frames", 3)
        self.declare_parameter("source_switch_timeout", 2.0)
        self.declare_parameter("orange_hsv_lower", [5, 80, 70])
        self.declare_parameter("orange_hsv_upper", [25, 255, 255])
        self.declare_parameter("blue_hsv_lower", [90, 70, 50])
        self.declare_parameter("blue_hsv_upper", [135, 255, 255])

        override_topic = str(self.get_parameter("image_topic").value)
        candidates = list(self.get_parameter("image_topic_candidates").value)
        self.image_topics = (
            [override_topic] if override_topic else list(dict.fromkeys(candidates))
        )
        self.publish_debug = bool(self.get_parameter("publish_debug_mask").value)
        self.resize_width = max(0, int(self.get_parameter("resize_width").value))
        self.min_area = max(1.0, float(self.get_parameter("min_area_px").value))
        self.edge_low = max(0, int(self.get_parameter("edge_low_threshold").value))
        self.edge_high = max(
            self.edge_low + 1,
            int(self.get_parameter("edge_high_threshold").value),
        )
        history_size = max(1, int(self.get_parameter("history_size").value))
        self.confirmation_frames = min(
            history_size,
            max(1, int(self.get_parameter("confirmation_frames").value)),
        )
        self.source_switch_timeout = max(
            0.1, float(self.get_parameter("source_switch_timeout").value)
        )
        self.evidence_history = deque(maxlen=history_size)
        kernel_size = max(1, int(self.get_parameter("morphology_size").value))
        kernel_size += 1 if kernel_size % 2 == 0 else 0
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        self.orange_lower = self._hsv_parameter("orange_hsv_lower")
        self.orange_upper = self._hsv_parameter("orange_hsv_upper")
        self.blue_lower = self._hsv_parameter("blue_hsv_lower")
        self.blue_upper = self._hsv_parameter("blue_hsv_upper")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.last_processed_stamp = None
        self.active_topic = None
        self.last_active_image_time = None
        self.feature_pub = self.create_publisher(
            Float32MultiArray, "/vision/color_features", 10
        )
        self.evidence_pub = self.create_publisher(
            Float32MultiArray, "/vision/obstacle_evidence", 10
        )
        self.hint_pub = self.create_publisher(String, "/vision/obstacle_hint", 10)
        self.mask_pub = (
            self.create_publisher(
                Image, str(self.get_parameter("debug_mask_topic").value), 1
            )
            if self.publish_debug
            else None
        )
        self._image_subscriptions = [
            self.create_subscription(
                Image,
                topic,
                lambda msg, source=topic: self.image_callback(msg, source),
                qos_profile_sensor_data,
            )
            for topic in self.image_topics
        ]
        processing_hz = min(
            30.0, max(0.5, float(self.get_parameter("processing_hz").value))
        )
        self.create_timer(1.0 / processing_hz, self.processing_callback)
        self.get_logger().info(
            f"OpenCV obstacle evidence: {self.image_topics} at {processing_hz:.1f} Hz"
        )

    def _hsv_parameter(self, name: str) -> np.ndarray:
        values = np.asarray(self.get_parameter(name).value, dtype=np.int32)
        return np.clip(values, 0, 255).astype(np.uint8)

    def image_callback(self, msg: Image, source: str) -> None:
        """Keep one newest frame so camera rate cannot create a backlog."""
        now = self.get_clock().now()
        active_age = (
            float("inf")
            if self.last_active_image_time is None
            else (now - self.last_active_image_time).nanoseconds / 1e9
        )
        if not should_accept_source(
            self.active_topic,
            source,
            active_age,
            self.source_switch_timeout,
        ):
            return
        # 同时存在多个默认图像话题时只选一个，失联后再自动切换。
        if source != self.active_topic:
            self.evidence_history.clear()
            self.last_processed_stamp = None
            self.active_topic = source
            self.get_logger().info(f"Using camera topic {source}")
        self.last_active_image_time = now
        self.latest_frame = (msg, source)

    def processing_callback(self) -> None:
        """Process one unseen frame and publish stable obstacle evidence."""
        if self.latest_frame is None:
            return
        msg, source = self.latest_frame
        stamp = (source, msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return
        if bgr.size == 0:
            return
        if self.resize_width and bgr.shape[1] > self.resize_width:
            scale = self.resize_width / float(bgr.shape[1])
            bgr = cv2.resize(
                bgr,
                (self.resize_width, max(1, round(bgr.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )

        # HSV 负责颜色，灰度 Canny 负责轮廓；两者不做神经网络推理。
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        orange_mask = self._clean_mask(
            cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        )
        blue_mask = self._clean_mask(
            cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        )
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edge_mask = cv2.Canny(gray, self.edge_low, self.edge_high)
        edge_mask = cv2.morphologyEx(edge_mask, cv2.MORPH_CLOSE, self.kernel)
        edge_mask = cv2.dilate(edge_mask, self.kernel, iterations=1)

        orange = largest_color_feature(orange_mask, self.min_area)
        blue = largest_color_feature(blue_mask, self.min_area)
        image_area = float(bgr.shape[0] * bgr.shape[1])
        self.feature_pub.publish(
            Float32MultiArray(
                data=[
                    orange[0] / image_area,
                    *orange[1:],
                    blue[0] / image_area,
                    *blue[1:],
                ]
            )
        )
        raw_evidence = detect_obstacle_evidence(
            orange_mask, blue_mask, edge_mask, self.min_area
        )
        # 只把稳定后的原子证据交给规划层，避免读取到不同帧的混合字段。
        self.evidence_history.append(raw_evidence)
        evidence = stabilize_evidence(
            self.evidence_history, self.confirmation_frames
        )
        self.evidence_pub.publish(Float32MultiArray(data=evidence.as_array()))
        self.hint_pub.publish(String(data=evidence.hint))

        if self.publish_debug:
            debug_mask = cv2.merge((blue_mask, edge_mask, orange_mask))
            debug_msg = self.bridge.cv2_to_imgmsg(debug_mask, encoding="bgr8")
            debug_msg.header = msg.header
            self.mask_pub.publish(debug_msg)

    def _clean_mask(self, mask: np.ndarray) -> np.ndarray:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)


def main(args=None):
    """Run the bounded-rate OpenCV perception node."""
    rclpy.init(args=args)
    node = VisionObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch 与终端可能同时发送 SIGINT，清理阶段再次中断也应正常退出。
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

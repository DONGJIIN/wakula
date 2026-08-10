"""使用轻量 OpenCV 为雷达/深度几何提供辅助障碍证据。

流水线为：限频取最新图像 → CLAHE 亮度归一化 → HSV 颜色掩膜与 Canny 轮廓 →
ROI/形态学去噪 → 几何启发式分类 → 多帧空间一致性确认。算法不恢复米制深度，
因此输出只允许规划层减速并请求点云复核，不能独立触发 STEP/CLIMB。

该实现面向 RK3588 的可解释、低算力基线。HSV、ROI 和像素面积均是相机相关参数，
更换镜头、安装角度或比赛场地光照后必须用 debug mask 与 rosbag 重新标定。
"""

from collections import Counter, deque
from dataclasses import dataclass
from itertools import combinations
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

from quadruped_interfaces.msg import VisionObstacle

from quadruped_perception.topic_selection import should_accept_source


DEFAULT_IMAGE_TOPICS = [
    "/camera/image_raw",
    "/camera/color/image_raw",
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
    """一条原子的归一化图像障碍观测。

    坐标原点位于图像左上角，中心和宽高均除以图像尺寸，因此合法范围是 0～1。
    ``confidence`` 是启发式一致性分数，不是统计校准后的真实概率。
    """

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
    """提取面积达标的最外层轮廓框，忽略孔洞以控制计算量。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        boxes.append((x, y, width, height, area))
    return boxes


def enhance_illumination(
    bgr: np.ndarray, clip_limit: float = 2.0, grid_size: int = 8
) -> np.ndarray:
    """仅增强 LAB 亮度通道，尽量不改变 HSV 检测依赖的颜色关系。"""
    grid_size = max(2, int(grid_size))
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lightness, channel_a, channel_b = cv2.split(lab)
    clahe = cv2.createCLAHE(
        clipLimit=max(0.1, float(clip_limit)),
        tileGridSize=(grid_size, grid_size),
    )
    lightness = clahe.apply(lightness)
    return cv2.cvtColor(
        cv2.merge((lightness, channel_a, channel_b)), cv2.COLOR_LAB2BGR
    )


def apply_detection_roi(
    mask: np.ndarray,
    top_ratio: float,
    bottom_ratio: float,
    side_margin_ratio: float,
) -> np.ndarray:
    """保留可配置前向画面区域，屏蔽天空、机身和镜头边缘。"""
    image_height, image_width = mask.shape[:2]
    top = int(np.clip(top_ratio, 0.0, 0.95) * image_height)
    bottom = int(np.clip(bottom_ratio, 0.05, 1.0) * image_height)
    margin = int(np.clip(side_margin_ratio, 0.0, 0.45) * image_width)
    if bottom <= top:
        return np.zeros_like(mask)
    result = np.zeros_like(mask)
    result[top:bottom, margin : image_width - margin] = mask[
        top:bottom, margin : image_width - margin
    ]
    return result


def adaptive_canny_thresholds(
    gray: np.ndarray, fallback_low: int, fallback_high: int, sigma: float
) -> Tuple[int, int]:
    """按灰度中位数自适应 Canny 阈值，并受配置上下界约束。"""
    median = float(np.median(gray))
    if median < 20.0:
        return fallback_low, fallback_high
    sigma = float(np.clip(sigma, 0.05, 0.90))
    low = int(np.clip((1.0 - sigma) * median, fallback_low * 0.5, fallback_high))
    high = int(
        np.clip(
            (1.0 + sigma) * median,
            max(low + 1, fallback_low),
            min(255, fallback_high * 1.5),
        )
    )
    return low, high


def box_fill_ratio(box: ContourBox) -> float:
    """Return contour rectangularity in its bounding box."""
    return box[4] / max(1.0, float(box[2] * box[3]))


def mask_density(mask: np.ndarray, box: ContourBox) -> float:
    """Measure supporting pixels inside a candidate bounding box."""
    x, y, width, height, _ = box
    region = mask[y : y + height, x : x + width]
    return float(cv2.countNonZero(region)) / max(1.0, float(width * height))


def best_pole_pair(
    boxes: Sequence[ContourBox],
    image_width: int,
    image_height: int,
    min_fill_ratio: float = 0.0,
) -> Tuple[List[ContourBox], float]:
    """选择最像成对立柱的两个框，并返回 0～1 几何一致性分数。

    分数综合垂直重叠、高宽相似性和轮廓填充率；这里只比较图像几何关系，
    不根据像素间距推断真实障碍宽度。
    """
    candidates = [
        box
        for box in boxes
        if box[3] >= box[2] * 1.5
        and box[3] >= image_height * 0.15
        and box_fill_ratio(box) >= min_fill_ratio
    ]
    best_pair: List[ContourBox] = []
    best_score = 0.0
    for first, second in combinations(candidates, 2):
        if first[0] > second[0]:
            first, second = second, first
        center_separation = (
            second[0] + second[2] / 2.0 - first[0] - first[2] / 2.0
        ) / image_width
        if not 0.10 <= center_separation <= 0.80:
            continue
        vertical_overlap = max(
            0,
            min(first[1] + first[3], second[1] + second[3])
            - max(first[1], second[1]),
        ) / max(1.0, float(min(first[3], second[3])))
        height_similarity = min(first[3], second[3]) / max(first[3], second[3])
        width_similarity = min(first[2], second[2]) / max(first[2], second[2])
        if vertical_overlap < 0.45 or height_similarity < 0.55:
            continue
        fill = min(box_fill_ratio(first), box_fill_ratio(second))
        score = (
            0.35 * vertical_overlap
            + 0.30 * height_similarity
            + 0.15 * width_similarity
            + 0.20 * min(1.0, fill / max(0.25, min_fill_ratio))
        )
        if score > best_score:
            best_pair = [first, second]
            best_score = score
    return best_pair, best_score


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


def image_quality_score(gray: np.ndarray) -> float:
    """估计曝光、对比度和清晰度是否足以支持传统视觉检测。

    返回值只是 0～1 的质量门控量，不是障碍概率。严重欠曝、过曝或失焦时，边缘
    启发式很容易把噪声识别成横杆/墙面，因此应降低该帧权重而不是强行分类。
    """
    image = np.asarray(gray, dtype=np.uint8)
    if image.size < 16:
        return 0.0
    usable_exposure = float(np.mean((image > 8) & (image < 247)))
    low, high = np.quantile(image, (0.10, 0.90))
    contrast = float(np.clip((float(high) - float(low)) / 45.0, 0.0, 1.0))
    laplacian_variance = float(cv2.Laplacian(image, cv2.CV_32F).var())
    sharpness = float(np.clip(laplacian_variance / 80.0, 0.0, 1.0))
    return float(
        np.clip(
            usable_exposure * (0.40 + 0.30 * contrast + 0.30 * sharpness),
            0.0,
            1.0,
        )
    )


def apply_image_quality(
    evidence: ObstacleEvidence, quality: float, minimum_quality: float
) -> ObstacleEvidence:
    """拒绝不可用图像，并对勉强可用图像的启发式置信度降权。"""
    quality = float(np.clip(quality, 0.0, 1.0))
    if evidence.hint == "none" or quality < minimum_quality:
        return ObstacleEvidence()
    return ObstacleEvidence(
        hint=evidence.hint,
        confidence=evidence.confidence * (0.60 + 0.40 * quality),
        center_x=evidence.center_x,
        center_y=evidence.center_y,
        width=evidence.width,
        height=evidence.height,
    )


def evidence_iou(first: ObstacleEvidence, second: ObstacleEvidence) -> float:
    """计算两个归一化目标框的交并比，约束多帧证据属于同一目标。"""
    first_left = first.center_x - first.width / 2
    first_top = first.center_y - first.height / 2
    second_left = second.center_x - second.width / 2
    second_top = second.center_y - second.height / 2
    first_right, first_bottom = first_left + first.width, first_top + first.height
    second_right, second_bottom = second_left + second.width, second_top + second.height
    intersection_width = max(
        0.0, min(first_right, second_right) - max(first_left, second_left)
    )
    intersection_height = max(
        0.0, min(first_bottom, second_bottom) - max(first_top, second_top)
    )
    intersection = intersection_width * intersection_height
    union = first.width * first.height + second.width * second.height - intersection
    return (
        0.0
        if union <= 1e-9
        else float(np.clip(intersection / union, 0.0, 1.0))
    )


def detect_obstacle_evidence(
    orange_mask: np.ndarray,
    blue_mask: np.ndarray,
    edge_mask: np.ndarray,
    min_area: float,
    min_color_fill_ratio: float = 0.18,
) -> ObstacleEvidence:
    """融合颜色与轮廓几何，产生一帧尚未时序确认的候选证据。

    优先级依次为带颜色横杆、带颜色立柱、纯边缘立柱/横杆/墙、一般色块。
    越靠后的分支歧义越大，因此置信度上限更低；同一帧只返回一个完整候选，避免
    多话题字段在不同时间被规划层拼接。
    """
    image_height, image_width = orange_mask.shape[:2]
    image_area = float(image_height * image_width)
    orange_boxes = contour_boxes(orange_mask, min_area)
    blue_boxes = contour_boxes(blue_mask, min_area)

    # 优先使用颜色特征：比赛标志色比普通场景边缘更稳定。
    horizontal_blue = [
        box
        for box in blue_boxes
        if box[2] >= box[3] * 2.2
        and box[4] / image_area >= 0.005
        and box_fill_ratio(box) >= min_color_fill_ratio
        and box[1] + box[3] / 2.0 <= image_height * 0.70
    ]
    if horizontal_blue:
        box = max(horizontal_blue, key=lambda item: item[4])
        geometry = min(1.0, box[2] / max(1.0, box[3] * 5.0))
        edge_support = mask_density(edge_mask, box)
        confidence = min(
            0.98,
            0.66
            + 0.12 * geometry
            + 0.10 * min(1.0, edge_support * 5.0)
            + box[4] / image_area * 2.0,
        )
        return evidence_from_boxes(
            "height_bar", confidence, [box], image_width, image_height
        )

    pole_pair, pair_score = best_pole_pair(
        orange_boxes,
        image_width,
        image_height,
        min_color_fill_ratio,
    )
    if pole_pair:
        edge_support = np.mean([mask_density(edge_mask, box) for box in pole_pair])
        confidence = min(
            0.98,
            0.66 + 0.22 * pair_score + 0.10 * min(1.0, edge_support * 5.0),
        )
        return evidence_from_boxes(
            "poles", confidence, pole_pair, image_width, image_height
        )

    # 当颜色受光照影响时，再使用 Canny 轮廓作为保守的几何补充。
    edge_boxes = contour_boxes(edge_mask, min_area)
    edge_pair, edge_pair_score = best_pole_pair(
        edge_boxes, image_width, image_height
    )
    if edge_pair:
        confidence = min(0.72, 0.54 + 0.14 * edge_pair_score)
        return evidence_from_boxes(
            "poles", confidence, edge_pair, image_width, image_height
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
        if (
            box[4] / image_area >= 0.03
            and box_fill_ratio(box) >= min_color_fill_ratio
        ):
            return evidence_from_boxes(
                "colored_obstacle", 0.55, [box], image_width, image_height
            )
    return ObstacleEvidence()


def stabilize_evidence(
    history: Sequence[ObstacleEvidence],
    minimum_matches: int,
    max_center_jitter: float = 0.15,
    max_size_jitter: float = 0.25,
    minimum_match_ratio: float = 0.60,
    minimum_iou: float = 0.20,
) -> ObstacleEvidence:
    """仅接受在近期多帧重复且位置/尺寸变化合理的非空候选。

    输出采用各字段中位数，降低单帧抖动影响。窗口内多数投票解决类别闪烁，中心与
    尺寸极差约束则拒绝反光、运动模糊或多个不同目标碰巧同类的情况。
    """
    # 多帧投票抑制反光、运动模糊和单帧噪声。
    hints = [item.hint for item in history if item.hint != "none"]
    if not hints:
        return ObstacleEvidence()
    hint, count = Counter(hints).most_common(1)[0]
    required = max(
        int(minimum_matches),
        int(np.ceil(len(history) * np.clip(minimum_match_ratio, 0.0, 1.0))),
    )
    if count < required:
        return ObstacleEvidence()
    matches = [item for item in history if item.hint == hint]
    center_x = np.asarray([item.center_x for item in matches])
    center_y = np.asarray([item.center_y for item in matches])
    widths = np.asarray([item.width for item in matches])
    heights = np.asarray([item.height for item in matches])
    center_jitter = max(float(np.ptp(center_x)), float(np.ptp(center_y)))
    size_jitter = max(float(np.ptp(widths)), float(np.ptp(heights)))
    if center_jitter > max_center_jitter or size_jitter > max_size_jitter:
        return ObstacleEvidence()
    median_box = ObstacleEvidence(
        hint=hint,
        center_x=float(np.median(center_x)),
        center_y=float(np.median(center_y)),
        width=float(np.median(widths)),
        height=float(np.median(heights)),
    )
    overlaps = [evidence_iou(item, median_box) for item in matches]
    minimum_overlap = float(min(overlaps))
    if minimum_overlap < float(np.clip(minimum_iou, 0.0, 1.0)):
        return ObstacleEvidence()
    confidence = float(np.median([item.confidence for item in matches]))
    consistency = count / max(1, len(history))
    spatial_consistency = 1.0 - max(
        center_jitter / max(1e-6, max_center_jitter),
        size_jitter / max(1e-6, max_size_jitter),
    )
    return ObstacleEvidence(
        hint=hint,
        # Repetition is already enforced by minimum_matches. Keep a small
        # consistency penalty without suppressing a valid 3-of-5 result.
        confidence=confidence
        * (0.8 + 0.2 * consistency)
        * (0.85 + 0.10 * spatial_consistency + 0.05 * minimum_overlap),
        center_x=median_box.center_x,
        center_y=median_box.center_y,
        width=median_box.width,
        height=median_box.height,
    )


class VisionObstacleDetector(Node):
    """从常见相机话题选择单一数据源，并发布经过时序确认的辅助证据。"""

    def __init__(self):
        super().__init__("vision_obstacle_detector")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("image_topic_candidates", DEFAULT_IMAGE_TOPICS)
        self.declare_parameter("debug_mask_topic", "/vision/debug_mask")
        self.declare_parameter("publish_debug_mask", False)
        self.declare_parameter("processing_hz", 8.0)
        self.declare_parameter("resize_width", 640)
        self.declare_parameter("min_area_px", 300.0)
        self.declare_parameter("min_area_ratio", 0.0008)
        self.declare_parameter("morphology_size", 5)
        self.declare_parameter("edge_low_threshold", 60)
        self.declare_parameter("edge_high_threshold", 160)
        self.declare_parameter("adaptive_canny", True)
        self.declare_parameter("adaptive_canny_sigma", 0.33)
        self.declare_parameter("illumination_normalization", True)
        self.declare_parameter("clahe_clip_limit", 2.0)
        self.declare_parameter("clahe_grid_size", 8)
        self.declare_parameter("roi_top_ratio", 0.05)
        self.declare_parameter("roi_bottom_ratio", 0.95)
        self.declare_parameter("roi_side_margin_ratio", 0.02)
        self.declare_parameter("min_color_fill_ratio", 0.18)
        self.declare_parameter("min_image_quality", 0.35)
        self.declare_parameter("history_size", 5)
        self.declare_parameter("confirmation_frames", 3)
        self.declare_parameter("max_temporal_center_jitter", 0.15)
        self.declare_parameter("max_temporal_size_jitter", 0.25)
        self.declare_parameter("temporal_match_ratio", 0.60)
        self.declare_parameter("min_temporal_iou", 0.20)
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
        self.min_area_ratio = float(
            np.clip(self.get_parameter("min_area_ratio").value, 0.0, 0.10)
        )
        self.edge_low = max(0, int(self.get_parameter("edge_low_threshold").value))
        self.edge_high = max(
            self.edge_low + 1,
            int(self.get_parameter("edge_high_threshold").value),
        )
        self.adaptive_canny = bool(self.get_parameter("adaptive_canny").value)
        self.adaptive_canny_sigma = float(
            np.clip(self.get_parameter("adaptive_canny_sigma").value, 0.05, 0.90)
        )
        self.illumination_normalization = bool(
            self.get_parameter("illumination_normalization").value
        )
        self.clahe_clip_limit = max(
            0.1, float(self.get_parameter("clahe_clip_limit").value)
        )
        self.clahe_grid_size = max(
            2, int(self.get_parameter("clahe_grid_size").value)
        )
        self.roi_top_ratio = float(
            np.clip(self.get_parameter("roi_top_ratio").value, 0.0, 0.95)
        )
        self.roi_bottom_ratio = float(
            np.clip(self.get_parameter("roi_bottom_ratio").value, 0.05, 1.0)
        )
        self.roi_side_margin_ratio = float(
            np.clip(self.get_parameter("roi_side_margin_ratio").value, 0.0, 0.45)
        )
        self.min_color_fill_ratio = float(
            np.clip(self.get_parameter("min_color_fill_ratio").value, 0.0, 1.0)
        )
        self.min_image_quality = float(
            np.clip(self.get_parameter("min_image_quality").value, 0.0, 1.0)
        )
        history_size = max(1, int(self.get_parameter("history_size").value))
        self.confirmation_frames = min(
            history_size,
            max(1, int(self.get_parameter("confirmation_frames").value)),
        )
        self.max_temporal_center_jitter = max(
            0.01,
            float(self.get_parameter("max_temporal_center_jitter").value),
        )
        self.max_temporal_size_jitter = max(
            0.01,
            float(self.get_parameter("max_temporal_size_jitter").value),
        )
        self.temporal_match_ratio = float(
            np.clip(self.get_parameter("temporal_match_ratio").value, 0.0, 1.0)
        )
        self.min_temporal_iou = float(
            np.clip(self.get_parameter("min_temporal_iou").value, 0.0, 1.0)
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
        self.typed_evidence_pub = self.create_publisher(
            VisionObstacle, "/vision/obstacle_stamped", 10
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
        """读取三通道 HSV 参数并夹紧到 OpenCV uint8 表示范围。"""
        values = np.asarray(self.get_parameter(name).value, dtype=np.int32)
        if values.shape != (3,):
            self.get_logger().warning(f"{name} must contain H, S and V; using zeros")
            values = np.zeros(3, dtype=np.int32)
        # OpenCV 的 Hue 范围是 0～179，S/V 才是 0～255。
        values[0] = np.clip(values[0], 0, 179)
        values[1:] = np.clip(values[1:], 0, 255)
        return values.astype(np.uint8)

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
        """处理一帧新图像并发布颜色特征、稳定证据和可选调试掩膜。"""
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

        # CLAHE 只处理亮度通道，在阴影与局部强光下保留较稳定的 HSV 色相。
        processed_bgr = (
            enhance_illumination(bgr, self.clahe_clip_limit, self.clahe_grid_size)
            if self.illumination_normalization
            else bgr
        )
        # HSV 负责颜色，灰度 Canny 负责轮廓；两者不做神经网络推理。
        hsv = cv2.cvtColor(processed_bgr, cv2.COLOR_BGR2HSV)
        orange_mask = self._clean_mask(
            cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        )
        blue_mask = self._clean_mask(
            cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        )
        gray = cv2.cvtColor(processed_bgr, cv2.COLOR_BGR2GRAY)
        quality = image_quality_score(gray)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edge_low, edge_high = (
            adaptive_canny_thresholds(
                gray, self.edge_low, self.edge_high, self.adaptive_canny_sigma
            )
            if self.adaptive_canny
            else (self.edge_low, self.edge_high)
        )
        edge_mask = cv2.Canny(gray, edge_low, edge_high)
        edge_mask = cv2.morphologyEx(edge_mask, cv2.MORPH_CLOSE, self.kernel)
        edge_mask = cv2.dilate(edge_mask, self.kernel, iterations=1)
        # 去掉天花板、机身边缘和镜头黑边等通常不属于前向通道的区域。
        orange_mask = apply_detection_roi(
            orange_mask,
            self.roi_top_ratio,
            self.roi_bottom_ratio,
            self.roi_side_margin_ratio,
        )
        blue_mask = apply_detection_roi(
            blue_mask,
            self.roi_top_ratio,
            self.roi_bottom_ratio,
            self.roi_side_margin_ratio,
        )
        edge_mask = apply_detection_roi(
            edge_mask,
            self.roi_top_ratio,
            self.roi_bottom_ratio,
            self.roi_side_margin_ratio,
        )

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
        effective_min_area = max(self.min_area, image_area * self.min_area_ratio)
        raw_evidence = detect_obstacle_evidence(
            orange_mask,
            blue_mask,
            edge_mask,
            effective_min_area,
            self.min_color_fill_ratio,
        )
        raw_evidence = apply_image_quality(
            raw_evidence, quality, self.min_image_quality
        )
        # 只把稳定后的原子证据交给规划层，避免读取到不同帧的混合字段。
        self.evidence_history.append(raw_evidence)
        evidence = stabilize_evidence(
            self.evidence_history,
            self.confirmation_frames,
            self.max_temporal_center_jitter,
            self.max_temporal_size_jitter,
            self.temporal_match_ratio,
            self.min_temporal_iou,
        )
        self.evidence_pub.publish(Float32MultiArray(data=evidence.as_array()))
        typed = VisionObstacle()
        typed.header = msg.header
        typed.obstacle_type = int(round(HINT_CODES.get(evidence.hint, 0.0)))
        typed.confidence = float(evidence.confidence)
        typed.center_x = float(evidence.center_x)
        typed.center_y = float(evidence.center_y)
        typed.width = float(evidence.width)
        typed.height = float(evidence.height)
        self.typed_evidence_pub.publish(typed)
        self.hint_pub.publish(String(data=evidence.hint))

        if self.publish_debug:
            debug_mask = cv2.merge((blue_mask, edge_mask, orange_mask))
            debug_msg = self.bridge.cv2_to_imgmsg(debug_mask, encoding="bgr8")
            debug_msg.header = msg.header
            self.mask_pub.publish(debug_msg)

    def _clean_mask(self, mask: np.ndarray) -> np.ndarray:
        """先开运算去孤立噪点，再闭运算填补同一色块的小孔洞。"""
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

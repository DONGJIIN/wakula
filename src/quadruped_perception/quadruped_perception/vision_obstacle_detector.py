"""轻量 OpenCV 障碍证据节点。

职责
----
从相机 Image 提取颜色和轮廓证据，输出归一化检测框与启发式置信度。处理顺序固定为：
最新帧限频、光照归一化、HSV/Canny、ROI 与形态学、结构分类、多帧空间确认。该节点不
恢复米制深度；输出只能辅助减速和类别复核，不能单独批准 STEP、CLIMB 或真实越障。

真机标定入口
------------
所有可调值只在 ``config/vision.yaml``。换相机、镜头、安装角或场馆后按以下顺序处理：
1. 固定曝光/白平衡，确认 CameraInfo、时间戳和图像方向；
2. 用 ``scripts/record_bag.sh`` 采集各障碍、空场、不同距离/角度及明暗/模糊负样本；
3. 按根目录 ``instruction.txt`` 第五节生成标签和报告；
4. 回放原始图像，依次调整 ROI、HSV、轮廓/蓝段间距、质量门和多帧确认，每次只改一组；
5. 用未参与调参的 bag 比较逐类 precision、recall、F1，再决定是否采用。

安全与移植边界
--------------
重复、乱序或过期帧不刷新心跳，也不贡献多帧投票。代码只定义算法结构，不保存某款相机
的第二套阈值；这样真实 launch、离线回放与移植后的机器人始终读取同一个 YAML 参数源。
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
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from quadruped_interfaces.msg import NavigationSafety, VisionObstacle

from quadruped_perception.parameter_validation import (
    VISION_PARAMETER_NAMES,
    validate_vision_parameters,
)
from quadruped_perception.sensor_contracts import (
    image_message_contract_valid,
    source_stamp_strictly_advances,
    source_stamp_is_plausible,
)

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
HINT_DISPLAY_NAMES = {
    "none": "NONE",
    "poles": "POLES",
    "height_bar": "HEIGHT BAR",
    "wall": "WALL",
    "colored_obstacle": "COLORED OBSTACLE",
}
GEOMETRY_DISPLAY_NAMES = {
    NavigationSafety.OBSTACLE_UNKNOWN: "UNKNOWN",
    NavigationSafety.OBSTACLE_CLEAR: "CLEAR",
    NavigationSafety.OBSTACLE_STEP: "STEP",
    NavigationSafety.OBSTACLE_PIT: "PIT",
    NavigationSafety.OBSTACLE_WALL: "WALL",
    NavigationSafety.OBSTACLE_BAR: "HEIGHT BAR",
    NavigationSafety.OBSTACLE_POLE: "POLES",
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
        """返回紧凑且字段同时更新的 ROS 数组兼容表示。"""
        return [
            HINT_CODES.get(self.hint, 0.0),
            self.confidence,
            self.center_x,
            self.center_y,
            self.width,
            self.height,
        ]


def annotate_detection_frame(
    bgr: np.ndarray,
    raw_evidence: ObstacleEvidence,
    stable_evidence: ObstacleEvidence,
    quality: float,
    roi_top_ratio: float,
    roi_bottom_ratio: float,
    roi_side_margin_ratio: float,
    geometry_label: str = "",
) -> np.ndarray:
    """生成供 RViz 调试的标注图，不参与检测或规划决策。

    黄色框表示本帧候选，绿色框表示通过多帧一致性门的最终视觉证据；青色矩形是实际
    检测 ROI。把候选与确认结果同时显示，能够快速区分“没检测到”和“检测到但尚未
    连续确认”，避免为了画面好看而降低生产阈值。
    """
    annotated = np.asarray(bgr, dtype=np.uint8).copy()
    if annotated.ndim != 3 or annotated.shape[2] != 3 or annotated.size == 0:
        return annotated
    image_height, image_width = annotated.shape[:2]
    top = int(np.clip(roi_top_ratio, 0.0, 0.95) * image_height)
    bottom = int(np.clip(roi_bottom_ratio, 0.05, 1.0) * image_height)
    margin = int(np.clip(roi_side_margin_ratio, 0.0, 0.45) * image_width)
    if bottom > top and image_width - margin > margin:
        cv2.rectangle(
            annotated,
            (margin, top),
            (image_width - margin - 1, bottom - 1),
            (255, 255, 0),
            1,
        )

    def draw_evidence(evidence: ObstacleEvidence, color, prefix: str, row: int) -> None:
        """把归一化候选框裁剪到当前图像，并绘制人类可读类别/置信度。"""
        if evidence.hint == "none" or evidence.width <= 0.0 or evidence.height <= 0.0:
            return
        left = int((evidence.center_x - evidence.width / 2.0) * image_width)
        right = int((evidence.center_x + evidence.width / 2.0) * image_width)
        top_px = int((evidence.center_y - evidence.height / 2.0) * image_height)
        bottom_px = int((evidence.center_y + evidence.height / 2.0) * image_height)
        left, right = sorted(
            (
                int(np.clip(left, 0, image_width - 1)),
                int(np.clip(right, 0, image_width - 1)),
            )
        )
        top_px, bottom_px = sorted(
            (
                int(np.clip(top_px, 0, image_height - 1)),
                int(np.clip(bottom_px, 0, image_height - 1)),
            )
        )
        cv2.rectangle(annotated, (left, top_px), (right, bottom_px), color, 2)
        label = (
            f"{prefix}: {HINT_DISPLAY_NAMES.get(evidence.hint, 'UNKNOWN')} "
            f"{evidence.confidence:.2f}"
        )
        cv2.putText(
            annotated,
            label,
            (8, row),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    draw_evidence(raw_evidence, (0, 190, 255), "CANDIDATE", 44)
    draw_evidence(stable_evidence, (0, 255, 0), "CONFIRMED", 68)
    stable_name = HINT_DISPLAY_NAMES.get(stable_evidence.hint, "UNKNOWN")
    front_name = geometry_label or stable_name
    cv2.putText(
        annotated,
        f"FRONT: {front_name} | VISION: {stable_name} | QUALITY: {quality:.2f}",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0) if stable_evidence.hint != "none" else (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return annotated


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
    """计算轮廓面积占外接矩形的比例，用于剔除极细碎边缘。"""
    return box[4] / max(1.0, float(box[2] * box[3]))


def mask_density(mask: np.ndarray, box: ContourBox) -> float:
    """计算候选框内有效掩膜像素密度，过滤大框内的稀疏反光。"""
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


def best_segmented_horizontal_bar(
    boxes: Sequence[ContourBox],
    image_width: int,
    image_height: int,
    min_fill_ratio: float = 0.0,
    maximum_gap_ratio: float = 2.5,
    maximum_gap_cv: float = 0.55,
) -> Tuple[List[ContourBox], float]:
    """合并同一水平线上的蓝色短段，识别规则中的蓝白相间限高杆。

    比赛横杆由蓝白短段交替组成，HSV 蓝色掩膜天然会得到多个互不连通的小框。旧算法
    只接受单个宽横框，因此即使肉眼清楚看到横杆也会输出 ``VISION: NONE``。这里不做
    任意形态学长距离闭合（那会粘连台阶边缘），而是要求至少三个蓝色短段同时满足：
    高度中心对齐、单段较薄、合并后横向跨度有限且蓝色覆盖率足够。

    返回的分数只表示二维几何一致性；真实净空仍必须由点云确认。
    """
    candidates = [
        box
        for box in boxes
        if box[2] >= 3
        and box[3] >= 2
        and box[3] <= image_height * 0.12
        and box[2] <= image_width * 0.28
        # 相机安装高度高于 0.32 m 横杆时，接近过程中横杆会落到画面下部；至少
        # 三段对齐的强结构证据足以把下界放到 ROI 底部附近。
        and box[1] + box[3] / 2.0 <= image_height * 0.93
        and box_fill_ratio(box) >= min_fill_ratio
    ]
    best_group: List[ContourBox] = []
    best_score = 0.0
    alignment_limit = max(4.0, image_height * 0.035)
    for anchor in candidates:
        anchor_center_y = anchor[1] + anchor[3] / 2.0
        group = [
            box
            for box in candidates
            if abs((box[1] + box[3] / 2.0) - anchor_center_y)
            <= alignment_limit
        ]
        if len(group) < 3:
            continue
        group = sorted(group, key=lambda box: box[0])
        segment_widths = np.asarray([box[2] for box in group], dtype=np.float64)
        segment_heights = np.asarray([box[3] for box in group], dtype=np.float64)
        gaps = np.asarray(
            [
                next_box[0] - (box[0] + box[2])
                for box, next_box in zip(group, group[1:])
            ],
            dtype=np.float64,
        )
        # A rule crossbar produces repeated blue-white segments, not merely three
        # blue objects at a similar image row. Require positive, roughly regular gaps
        # and comparable segment sizes. Perspective is allowed by deliberately loose
        # ratios; one distant sign or team shirt separated by a large empty interval
        # is rejected before temporal confirmation can make it look stable.
        median_width = max(1.0, float(np.median(segment_widths)))
        mean_gap = float(np.mean(gaps)) if gaps.size else 0.0
        gap_cv = (
            float(np.std(gaps)) / max(1.0, mean_gap)
            if gaps.size
            else float("inf")
        )
        if (
            np.any(gaps <= 0.0)
            or float(np.max(gaps)) / median_width > max(0.1, float(maximum_gap_ratio))
            or gap_cv > max(0.0, float(maximum_gap_cv))
            or float(np.min(segment_widths)) / max(1.0, float(np.max(segment_widths)))
            < 0.35
            or float(np.min(segment_heights)) / max(1.0, float(np.max(segment_heights)))
            < 0.35
        ):
            continue
        left = min(box[0] for box in group)
        right = max(box[0] + box[2] for box in group)
        top = min(box[1] for box in group)
        bottom = max(box[1] + box[3] for box in group)
        span_width = right - left
        span_height = bottom - top
        width_ratio = span_width / max(1.0, float(image_width))
        height_ratio = span_height / max(1.0, float(image_height))
        horizontal_coverage = sum(box[2] for box in group) / max(
            1.0, float(span_width)
        )
        if not 0.18 <= width_ratio <= 0.85:
            continue
        if height_ratio > 0.16 or span_width < span_height * 3.0:
            continue
        if horizontal_coverage < 0.22:
            continue
        centers = np.asarray([box[1] + box[3] / 2.0 for box in group])
        alignment = 1.0 - min(
            1.0, float(np.ptp(centers)) / max(1.0, 2.0 * alignment_limit)
        )
        score = min(
            1.0,
            0.34
            + 0.08 * min(5, len(group))
            + 0.18 * alignment
            + 0.18 * min(1.0, horizontal_coverage / 0.50),
        )
        if score > best_score:
            best_group, best_score = group, score
    return best_group, best_score


def largest_color_feature(mask: np.ndarray, min_area: float) -> ColorFeature:
    """返回最大有效色块的面积与归一化外接框。"""
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
    """合并一个或多个相关区域，生成覆盖它们的归一化障碍证据。"""
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
    median = float(np.median(image))
    # 中位亮度接近任一裁剪端时，即使 CLAHE 后看起来有纹理，原始信噪比仍然不可靠。
    exposure_balance = float(
        np.clip(min(median / 32.0, (255.0 - median) / 32.0), 0.0, 1.0)
    )
    laplacian_variance = float(cv2.Laplacian(image, cv2.CV_32F).var())
    sharpness = float(np.clip(laplacian_variance / 80.0, 0.0, 1.0))
    return float(
        np.clip(
            usable_exposure
            * (
                0.15
                + 0.25 * exposure_balance
                + 0.30 * contrast
                + 0.30 * sharpness
            ),
            0.0,
            1.0,
        )
    )


def combined_hsv_mask(
    original_bgr: np.ndarray,
    enhanced_bgr: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> np.ndarray:
    """合并原图与光照增强图的 HSV 分割结果。

    原图在正常曝光时保留最可信的色相；增强图补回阴影中的低亮度颜色。二者取并集后仍
    要经过面积、填充率、多帧和点云复核，因此不会单凭扩大的颜色区域批准越障。
    """
    original_hsv = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2HSV)
    original_mask = hsv_range_mask(original_hsv, lower, upper)
    if enhanced_bgr is original_bgr:
        return original_mask
    enhanced_hsv = cv2.cvtColor(enhanced_bgr, cv2.COLOR_BGR2HSV)
    return cv2.bitwise_or(original_mask, hsv_range_mask(enhanced_hsv, lower, upper))


def hsv_range_mask(
    hsv: np.ndarray, lower: np.ndarray, upper: np.ndarray
) -> np.ndarray:
    """生成支持 Hue 0/179 环绕的 HSV 掩膜。

    OpenCV 把 Hue 压缩到 0～179。红橙色在真实相机白平衡变化时可能跨过边界，例如
    标定范围希望表达 ``H=175..179 或 0..12``。普通 ``cv2.inRange`` 要求下界不大于
    上界，会让这种配置静默输出全黑。这里约定：当 lower.H > upper.H 时仅 Hue 分成
    两段，S/V 仍共用同一上下界。正常范围保持一次 inRange 的低开销路径。
    """
    lower = np.asarray(lower, dtype=np.uint8).reshape(3)
    upper = np.asarray(upper, dtype=np.uint8).reshape(3)
    if int(lower[0]) <= int(upper[0]):
        return cv2.inRange(hsv, lower, upper)
    high_segment = cv2.inRange(
        hsv,
        lower,
        np.asarray((179, upper[1], upper[2]), dtype=np.uint8),
    )
    low_segment = cv2.inRange(
        hsv,
        np.asarray((0, lower[1], lower[2]), dtype=np.uint8),
        upper,
    )
    return cv2.bitwise_or(high_segment, low_segment)


def suppress_specular_edges(
    edge_mask: np.ndarray,
    bgr: np.ndarray,
    saturation_max: int = 25,
    value_min: int = 245,
    dilation_size: int = 7,
) -> np.ndarray:
    """移除贴近大面积白色高光的边缘，降低灯光反射造成的假横杆。

    只屏蔽低饱和且接近传感器上限的像素，不删除正常白色区域之外的结构边缘；蓝白限高
    杆仍由蓝色色块和剩余轮廓支持。膨胀范围有上限，避免整幅强光画面产生巨大掩膜。
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    glare = cv2.inRange(
        hsv,
        np.asarray((0, 0, int(np.clip(value_min, 0, 255))), dtype=np.uint8),
        np.asarray(
            (179, int(np.clip(saturation_max, 0, 255)), 255), dtype=np.uint8
        ),
    )
    kernel_size = max(1, min(31, int(dilation_size)))
    kernel_size += 1 if kernel_size % 2 == 0 else 0
    if kernel_size > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        glare = cv2.dilate(glare, kernel, iterations=1)
    return cv2.bitwise_and(edge_mask, cv2.bitwise_not(glare))


def apply_image_quality(
    evidence: ObstacleEvidence, quality: float, minimum_quality: float
) -> ObstacleEvidence:
    """拒绝不可用图像，并对勉强可用图像的启发式置信度降权。"""
    quality = float(np.clip(quality, 0.0, 1.0))
    if evidence.hint == "none" or quality < minimum_quality:
        return ObstacleEvidence()
    return ObstacleEvidence(
        hint=evidence.hint,
        # 质量刚过门槛时要明显降低单帧权重；旧 0.60+0.40*q 对严重模糊帧惩罚过轻，
        # 颜色轮廓仍完整时几乎保持原置信度。0.40+0.60*q 保留正常画面召回率，同时让
        # 阴影、轻微模糊可参与但更难凭少数帧形成稳定高置信结果。
        confidence=evidence.confidence * (0.40 + 0.60 * quality),
        center_x=evidence.center_x,
        center_y=evidence.center_y,
        width=evidence.width,
        height=evidence.height,
    )


def temporal_history_requires_reset(
    previous_receive_seconds: float,
    current_receive_seconds: float,
    maximum_gap: float,
) -> bool:
    """判断两帧接收间隔是否已破坏多帧投票的连续性。

    时序确认的前提是若干帧来自同一段连续视频。相机掉线后若保留旧票数，恢复时只需
    一帧相似画面就可能立即确认障碍；ROS 时钟回拨（rosbag 重播/重置）也必须清空历史。
    """
    values = (previous_receive_seconds, current_receive_seconds, maximum_gap)
    if not all(np.isfinite(float(value)) for value in values):
        return True
    gap = float(current_receive_seconds) - float(previous_receive_seconds)
    return maximum_gap <= 0.0 or gap < 0.0 or gap > maximum_gap


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
    min_bar_aspect_ratio: float = 3.0,
    max_bar_width_ratio: float = 0.85,
    max_bar_height_ratio: float = 0.22,
    segmented_bar_max_gap_ratio: float = 2.5,
    segmented_bar_max_gap_cv: float = 0.55,
) -> ObstacleEvidence:
    """融合颜色与轮廓几何，产生一帧尚未时序确认的候选证据。

    优先级依次为带颜色横杆、带颜色立柱和纯边缘成对立柱。
    越靠后的分支歧义越大，因此置信度上限更低；同一帧只返回一个完整候选，避免
    多话题字段在不同时间被规划层拼接。
    """
    image_height, image_width = orange_mask.shape[:2]
    image_area = float(image_height * image_width)
    # 横杆应是一条有限宽度的细长结构。开放场地中的地平线、坡面边缘或整片地板颜色
    # 经透视后也可能形成稳定横向轮廓，但它们往往横跨几乎整幅图像，且外接框很高。
    # 这些比例保持为可标定参数，换相机视场后只改 YAML，不改检测逻辑。
    bar_aspect = max(1.0, float(min_bar_aspect_ratio))
    bar_width_limit = float(np.clip(max_bar_width_ratio, 0.10, 1.0))
    bar_height_limit = float(np.clip(max_bar_height_ratio, 0.02, 1.0))
    orange_boxes = contour_boxes(orange_mask, min_area)
    blue_boxes = contour_boxes(blue_mask, min_area)

    # 蓝白相间横杆的单个蓝段可能小于通用轮廓面积门槛。仅在“至少三段水平对齐”的
    # 强结构约束下使用更小面积门，既恢复远距离横杆召回率，又不会放宽其他类别。
    segmented_blue_boxes = contour_boxes(
        blue_mask,
        max(20.0, float(min_area) * 0.12),
    )
    segmented_bar, segmented_score = best_segmented_horizontal_bar(
        segmented_blue_boxes,
        image_width,
        image_height,
        min_color_fill_ratio,
        segmented_bar_max_gap_ratio,
        segmented_bar_max_gap_cv,
    )
    if segmented_bar:
        confidence = min(0.96, 0.62 + 0.30 * segmented_score)
        return evidence_from_boxes(
            "height_bar",
            confidence,
            segmented_bar,
            image_width,
            image_height,
        )

    # 优先使用颜色特征：比赛标志色比普通场景边缘更稳定。
    horizontal_blue = [
        box
        for box in blue_boxes
        if box[2] >= box[3] * bar_aspect
        and box[2] <= image_width * bar_width_limit
        and box[3] <= image_height * bar_height_limit
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

    # 接近直角绕杆区时，另一根杆可能位于视野外或被近处杆遮挡。只接受位于前向通道、
    # 高宽比很大且颜色填充充分的单根细长区域；这比随后歧义较大的“无色墙轮廓”更能
    # 解释比赛立柱，也消除 RViz 中点云已判 POLE、视觉却长期显示 WALL 的矛盾。
    single_colored_poles = [
        box
        for box in orange_boxes
        if box[3] >= box[2] * 3.0
        and box[3] >= image_height * 0.20
        and box[2] <= image_width * 0.12
        and image_width * 0.20
        <= box[0] + box[2] / 2.0
        <= image_width * 0.80
        and image_height * 0.18
        <= box[1] + box[3] / 2.0
        <= image_height * 0.78
        and box_fill_ratio(box) >= min_color_fill_ratio
    ]
    if single_colored_poles:
        box = max(single_colored_poles, key=lambda item: item[3])
        slenderness = min(1.0, box[3] / max(1.0, box[2] * 6.0))
        edge_support = mask_density(edge_mask, box)
        confidence = min(
            0.86,
            0.64
            + 0.12 * slenderness
            + 0.08 * min(1.0, edge_support * 5.0),
        )
        return evidence_from_boxes(
            "poles", confidence, [box], image_width, image_height
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

    # 单条无颜色横边不能区分真正横杆、台阶顶边、坡面边缘和地平线。联调中这类轮廓
    # 即使跨多帧完全稳定，也会把远处成排障碍误报成限高横杆。Canny 仍用于给蓝色横杆
    # 提供 edge_support，并可辅助墙/立柱候选；但 HEIGHT_BAR 类别必须拥有蓝色色块证据，
    # 近距离无色横杆则由深度点云的离地净空分类兜底。

    # 单个闭合边缘框无法区分墙、台阶正面、坡道边界、部分出画的立柱组和场地边缘。
    # 旧版用固定 0.54 输出 WALL，虽低于规划介入阈值，却会在 RViz 长期显示错误名称。
    # 这里不再给纯边缘框赋语义；高墙由点云的米制高度/垂直跨度可靠确认。

    # 单个橙/蓝色块无法区分台阶、桥板、坡面、坑区边框或黄色场地反射。旧逻辑把
    # 任意大色块稳定确认为 COLORED OBSTACLE，既没有动作语义，又会让 RViz 看起来像
    # 识别成功。只有横杆、双立柱等具备结构约束的颜色证据才对外发布；其他类别由
    # 点云几何给出权威名称。
    return ObstacleEvidence()


def stabilize_evidence(
    history: Sequence[ObstacleEvidence],
    minimum_matches: int,
    max_center_jitter: float = 0.15,
    max_size_jitter: float = 0.25,
    minimum_match_ratio: float = 0.60,
    minimum_iou: float = 0.20,
) -> ObstacleEvidence:
    """仅接受在近期多帧重复且相邻观测运动合理的当前候选。

    窗口内多数投票解决类别闪烁；空间门则比较相邻同类框，而不是比较整个
    窗口的极差。机器人持续接近或转向时，同一障碍的框会每帧平滑放大/平移；
    累计位移可以很大，但单次跳变仍必须被拒绝。

    返回框始终取当前（窗口最后）匹配帧，使节点赋予的当前 Image Header 与框
    来自同一时刻。历史中位框虽然更平滑，但在 5 Hz、5 帧窗口下约滞后 0.4 s，
    时间同步节点会误以为它是当前空间证据。
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
    # 历史多数不得为当前帧伪造框。本帧丢检或已切换类别时先输出 NONE，
    # 待新类别重新累积；否则会把旧障碍位置配上新图像的 Header。
    current = history[-1]
    if current.hint != hint:
        return ObstacleEvidence()
    indexed_matches = [
        (index, item)
        for index, item in enumerate(history)
        if item.hint == hint
    ]
    matches = [item for _, item in indexed_matches]
    center_steps = []
    size_steps = []
    overlaps = []
    for (previous_index, previous), (following_index, following) in zip(
        indexed_matches, indexed_matches[1:]
    ):
        # NONE 通常来自一帧拖影/曝光质量门。跨过这种缺口时，允许的位移与
        # 尺寸变化应按真实帧间隔线性放大，不能把相隔两帧的框当成相邻一帧。
        frame_gap = max(1, int(following_index) - int(previous_index))
        center_delta = max(
            abs(float(following.center_x) - float(previous.center_x)),
            abs(float(following.center_y) - float(previous.center_y)),
        )
        size_delta = max(
            abs(float(following.width) - float(previous.width)),
            abs(float(following.height) - float(previous.height)),
        )
        if (
            center_delta > max_center_jitter * frame_gap
            or size_delta > max_size_jitter * frame_gap
        ):
            return ObstacleEvidence()
        center_steps.append(center_delta / frame_gap)
        size_steps.append(size_delta / frame_gap)
        # IoU 是“相邻帧是同一实体”的强约束；跨过缺帧时框可能正常放大
        # 数倍，此时由上面的单帧平均运动门承担关联，不强制原始 IoU。
        if frame_gap == 1:
            overlaps.append(evidence_iou(previous, following))
    maximum_center_step = max(center_steps, default=0.0)
    maximum_size_step = max(size_steps, default=0.0)
    # 上面已用实际 frame_gap 拒绝超限跳变；这两个归一化值仅用于置信度降权。
    minimum_overlap = float(min(overlaps, default=1.0))
    if minimum_overlap < float(np.clip(minimum_iou, 0.0, 1.0)):
        return ObstacleEvidence()
    confidence = float(np.median([item.confidence for item in matches]))
    consistency = count / max(1, len(history))
    spatial_consistency = 1.0 - max(
        maximum_center_step / max(1e-6, max_center_jitter),
        maximum_size_step / max(1e-6, max_size_jitter),
    )
    return ObstacleEvidence(
        hint=hint,
        # minimum_matches 已保证重复出现；这里只施加小幅一致性惩罚，不能压掉有效的 3/5 投票。
        confidence=confidence
        * (0.8 + 0.2 * consistency)
        * (0.85 + 0.10 * spatial_consistency + 0.05 * minimum_overlap),
        # 时间同步的原子性比平滑显示更重要：框与外层即将填入的最新 Header 同帧。
        center_x=current.center_x,
        center_y=current.center_y,
        width=current.width,
        height=current.height,
    )


class VisionObstacleDetector(Node):
    """从常见相机话题选择单一数据源，并发布经过时序确认的辅助证据。"""

    def __init__(self, **node_kwargs):
        """加载可标定参数，创建候选相机订阅和视觉证据发布器。

        参数分为图像质量、颜色、轮廓和时序确认四组。换相机时应通过 YAML 调参，不能把
        场地颜色或分辨率常量写回检测函数，确保在线节点和离线评估使用同一算法。
        """
        # Forward standard Node keyword arguments (especially parameter_overrides) so launch
        # tests and future composed deployments exercise the exact same constructor as runtime.
        super().__init__("vision_obstacle_detector", **node_kwargs)
        self.declare_parameter("image_topic", "")
        self.declare_parameter("image_topic_candidates", DEFAULT_IMAGE_TOPICS)
        self.declare_parameter("debug_mask_topic", "/vision/debug_mask")
        self.declare_parameter("publish_debug_mask", False)
        self.declare_parameter("annotated_image_topic", "/vision/annotated_image")
        self.declare_parameter("publish_annotated_image", True)
        # 与 vision.yaml 的 RK3588 在线档一致；直接 ros2 run 时也不会绕过限频/缩放。
        self.declare_parameter("processing_hz", 5.0)
        self.declare_parameter("resize_width", 576)
        self.declare_parameter("min_area_px", 300.0)
        self.declare_parameter("min_area_ratio", 0.0008)
        self.declare_parameter("morphology_size", 5)
        self.declare_parameter("edge_low_threshold", 60)
        self.declare_parameter("edge_high_threshold", 160)
        self.declare_parameter("adaptive_canny", True)
        self.declare_parameter("adaptive_canny_sigma", 0.33)
        self.declare_parameter("illumination_normalization", True)
        self.declare_parameter("dual_illumination_color_mask", True)
        self.declare_parameter("clahe_clip_limit", 2.0)
        self.declare_parameter("clahe_grid_size", 8)
        self.declare_parameter("suppress_specular_glare", True)
        self.declare_parameter("glare_saturation_max", 25)
        self.declare_parameter("glare_value_min", 245)
        self.declare_parameter("glare_dilation_size", 7)
        self.declare_parameter("roi_top_ratio", 0.05)
        self.declare_parameter("roi_bottom_ratio", 0.95)
        self.declare_parameter("roi_side_margin_ratio", 0.02)
        self.declare_parameter("min_color_fill_ratio", 0.18)
        self.declare_parameter("min_bar_aspect_ratio", 3.0)
        self.declare_parameter("max_bar_width_ratio", 0.85)
        self.declare_parameter("max_bar_height_ratio", 0.22)
        self.declare_parameter("segmented_bar_max_gap_ratio", 2.5)
        self.declare_parameter("segmented_bar_max_gap_cv", 0.55)
        self.declare_parameter("min_image_quality", 0.35)
        self.declare_parameter("history_size", 5)
        self.declare_parameter("confirmation_frames", 3)
        self.declare_parameter("max_temporal_center_jitter", 0.15)
        self.declare_parameter("max_temporal_size_jitter", 0.25)
        self.declare_parameter("temporal_match_ratio", 0.60)
        self.declare_parameter("min_temporal_iou", 0.20)
        self.declare_parameter("history_reset_timeout", 0.75)
        self.declare_parameter("source_switch_timeout", 2.0)
        self.declare_parameter("source_failure_cooldown", 2.0)
        self.declare_parameter("orange_hsv_lower", [5, 80, 70])
        self.declare_parameter("orange_hsv_upper", [25, 255, 255])
        self.declare_parameter("blue_hsv_lower", [90, 70, 50])
        self.declare_parameter("blue_hsv_upper", [135, 255, 255])

        # Validate the raw YAML values before clipping or allocating ROS entities.  A typo such
        # as an inverted ROI or impossible HSV range must fail at launch instead of silently
        # changing the algorithm and masquerading as poor camera accuracy.
        validate_vision_parameters(
            {name: self.get_parameter(name).value for name in VISION_PARAMETER_NAMES}
        )

        override_topic = str(self.get_parameter("image_topic").value)
        candidates = list(self.get_parameter("image_topic_candidates").value)
        self.image_topics = (
            [override_topic] if override_topic else list(dict.fromkeys(candidates))
        )
        self.publish_debug = bool(self.get_parameter("publish_debug_mask").value)
        self.publish_annotated = bool(
            self.get_parameter("publish_annotated_image").value
        )
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
        self.dual_illumination_color_mask = bool(
            self.get_parameter("dual_illumination_color_mask").value
        )
        self.clahe_clip_limit = max(
            0.1, float(self.get_parameter("clahe_clip_limit").value)
        )
        self.clahe_grid_size = max(
            2, int(self.get_parameter("clahe_grid_size").value)
        )
        self.suppress_specular_glare = bool(
            self.get_parameter("suppress_specular_glare").value
        )
        self.glare_saturation_max = int(
            np.clip(self.get_parameter("glare_saturation_max").value, 0, 255)
        )
        self.glare_value_min = int(
            np.clip(self.get_parameter("glare_value_min").value, 0, 255)
        )
        self.glare_dilation_size = max(
            1, int(self.get_parameter("glare_dilation_size").value)
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
        self.min_bar_aspect_ratio = max(
            1.0, float(self.get_parameter("min_bar_aspect_ratio").value)
        )
        self.max_bar_width_ratio = float(
            np.clip(self.get_parameter("max_bar_width_ratio").value, 0.10, 1.0)
        )
        self.max_bar_height_ratio = float(
            np.clip(self.get_parameter("max_bar_height_ratio").value, 0.02, 1.0)
        )
        self.segmented_bar_max_gap_ratio = max(
            0.1,
            float(self.get_parameter("segmented_bar_max_gap_ratio").value),
        )
        self.segmented_bar_max_gap_cv = float(
            np.clip(
                self.get_parameter("segmented_bar_max_gap_cv").value,
                0.0,
                2.0,
            )
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
        self.history_reset_timeout = max(
            0.1, float(self.get_parameter("history_reset_timeout").value)
        )
        self.source_switch_timeout = max(
            0.1, float(self.get_parameter("source_switch_timeout").value)
        )
        self.source_failure_cooldown = max(
            0.1, float(self.get_parameter("source_failure_cooldown").value)
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
        # Separate from the processing watermark: callbacks run faster than the 5 Hz
        # detector. This prevents an older buffered image from replacing a newer
        # ``latest_frame`` before the timer gets to it.
        self.last_received_source_stamp = None
        self.active_topic = None
        self.last_active_image_time = None
        # Keep the ROS epoch independent of source health.  Otherwise a future watermark or
        # cooldown left by a rosbag seek can reject every first-frame candidate in the new epoch.
        self.last_ros_time_ns = None
        self.source_cooldown_until = {}
        self.geometry_label = "WAITING FOR DEPTH"
        self.last_geometry_time = None
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
        # 标注图仅用于 RViz/录包调试，不回灌检测算法。RViz 的 Image 显示默认请求
        # RELIABLE；若沿用相机输入常用的 BEST_EFFORT 传感器 QoS，双方会因可靠性策略
        # 不兼容而完全收不到图。小队列仍可避免慢速 GUI 在 RK3588 上造成图像积压。
        annotated_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.annotated_pub = (
            self.create_publisher(
                Image,
                str(self.get_parameter("annotated_image_topic").value),
                annotated_qos,
            )
            if self.publish_annotated
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
        self.create_subscription(
            NavigationSafety,
            "/terrain/navigation_safety",
            self.geometry_callback,
            10,
        )
        processing_hz = min(
            30.0, max(0.5, float(self.get_parameter("processing_hz").value))
        )
        self.create_timer(1.0 / processing_hz, self.processing_callback)
        self.get_logger().info(
            f"OpenCV obstacle evidence: {self.image_topics} at {processing_hz:.1f} Hz"
        )

    def geometry_callback(self, msg: NavigationSafety) -> None:
        """缓存点云融合后的权威类别，仅用于相机调试画面文字，不回灌视觉检测。"""
        self._observe_ros_epoch(self.get_clock().now())
        self.geometry_label = (
            GEOMETRY_DISPLAY_NAMES.get(int(msg.obstacle_type), "UNKNOWN")
            if msg.perception_valid
            else "INVALID DEPTH"
        )
        self.last_geometry_time = self.get_clock().now()

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
        """只保存最新图像，避免相机帧率高于处理能力时形成积压。"""
        now = self.get_clock().now()
        self._observe_ros_epoch(now)
        now_seconds = now.nanoseconds * 1e-9
        cooldown_until = self.source_cooldown_until.get(source, float("-inf"))
        if now_seconds < cooldown_until:
            return
        self.source_cooldown_until = {
            topic: deadline
            for topic, deadline in self.source_cooldown_until.items()
            if deadline > now_seconds
        }
        # A publisher is not a usable camera merely because DDS messages arrive.  Validate the
        # cheap structural contract before it can lock ``active_topic`` and suppress a healthy
        # fallback camera.  Encoding interpretation is also cheap and catches unsupported
        # vendor strings without decoding the full image in this high-frequency callback.
        try:
            dtype, channels = self.bridge.encoding_to_dtype_with_channels(msg.encoding)
            encoding_valid = int(channels) in (1, 3, 4)
            bytes_per_pixel = int(np.dtype(dtype).itemsize) * int(channels)
        except (CvBridgeError, KeyError, RuntimeError, TypeError, ValueError):
            encoding_valid = False
            bytes_per_pixel = 1
        if (
            not image_message_contract_valid(msg, bytes_per_pixel)
            or not encoding_valid
            or not source_stamp_is_plausible(
                msg.header, now_seconds, self.source_switch_timeout
            )
        ):
            self.get_logger().warning(
                f"Ignoring invalid Image contract from {source}",
                throttle_duration_sec=2.0,
            )
            return
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
        new_source_session = source != self.active_topic
        # 同时存在多个默认图像话题时只选一个，失联后再自动切换。
        if new_source_session:
            self.evidence_history.clear()
            self.last_processed_stamp = None
            self.last_received_source_stamp = None
            self.active_topic = source
            self.get_logger().info(f"Using camera topic {source}")
        if not source_stamp_strictly_advances(
            msg.header, self.last_received_source_stamp
        ):
            self.get_logger().warning(
                f"Ignoring duplicate/out-of-order Image from {source}",
                throttle_duration_sec=2.0,
            )
            return
        if (
            not new_source_session
            and self.last_active_image_time is not None
            and temporal_history_requires_reset(
                self.last_active_image_time.nanoseconds * 1e-9,
                now_seconds,
                self.history_reset_timeout,
            )
        ):
            # 同一相机恢复也不能沿用断流前的障碍票数；必须重新积累 confirmation_frames。
            self.evidence_history.clear()
            self.last_processed_stamp = None
            self.get_logger().warning("Camera stream gap; reset visual history")
        self.last_received_source_stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        self.last_active_image_time = now
        self.latest_frame = (msg, source)

    def _observe_ros_epoch(self, now) -> bool:
        """Clear all temporal/source state before processing data after a ROS-time rewind.

        The comparison uses integer nanoseconds to avoid floating-point loss on long recordings.
        It runs at every subscription/timer entry so neither an old pending image nor a future
        cooldown/watermark can cross a rosbag or Gazebo reset.
        """
        current_ns = int(now.nanoseconds)
        rewound = (
            self.last_ros_time_ns is not None
            and current_ns < self.last_ros_time_ns
        )
        if rewound:
            self._reset_sensor_epoch()
            self.get_logger().warning(
                "ROS clock moved backward; reset camera source epoch",
                throttle_duration_sec=2.0,
            )
        self.last_ros_time_ns = current_ns
        return rewound

    def _reset_sensor_epoch(self) -> None:
        """Forget observations and deadlines that belong to a previous ROS-time epoch."""
        self.latest_frame = None
        self.last_processed_stamp = None
        self.last_received_source_stamp = None
        self.active_topic = None
        self.last_active_image_time = None
        self.source_cooldown_until.clear()
        self.evidence_history.clear()
        # The annotation must not display a depth label received before the replay seek.
        self.geometry_label = "WAITING FOR DEPTH"
        self.last_geometry_time = None

    def _mark_source_unhealthy(self, source: str) -> None:
        """Cooldown an image source whose pixel payload cannot produce a usable BGR frame."""
        now = self.get_clock().now()
        if self._observe_ros_epoch(now):
            return
        now_seconds = now.nanoseconds * 1e-9
        self.source_cooldown_until[source] = (
            now_seconds + self.source_failure_cooldown
        )
        if self.latest_frame is not None and self.latest_frame[1] == source:
            self.latest_frame = None
        if source != self.active_topic:
            return
        self.get_logger().warning(
            f"Camera source {source} is unusable; allowing backup source takeover",
            throttle_duration_sec=2.0,
        )
        self.active_topic = None
        self.last_active_image_time = None
        self.last_processed_stamp = None
        self.last_received_source_stamp = None
        self.evidence_history.clear()

    def processing_callback(self) -> None:
        """处理一帧新图像并发布颜色特征、稳定证据和可选调试掩膜。"""
        # Discard an old pending frame if /clock rewinds between subscriber and timer callbacks.
        if self._observe_ros_epoch(self.get_clock().now()):
            return
        if self.latest_frame is None:
            return
        msg, source = self.latest_frame
        stamp = (source, msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except (CvBridgeError, RuntimeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            # Metadata-valid vendor encodings can still fail the real bgr8 conversion.  Quarantine
            # that source so its next high-rate frame cannot immediately reacquire before a healthy
            # backup is seen; a one-camera installation retries automatically after the cooldown.
            self._mark_source_unhealthy(source)
            return
        if bgr.size == 0:
            self.get_logger().warning("Image conversion produced an empty frame")
            self._mark_source_unhealthy(source)
            return
        if self.resize_width and bgr.shape[1] > self.resize_width:
            scale = self.resize_width / float(bgr.shape[1])
            bgr = cv2.resize(
                bgr,
                (self.resize_width, max(1, round(bgr.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )

        # 必须在增强前评估曝光；否则 CLAHE 可能把严重暗光噪声伪装成“清晰纹理”。
        raw_gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        quality = image_quality_score(raw_gray)
        # CLAHE 只处理亮度通道，在阴影与局部强光下保留较稳定的 HSV 色相。
        processed_bgr = (
            enhance_illumination(bgr, self.clahe_clip_limit, self.clahe_grid_size)
            if self.illumination_normalization
            else bgr
        )
        # 正常曝光原图与 CLAHE 图共同提供颜色证据，兼顾色相保真和阴影区域召回率。
        color_source = bgr if self.dual_illumination_color_mask else processed_bgr
        orange_mask = self._clean_mask(
            combined_hsv_mask(
                color_source,
                processed_bgr,
                self.orange_lower,
                self.orange_upper,
            )
        )
        blue_mask = self._clean_mask(
            combined_hsv_mask(
                color_source,
                processed_bgr,
                self.blue_lower,
                self.blue_upper,
            )
        )
        gray = cv2.cvtColor(processed_bgr, cv2.COLOR_BGR2GRAY)
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
        if self.suppress_specular_glare:
            edge_mask = suppress_specular_edges(
                edge_mask,
                bgr,
                self.glare_saturation_max,
                self.glare_value_min,
                self.glare_dilation_size,
            )
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
            self.min_bar_aspect_ratio,
            self.max_bar_width_ratio,
            self.max_bar_height_ratio,
            self.segmented_bar_max_gap_ratio,
            self.segmented_bar_max_gap_cv,
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

        if self.annotated_pub is not None:
            geometry_label = "WAITING FOR DEPTH"
            if self.last_geometry_time is not None:
                geometry_age = (
                    self.get_clock().now() - self.last_geometry_time
                ).nanoseconds * 1e-9
                if 0.0 <= geometry_age <= 1.0:
                    geometry_label = self.geometry_label
            annotated = annotate_detection_frame(
                bgr,
                raw_evidence,
                evidence,
                quality,
                self.roi_top_ratio,
                self.roi_bottom_ratio,
                self.roi_side_margin_ratio,
                geometry_label,
            )
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

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
    """运行有算力上限的 OpenCV 障碍辅助节点。"""
    rclpy.init(args=args)
    node = VisionObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Jazzy 在 Gazebo 仍高速发布图像、launch 同时关闭订阅句柄时，pybind11 偶尔
        # 会在 take_message 抛出转换 RuntimeError。仅在 ROS context 已关闭时把它视为
        # 正常退出；运行期错误仍继续抛出，避免掩盖真正的图像消息兼容问题。
        if rclpy.ok():
            raise
    finally:
        # launch 与终端可能同时发送 SIGINT，清理阶段再次中断也应正常退出。
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

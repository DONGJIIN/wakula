"""可测试的点云地形分割与障碍几何分类。

本模块刻意不依赖 ROS：在线节点、rosbag 离线工具和单元测试使用同一算法。
算法先把前向点云压成二维栅格高度图，再从占多数的栅格高度估计地面平面，避免
台阶顶面、墙面或少量深度飞点把地面拟合拉偏。输出只描述几何，不产生腿部命令。
"""

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


UNKNOWN, CLEAR, STEP, PIT, WALL, BAR, POLE = range(7)


@dataclass(frozen=True)
class GeometryEstimate:
    """一帧点云的有界几何摘要。"""

    valid: bool = False
    obstacle_type: int = UNKNOWN
    confidence: float = 0.0
    ground_height: float = 0.0
    obstacle_height: float = 0.0
    pit_depth: float = 0.0
    slope_pitch: float = 0.0
    slope_roll: float = 0.0
    roughness: float = 0.0
    distance: float = 0.0
    width: float = 0.0
    clearance_height: float = 0.0
    valid_points: int = 0


def _grid_samples(points: np.ndarray, cell_size: float):
    """每个 XY 栅格返回低、中、高三个稳健高度分位数。"""
    xy = np.floor(points[:, :2] / cell_size).astype(np.int32)
    order = np.lexsort((xy[:, 1], xy[:, 0]))
    xy, ordered = xy[order], points[order]
    changes = np.r_[True, np.any(xy[1:] != xy[:-1], axis=1)]
    starts = np.flatnonzero(changes)
    rows = []
    for begin, end in zip(starts, np.r_[starts[1:], len(ordered)]):
        cell = ordered[begin:end]
        # 单点格很容易是飞点；至少两个回波才参与地面与坑洞判定。
        if len(cell) < 2:
            continue
        rows.append(
            (
                float(np.median(cell[:, 0])),
                float(np.median(cell[:, 1])),
                float(np.quantile(cell[:, 2], 0.15)),
                float(np.median(cell[:, 2])),
                float(np.quantile(cell[:, 2], 0.90)),
                len(cell),
            )
        )
    return np.asarray(rows, dtype=np.float64).reshape(-1, 6)


def _dominant_ground_mask(cells: np.ndarray, bin_size: float) -> np.ndarray:
    """在栅格中值高度直方图中选择面积占优的地面层。"""
    z = cells[:, 3]
    origin = float(np.quantile(z, 0.05))
    bins = np.floor((z - origin) / bin_size).astype(np.int32)
    values, counts = np.unique(bins, return_counts=True)
    dominant = values[int(np.argmax(counts))]
    # 相邻一个高度箱也纳入拟合，容许缓坡和深度噪声。
    return np.abs(bins - dominant) <= 1


def _fit_plane(samples: np.ndarray) -> Tuple[float, float, float, float]:
    """拟合 z=ax+by+c，并以 MAD 给出抗离群粗糙度。"""
    design = np.column_stack((samples[:, 0], samples[:, 1], np.ones(len(samples))))
    coefficients, *_ = np.linalg.lstsq(design, samples[:, 3], rcond=None)
    residual = samples[:, 3] - design @ coefficients
    median = float(np.median(residual))
    mad = float(np.median(np.abs(residual - median)) * 1.4826)
    return float(coefficients[0]), float(coefficients[1]), float(coefficients[2]), mad


def analyze_terrain_geometry(
    xyz: np.ndarray,
    *,
    cell_size: float = 0.05,
    ground_bin_size: float = 0.03,
    step_height: float = 0.08,
    pit_depth: float = 0.08,
    wall_height: float = 0.25,
    min_cells: int = 12,
) -> GeometryEstimate:
    """分割地面并识别台阶、坑洞、墙面、横杆和立柱。

    判定是保守的：无效点或缺少地面支撑返回 ``valid=False``。坑洞要求真实的低处回波，
    单纯“没有点”不会被当成坑；这避免透明/反光物体和相机盲区造成危险误判。
    """
    points = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < max(30, min_cells * 2):
        return GeometryEstimate(valid_points=len(points))
    cells = _grid_samples(points, max(0.02, float(cell_size)))
    if len(cells) < min_cells:
        return GeometryEstimate(valid_points=len(points))
    ground_mask = _dominant_ground_mask(cells, max(0.01, float(ground_bin_size)))
    if np.count_nonzero(ground_mask) < max(6, min_cells // 2):
        return GeometryEstimate(valid_points=len(points))
    a, b, c, roughness = _fit_plane(cells[ground_mask])
    plane_cells = a * cells[:, 0] + b * cells[:, 1] + c
    low_relative = cells[:, 2] - plane_cells
    high_relative = cells[:, 4] - plane_cells
    positive = high_relative >= step_height
    negative = low_relative <= -pit_depth
    obstacle_height = max(0.0, float(np.quantile(high_relative, 0.98)))
    measured_pit = max(0.0, -float(np.quantile(low_relative, 0.02)))
    slope_pitch, slope_roll = math.atan(a), math.atan(b)
    ground_height = float(c)

    obstacle_type = CLEAR
    confidence = min(1.0, np.count_nonzero(ground_mask) / max(1.0, min_cells * 2.0))
    distance = float(np.max(points[:, 0]))
    width = 0.0
    clearance = 0.0

    if np.count_nonzero(negative) >= 3:
        selected = cells[negative]
        obstacle_type = PIT
        distance = float(np.quantile(selected[:, 0], 0.10))
        width = float(np.ptp(selected[:, 1]) + cell_size)
        confidence = min(1.0, 0.45 + 0.08 * len(selected))
    elif np.count_nonzero(positive) >= 3:
        selected_cells = cells[positive]
        distance = float(np.quantile(selected_cells[:, 0], 0.10))
        width = float(np.ptp(selected_cells[:, 1]) + cell_size)
        # 只看障碍前缘附近的原始点，利用垂直/横向跨度区分几何类别。
        front = points[
            (points[:, 0] >= distance - cell_size)
            & (points[:, 0] <= distance + 2.0 * cell_size)
        ]
        if len(front) >= 8:
            z_span = float(np.ptp(front[:, 2]))
            y_span = float(np.ptp(front[:, 1]))
            low_clearance = float(np.quantile(front[:, 2], 0.10) - (a * distance + c))
            x_span = float(np.ptp(front[:, 0]))
            vertical_score = min(1.0, z_span / max(wall_height, 1e-3))
            if z_span >= wall_height and y_span >= 0.25 and low_clearance <= step_height:
                obstacle_type = WALL
                confidence = 0.55 + 0.35 * vertical_score
            elif z_span >= 0.12 and y_span >= 0.25 and low_clearance > step_height:
                obstacle_type = BAR
                clearance = max(0.0, low_clearance)
                confidence = min(0.90, 0.55 + y_span * 0.35)
            elif z_span >= 0.15 and y_span <= 0.18 and x_span <= 0.18:
                obstacle_type = POLE
                confidence = min(0.88, 0.50 + z_span * 0.8)
            else:
                obstacle_type = STEP
                confidence = min(0.92, 0.50 + obstacle_height)
        else:
            obstacle_type = STEP
            confidence = 0.50

    return GeometryEstimate(
        valid=True,
        obstacle_type=obstacle_type,
        confidence=float(np.clip(confidence, 0.0, 1.0)),
        ground_height=ground_height,
        obstacle_height=obstacle_height,
        pit_depth=measured_pit,
        slope_pitch=slope_pitch,
        slope_roll=slope_roll,
        roughness=roughness,
        distance=max(0.0, distance),
        width=max(0.0, width),
        clearance_height=max(0.0, clearance),
        valid_points=len(points),
    )

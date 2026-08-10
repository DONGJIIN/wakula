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
    """迭代拟合 ``z=ax+by+c``，并以 MAD 剔除残余离群格。

    高度直方图已经去掉大部分障碍，但台阶边缘和混合像素仍可能进入地面候选。
    两轮 MAD 裁剪比一次普通最小二乘稳定，同时保持确定性和很低的计算量。
    """
    design = np.column_stack((samples[:, 0], samples[:, 1], np.ones(len(samples))))
    heights = samples[:, 3]
    active = np.ones(len(samples), dtype=bool)
    coefficients = np.zeros(3, dtype=np.float64)
    for _ in range(3):
        if np.count_nonzero(active) < 6:
            break
        coefficients, *_ = np.linalg.lstsq(design[active], heights[active], rcond=None)
        residual = heights - design @ coefficients
        center = float(np.median(residual[active]))
        mad = float(np.median(np.abs(residual[active] - center)) * 1.4826)
        # 深度相机地面噪声通常为毫米级；1 cm 下限避免 MAD 接近零时误删正常点。
        updated = np.abs(residual - center) <= max(0.01, 3.0 * mad)
        if np.array_equal(updated, active) or np.count_nonzero(updated) < 6:
            break
        active = updated
    residual = heights - design @ coefficients
    center = float(np.median(residual[active]))
    mad = float(np.median(np.abs(residual[active] - center)) * 1.4826)
    return float(coefficients[0]), float(coefficients[1]), float(coefficients[2]), mad


def _largest_connected_region(
    cells: np.ndarray, candidate_mask: np.ndarray, cell_size: float
) -> np.ndarray:
    """返回候选高度格中最大的八邻域连通区域下标。

    过去只要任意三个异常格就会触发障碍，三个互不相邻的飞点也可能造成误检。
    真正的台阶、坑洞和墙面应在 XY 高度栅格中形成连续表面，因此先做连通域筛选。
    """
    candidate_indices = np.flatnonzero(candidate_mask)
    if not len(candidate_indices):
        return candidate_indices
    coordinates = np.rint(cells[candidate_indices, :2] / cell_size).astype(np.int32)
    by_coordinate = {
        (int(coordinate[0]), int(coordinate[1])): int(index)
        for coordinate, index in zip(coordinates, candidate_indices)
    }
    remaining = set(by_coordinate)
    largest = []
    while remaining:
        seed = remaining.pop()
        stack = [seed]
        component = []
        while stack:
            coordinate = stack.pop()
            component.append(by_coordinate[coordinate])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbor = (coordinate[0] + dx, coordinate[1] + dy)
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        stack.append(neighbor)
        if len(component) > len(largest):
            largest = component
    return np.asarray(largest, dtype=np.int64)


def analyze_terrain_geometry(
    xyz: np.ndarray,
    *,
    cell_size: float = 0.05,
    ground_bin_size: float = 0.03,
    step_height: float = 0.08,
    pit_depth: float = 0.08,
    wall_height: float = 0.25,
    min_cells: int = 12,
    min_region_cells: int = 3,
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
    # 用首次平面把同一坡面上落入其他高度箱的地面格吸收回来，再稳健重拟合。
    # 这一步可避免长坡因直方图被切成多个高度层而只使用一小段地面。
    initial_plane = a * cells[:, 0] + b * cells[:, 1] + c
    expanded_ground = np.abs(cells[:, 3] - initial_plane) <= max(
        0.04, 2.0 * ground_bin_size
    )
    if np.count_nonzero(expanded_ground) >= np.count_nonzero(ground_mask):
        ground_mask = expanded_ground
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

    minimum_region = max(2, int(min_region_cells))
    negative_region = _largest_connected_region(cells, negative, cell_size)
    positive_region = _largest_connected_region(cells, positive, cell_size)

    if len(negative_region) >= minimum_region:
        selected = cells[negative_region]
        measured_pit = max(
            0.0, -float(np.quantile(low_relative[negative_region], 0.10))
        )
        obstacle_type = PIT
        distance = float(np.quantile(selected[:, 0], 0.10))
        width = float(np.ptp(selected[:, 1]) + cell_size)
        confidence = min(0.96, 0.42 + 0.07 * len(selected))
    elif len(positive_region) >= minimum_region:
        selected_cells = cells[positive_region]
        obstacle_height = max(
            0.0, float(np.quantile(high_relative[positive_region], 0.90))
        )
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
                confidence = min(
                    0.96, 0.50 + 0.30 * vertical_score + 0.02 * len(selected_cells)
                )
            elif z_span >= 0.12 and y_span >= 0.25 and low_clearance > step_height:
                obstacle_type = BAR
                clearance = max(0.0, low_clearance)
                confidence = min(
                    0.92, 0.50 + y_span * 0.35 + 0.02 * len(selected_cells)
                )
            elif z_span >= 0.15 and y_span <= 0.18 and x_span <= 0.18:
                obstacle_type = POLE
                confidence = min(
                    0.90, 0.46 + z_span * 0.8 + 0.02 * len(selected_cells)
                )
            else:
                obstacle_type = STEP
                confidence = min(
                    0.92, 0.46 + obstacle_height + 0.025 * len(selected_cells)
                )
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

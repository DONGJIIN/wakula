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
    """一帧点云的有界几何摘要。

    ``ground_height``、``slope_pitch`` 和 ``slope_roll`` 共同描述目标坐标系中的
    地面平面。下游必须用这三个量计算“点相对地面的高度”，不能直接用 ``z`` 判断
    障碍；四足的 ``base_link`` 通常远高于地面，低矮台阶在该坐标系里的 z 可能为负。
    """

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


def navigation_obstacle_points(
    xyz: np.ndarray,
    estimate: GeometryEstimate,
    *,
    minimum_height_above_ground: float = 0.05,
    maximum_points: int = 5000,
) -> np.ndarray:
    """从前向点云中只保留真正高于局部地面的 Nav2 标障点。

    Nav2 的 PointCloud2 obstacle layer 按消息坐标系中的绝对 z 过滤点；它并不知道
    当前地面是平地还是坡面。如果直接发布原始 ROI，10°/14° 合法坡面会随着 x 增大而
    高于固定 z 阈值，最终被代价地图错误封死。本函数用本帧稳健地面平面
    ``z = tan(pitch)*x + tan(roll)*y + ground_height`` 计算残差，只发布比地面高出
    指定阈值的点。几何估计无效时返回空集，让上层安全评估负责停车，而不是用未经
    解释的点云污染代价地图。

    降采样采用等间隔索引而非随机抽样，使 rosbag 回放、测试和 CI 结果可重复。
    """
    points = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if not estimate.valid or not len(points):
        return np.empty((0, 3), dtype=np.float32)
    plane = (
        math.tan(float(estimate.slope_pitch)) * points[:, 0]
        + math.tan(float(estimate.slope_roll)) * points[:, 1]
        + float(estimate.ground_height)
    )
    threshold = max(0.0, float(minimum_height_above_ground))
    relative_height = points[:, 2] - plane
    selected = points[relative_height >= threshold]
    if estimate.obstacle_type == PIT and estimate.pit_depth > 0.0:
        # 坑洞的真实回波低于地面，直接发布后可能落到 costmap 的绝对 z 下限以下；完全
        # 不发布又会造成“上层知道有坑、局部规划器却看不到”的死锁。把已经由连通域确认
        # 的低回波投影到局部地面稍上方，形成仅供 Nav2 绕行的虚拟障碍点。未知/无回波
        # 区域仍不会被凭空当成坑，安全性前提与几何分类保持一致。
        pit_gate = max(threshold, min(float(estimate.pit_depth) * 0.60, 0.12))
        pit_mask = relative_height <= -pit_gate
        if np.any(pit_mask):
            virtual_pit = points[pit_mask].copy()
            virtual_pit[:, 2] = plane[pit_mask] + max(threshold, 0.05)
            selected = (
                virtual_pit
                if not len(selected)
                else np.vstack((selected, virtual_pit))
            )
    limit = max(1, int(maximum_points))
    if len(selected) > limit:
        indices = np.linspace(0, len(selected) - 1, limit, dtype=np.int64)
        selected = selected[indices]
    return selected.astype(np.float32, copy=False)


def _grid_samples(points: np.ndarray, cell_size: float):
    """每个 XY 栅格返回低、中、高三个稳健高度分位数。

    旧实现为每个栅格分别调用 ``np.quantile``。一帧 RGB-D 点云通常会形成数百个栅格，
    Python 循环和数百次临时数组分配在 RK3588 上比实际平面拟合更昂贵。这里一次性按
    ``(cell_x, cell_y, z)`` 排序，再以向量索引计算线性分位数；复杂度仍为
    ``O(N log N)``，但热路径只进入 NumPy 数次。XY 使用格内均值，它一定仍落在原格内，
    并且比逐格中位数少两次排序。高度分位数保持与 NumPy 默认 linear 方法一致。
    """
    xy = np.floor(points[:, :2] / cell_size).astype(np.int32)
    # lexsort 的最后一个键优先，因此先按格 x/y 分组，再让每个格内的 z 单调递增。
    order = np.lexsort((points[:, 2], xy[:, 1], xy[:, 0]))
    xy, ordered = xy[order], points[order]
    changes = np.r_[True, np.any(xy[1:] != xy[:-1], axis=1)]
    starts = np.flatnonzero(changes)
    counts = np.diff(np.r_[starts, len(ordered)])
    # 单点格很容易是飞点；至少两个回波才参与地面与坑洞判定。
    valid = counts >= 2
    if not np.any(valid):
        return np.empty((0, 6), dtype=np.float64)
    valid_starts = starts[valid]
    valid_counts = counts[valid]

    sums_x = np.add.reduceat(ordered[:, 0], starts)[valid]
    sums_y = np.add.reduceat(ordered[:, 1], starts)[valid]

    def linear_quantile(quantile: float) -> np.ndarray:
        """在已经按格内 z 排序的数组上向量计算一个分位数。"""
        positions = (valid_counts - 1) * quantile
        lower = np.floor(positions).astype(np.int64)
        upper = np.ceil(positions).astype(np.int64)
        weight = positions - lower
        lower_values = ordered[valid_starts + lower, 2]
        upper_values = ordered[valid_starts + upper, 2]
        return lower_values + (upper_values - lower_values) * weight

    return np.column_stack(
        (
            sums_x / valid_counts,
            sums_y / valid_counts,
            linear_quantile(0.15),
            linear_quantile(0.50),
            linear_quantile(0.90),
            valid_counts,
        )
    ).astype(np.float64, copy=False)


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
    # ``_grid_samples`` 最初用 floor(x / cell_size) 建格。这里必须使用同一规则恢复
    # 栅格坐标：若用四舍五入，位于相邻格两侧的 0.099 m 和 0.101 m 可能都变成索引 2，
    # 字典随后覆盖其中一格，使细台阶、横杆或立柱的连通区域被错误缩小。
    coordinates = np.floor(
        cells[candidate_indices, :2] / cell_size
    ).astype(np.int32)
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


def _region_has_support(
    cells: np.ndarray,
    region: np.ndarray,
    min_region_cells: int,
    min_region_points: int,
) -> bool:
    """同时检查异常区域的空间连续性和原始回波数量。

    连续的少量飞点可能恰好落入相邻栅格，仅检查格数仍会误报。第 6 列保存每格原始点数，
    因而可在不增加一次点云遍历的前提下要求真实表面具有足够回波支撑。
    """
    return len(region) >= max(2, int(min_region_cells)) and int(
        np.sum(cells[region, 5])
    ) >= max(4, int(min_region_points))


def analyze_terrain_geometry(
    xyz: np.ndarray,
    *,
    cell_size: float = 0.05,
    ground_bin_size: float = 0.03,
    step_height: float = 0.08,
    pit_depth: float = 0.08,
    wall_height: float = 0.25,
    bar_min_clearance: float = 0.18,
    min_cells: int = 12,
    min_region_cells: int = 3,
    min_region_points: int = 12,
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

    negative_region = _largest_connected_region(cells, negative, cell_size)
    positive_region = _largest_connected_region(cells, positive, cell_size)
    negative_supported = _region_has_support(
        cells,
        negative_region,
        min_region_cells,
        min_region_points,
    )
    positive_supported = _region_has_support(
        cells,
        positive_region,
        min_region_cells,
        min_region_points,
    )

    if negative_supported:
        selected = cells[negative_region]
        measured_pit = max(
            0.0, -float(np.quantile(low_relative[negative_region], 0.10))
        )
        obstacle_type = PIT
        distance = float(np.quantile(selected[:, 0], 0.10))
        width = float(np.ptp(selected[:, 1]) + cell_size)
        confidence = min(0.96, 0.42 + 0.07 * len(selected))
    elif positive_supported:
        selected_cells = cells[positive_region]
        supporting_points = int(np.sum(selected_cells[:, 5]))
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
        front_relative = front[:, 2] - (
            a * front[:, 0] + b * front[:, 1] + c
        )
        elevated_front = front[front_relative >= step_height]
        elevated_relative = front_relative[front_relative >= step_height]
        if len(elevated_front) >= 8:
            # 只在高于地面的物体回波中估计净空；否则同一 x 切片的地面点会把横杆
            # low_clearance 拉到零，导致比赛中的悬空细杆被误判为墙。
            z_span = float(np.ptp(elevated_front[:, 2]))
            y_span = float(np.ptp(elevated_front[:, 1]))
            low_clearance = float(np.quantile(elevated_relative, 0.10))
            x_span = float(np.ptp(elevated_front[:, 0]))
            vertical_score = min(1.0, z_span / max(wall_height, 1e-3))
            if (
                obstacle_height >= wall_height
                and z_span >= wall_height * 0.70
                and y_span >= 0.25
                and low_clearance < max(bar_min_clearance, step_height * 1.5)
            ):
                obstacle_type = WALL
                confidence = min(
                    0.96,
                    0.50
                    + 0.30 * vertical_score
                    + 0.01 * len(selected_cells)
                    + 0.002 * supporting_points,
                )
            elif (
                obstacle_height >= 0.12
                and y_span >= 0.25
                and low_clearance >= max(step_height, bar_min_clearance)
            ):
                obstacle_type = BAR
                clearance = max(0.0, low_clearance)
                confidence = min(
                    0.92,
                    0.50
                    + y_span * 0.35
                    + 0.01 * len(selected_cells)
                    + 0.002 * supporting_points,
                )
            elif z_span >= 0.15 and y_span <= 0.18 and x_span <= 0.18:
                obstacle_type = POLE
                confidence = min(
                    0.90,
                    0.46
                    + z_span * 0.8
                    + 0.01 * len(selected_cells)
                    + 0.002 * supporting_points,
                )
            else:
                obstacle_type = STEP
                confidence = min(
                    0.92,
                    0.46
                    + obstacle_height
                    + 0.012 * len(selected_cells)
                    + 0.002 * supporting_points,
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

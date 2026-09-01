"""可测试的点云地面分割与障碍几何分类核心。

职责
----
本模块不依赖 ROS，在线节点、rosbag 回放和单元测试共用同一算法。前向点云先转换为二维
高度栅格，再用近场连续表面锚定地面，避免宽台阶/桥面或少量飞点拉偏平面；随后依据真实
连通回波估计 STEP、PIT、WALL、BAR 和 POLE。输出只描述几何，不产生腿部动作。

真机标定入口
------------
本模块函数参数由 ``config/terrain.yaml`` 提供，禁止在这里维护第二套设备阈值。先校准
点云单位与 TF，再用已知高度、坑深、坡角、横杆净空和立柱宽度的标定物逐组调整地面、
类别与连通门；将 ``TerrainFeatures`` 米制字段与物理真值比较。采集矩阵、回放命令和
验收线见根目录 ``instruction.txt`` 第五节。

安全边界
--------
无回波不自动判坑，单个高/低飞点不构成障碍，坡面不能通过删除 Nav2 点云层来“修复”。
修改几何规则必须同时添加纯函数回归，并用未参与调参的真机 bag 检查危险类别召回率。
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
    lateral_offset: float = 0.0
    width: float = 0.0
    structure_heading: float = 0.0
    structure_heading_confidence: float = 0.0
    clearance_height: float = 0.0
    valid_points: int = 0
    # Internal state hint for TerrainAnalyzer; not published as a ROS measurement.  True means a
    # broad, well-observed surface moved as a whole relative to the stored ground-height prior, so
    # the frame must be UNKNOWN while the stateful node decides whether to discard that prior.
    ground_reference_conflict: bool = False


def navigation_obstacle_points(
    xyz: np.ndarray,
    estimate: GeometryEstimate,
    *,
    minimum_height_above_ground: float = 0.05,
    maximum_points: int = 5000,
) -> np.ndarray:
    """从前向点云中只保留真正高于局部地面的 Nav2 标障点。

    Nav2 的 PointCloud2 obstacle layer 按消息坐标系中的绝对 z 过滤点；它并不知道
    当前地面是平地还是坡面。如果直接发布原始 ROI，11.3°/14° 合法坡面会随着 x 增大而
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
        # 不发布又会造成“上层知道有坑、局部规划器却看不到”的危险。把已经由连通域确认
        # 的低回波投影到局部地面稍上方，阻止 Nav2 在运动控制器接管前误驶入坑；Nav2
        # 只负责到达坑区入口，越过坑区后再恢复导航。未知/无回波不会被凭空当成坑。
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
    """用机器人近场锚定地面高度层，而不是盲选全 ROI 的最大平面。

    桥面、宽台阶或坡面在画面里可能比入口前的地面占更多栅格。若直接把全 ROI 中面积
    最大的高度层当作地面，真实地面会落到拟合平面下方，随后被稳定误报为 ``PIT``。
    机器人当前脚下延伸到相机近场的表面才是可靠地面先验，因此优先在最近约 30%（上限
    0.65 m）的 x 范围选择主高度箱，再把同一高度层扩展到整幅 ROI。近场确实缺点时才
    回退到全局直方图，保留相机已经位于桥面/台阶顶部时的可用性。
    """
    z = cells[:, 3]
    x = cells[:, 0]
    x_low, x_high = np.quantile(x, (0.02, 0.98))
    anchor_span = min(0.65, max(0.35, 0.30 * float(x_high - x_low)))
    anchor_mask = x <= float(x_low) + anchor_span
    # 至少六个格才能拟合平面；稀疏近场使用原有全局面积主层作为安全退化。
    voting_z = z[anchor_mask] if np.count_nonzero(anchor_mask) >= 6 else z
    origin = float(np.quantile(voting_z, 0.05))
    bins = np.floor((z - origin) / bin_size).astype(np.int32)
    voting_bins = bins[anchor_mask] if np.count_nonzero(anchor_mask) >= 6 else bins
    values, counts = np.unique(voting_bins, return_counts=True)
    # 近距离面对墙/高台时，其顶面可能在图像中占据的栅格比脚下地面更多；直接选最大
    # 高度箱会把墙顶当成地面，并把真正地面反报为深坑。地面应是近场“最低且得到足够
    # 连续支持”的表面：候选至少六格且达到最大箱的 20%。零散低飞点达不到门限，真实
    # 地面即使被障碍部分遮挡仍能胜出；机器人已经位于平台时，近场没有更低支持层，
    # 自然仍选择平台。该规则不假定 base_link 的绝对安装高度。
    support_threshold = max(6, int(math.ceil(float(np.max(counts)) * 0.20)))
    supported = values[counts >= support_threshold]
    dominant = int(np.min(supported)) if len(supported) else values[int(np.argmax(counts))]
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


def _clear_corridor_coverage(
    cells: np.ndarray,
    support_mask: np.ndarray,
    *,
    cell_size: float,
    corridor_half_width: float,
    start_x: float,
    required_distance: float,
    maximum_gap: float,
    minimum_lateral_fraction: float,
) -> tuple[bool, float]:
    """Verify that CLEAR means a continuously observed central ground corridor.

    A depth camera or 3-D lidar cannot distinguish a deep/absorptive pit from an ordinary
    no-return pixel using XYZ values alone.  Missing returns must therefore never be invented as
    ``PIT``; however, treating them as ``CLEAR`` is equally unsafe.  This guard collapses supported
    ground cells into longitudinal bins inside the robot's physical body corridor and requires a
    sufficiently wide, nearly continuous strip up to ``required_distance``.

    ``maximum_gap`` is the largest tolerated *unobserved* longitudinal run in metres.  Decrease it
    when a real pit or field edge is missed; increase it only enough to cover measured speckle
    dropout on labelled flat-ground bags.  ``minimum_lateral_fraction`` prevents a narrow line of
    edge returns from claiming that the whole walking corridor was observed.

    Returns:
        ``(valid, score)`` where score is the supported longitudinal-bin fraction.  The caller
        folds this score into CLEAR confidence; a failed guard publishes UNKNOWN/invalid, not PIT.
    """
    size = max(0.02, float(cell_size))
    half_width = max(size * 0.5, float(corridor_half_width))
    # Decimal YAML values such as 0.15/0.05 may evaluate to 2.999999999... in binary floating
    # point.  The small dimensionless epsilon keeps exact physical grid boundaries inclusive while
    # remaining many orders below any sensor resolution.
    first_bin = int(math.floor(float(start_x) / size + 1e-9))
    last_bin = int(math.floor(float(required_distance) / size + 1e-9))
    if last_bin <= first_bin or len(cells) != len(support_mask):
        return False, 0.0

    supported = cells[
        np.asarray(support_mask, dtype=bool)
        & (np.abs(cells[:, 1]) <= half_width)
    ]
    total_bins = last_bin - first_bin + 1
    if not len(supported):
        return False, 0.0

    coordinates = np.floor(supported[:, :2] / size).astype(np.int32)
    # At least this many distinct lateral cells must support each x slice.  Requiring a fraction of
    # the physical corridor keeps the rule invariant when grid_cell_size changes during tuning.
    expected_lateral_bins = max(1, int(math.floor(2.0 * half_width / size)) + 1)
    minimum_lateral_bins = max(
        1,
        int(
            math.ceil(
                expected_lateral_bins
                * float(np.clip(minimum_lateral_fraction, 0.05, 1.0))
            )
        ),
    )
    occupied = np.zeros(total_bins, dtype=bool)
    for x_index in np.unique(coordinates[:, 0]):
        relative_index = int(x_index) - first_bin
        if not 0 <= relative_index < total_bins:
            continue
        lateral_count = len(
            np.unique(coordinates[coordinates[:, 0] == x_index, 1])
        )
        occupied[relative_index] = lateral_count >= minimum_lateral_bins

    coverage_score = float(np.count_nonzero(occupied)) / float(total_bins)
    # Include leading/trailing gaps: seeing only the toes, or ground beyond a large missing strip,
    # must not approve the unknown region in between.
    padded = np.r_[True, occupied, True]
    transitions = np.flatnonzero(padded[1:] != padded[:-1])
    missing_runs = transitions.reshape(-1, 2) if len(transitions) else np.empty((0, 2))
    largest_missing_bins = (
        int(np.max(missing_runs[:, 1] - missing_runs[:, 0]))
        if len(missing_runs)
        else 0
    )
    tolerated_missing_bins = max(
        0, int(math.floor(float(maximum_gap) / size + 1e-9))
    )
    return largest_missing_bins <= tolerated_missing_bins, coverage_score


def _connected_regions(
    cells: np.ndarray, candidate_mask: np.ndarray, cell_size: float
) -> list[np.ndarray]:
    """返回候选高度格的所有八邻域连通区。

    过去只要任意三个异常格就会触发障碍，三个互不相邻的飞点也可能造成误检。
    真正的台阶、坑洞和墙面应在 XY 高度栅格中形成连续表面，因此先做连通域筛选。

    不在此处只保留“面积最大”的一块：2.5 m ROI 内可能同时看到近处细杆
    和远处宽墙，远墙格数更多却不是机器人当前应对准的入口。上层会先对每块
    执行格数/回波数守卫，再从正负高度区中选前缘最近者。
    """
    candidate_indices = np.flatnonzero(candidate_mask)
    if not len(candidate_indices):
        return []
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
    regions = []
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
        regions.append(np.asarray(component, dtype=np.int64))
    return regions


def _largest_connected_region(
    cells: np.ndarray, candidate_mask: np.ndarray, cell_size: float
) -> np.ndarray:
    """返回最大连通区，仅保留给旧纯函数调用与回归测试。

    在线分类不再用该函数决定“当前障碍”，否则远处宽障碍会吞掉近处
    细障碍。
    """
    regions = _connected_regions(cells, candidate_mask, cell_size)
    if not regions:
        return np.empty(0, dtype=np.int64)
    return max(regions, key=len)


def _region_front_distance(cells: np.ndarray, region: np.ndarray) -> float:
    """返回连通区稳健前缘距离（m，``base_link`` x 轴）。"""
    if not len(region):
        return float("inf")
    return float(np.quantile(cells[region, 0], 0.10))


def _narrow_region_is_pole_like(
    points: np.ndarray,
    cells: np.ndarray,
    region: np.ndarray,
    *,
    plane_a: float,
    plane_b: float,
    plane_c: float,
    step_height: float,
    cell_size: float,
) -> bool:
    """在细障碍绕过普通连通格数门前，先确认其三维立柱形态。

    70 mm 规则立柱在 5 cm 高度图里可能只占一两格，所以不能与墙/台阶
    共用四格门。但若仅凭“单格点多”就把它排在远处真障碍前，密集飞点
    会遮蔽后方几何。因此这里先在该区域的 XY 包围内要求足够的垂直高度，
    并限制 x/y 宽度；后续正式分类仍会独立复核。
    """
    if not len(region):
        return False
    selected = cells[region]
    x_low = float(np.min(selected[:, 0])) - cell_size
    x_high = float(np.max(selected[:, 0])) + cell_size
    y_low = float(np.min(selected[:, 1])) - cell_size
    y_high = float(np.max(selected[:, 1])) + cell_size
    local = points[
        (points[:, 0] >= x_low)
        & (points[:, 0] <= x_high)
        & (points[:, 1] >= y_low)
        & (points[:, 1] <= y_high)
    ]
    if len(local) < 8:
        return False
    relative = local[:, 2] - (
        plane_a * local[:, 0] + plane_b * local[:, 1] + plane_c
    )
    elevated = local[relative >= step_height]
    if len(elevated) < 8:
        return False
    return (
        float(np.ptp(elevated[:, 2])) >= 0.15
        and float(np.ptp(elevated[:, 0])) <= 0.18
        and float(np.ptp(elevated[:, 1])) <= 0.18
    )


def _region_has_support(
    region: np.ndarray,
    anomaly_echo_counts: np.ndarray,
    min_region_cells: int,
    min_region_points: int,
) -> bool:
    """同时检查异常区域的空间连续性和异常原始回波数量。

    连续的少量飞点可能恰好落入相邻栅格，仅检查格数仍会误报。这里不能使用
    ``cells[:, 5]`` 的格内总点数：同一格的大量正常地面回波会替少数高度飞点“凑够”
    ``min_region_points``，使近处噪点抢在远处真实墙面之前。调用方传入逐格正/负异常
    回波数，因此支撑门只统计真正越过本类高度阈值的原始点。
    """
    return len(region) >= max(2, int(min_region_cells)) and int(
        np.sum(anomaly_echo_counts[region])
    ) >= max(4, int(min_region_points))


def _anomaly_echo_counts_by_cell(
    points: np.ndarray,
    cells: np.ndarray,
    *,
    cell_size: float,
    plane_a: float,
    plane_b: float,
    plane_c: float,
    step_height: float,
    pit_depth: float,
) -> tuple[np.ndarray, np.ndarray]:
    """统计每个高度格内真正越过正/负阈值的原始回波数。

    高度格用 15/90% 分位数决定“这个格是否异常”，它负责抵抗单点噪声；区域支撑则
    必须回到原始点计数，防止格内正常地面点被误当作障碍证据。这里只对整帧做一次
    向量化残差计算和两次 ``unique``，随后所有连通域复用计数，避免按候选区域反复扫描
    点云而增加 RK3588 延迟。
    """
    positive_counts = np.zeros(len(cells), dtype=np.int64)
    negative_counts = np.zeros(len(cells), dtype=np.int64)
    if not len(points) or not len(cells):
        return positive_counts, negative_counts

    size = max(0.02, float(cell_size))
    cell_coordinates = np.floor(cells[:, :2] / size).astype(np.int32)
    cell_index = {
        (int(coordinate[0]), int(coordinate[1])): index
        for index, coordinate in enumerate(cell_coordinates)
    }
    point_coordinates = np.floor(points[:, :2] / size).astype(np.int32)
    relative = points[:, 2] - (
        plane_a * points[:, 0] + plane_b * points[:, 1] + plane_c
    )

    def accumulate(mask: np.ndarray, output: np.ndarray) -> None:
        if not np.any(mask):
            return
        coordinates, counts = np.unique(
            point_coordinates[mask], axis=0, return_counts=True
        )
        for coordinate, count in zip(coordinates, counts):
            index = cell_index.get((int(coordinate[0]), int(coordinate[1])))
            # ``_grid_samples`` discards single-return cells. Their echoes cannot
            # support a connected height region and therefore have no output slot.
            if index is not None:
                output[index] = int(count)

    accumulate(relative >= step_height, positive_counts)
    accumulate(relative <= -pit_depth, negative_counts)
    return positive_counts, negative_counts


def obstacle_front_heading(
    selected_cells: np.ndarray,
    distance: float,
    cell_size: float,
) -> tuple[float, float]:
    """Estimate the traversal normal of the nearest obstacle edge.

    A flat bridge or stair has almost no plane gradient, so slope pitch/roll
    cannot tell the mission which way to face.  The nearest boundary, however,
    is normally a line across the obstacle entrance.  PCA finds that line's
    tangent and the perpendicular is the desired crossing direction.  The
    normal is folded toward the robot's forward half-plane, yielding a body
    relative correction in ``[-pi/2, pi/2]``.

    Only a narrow band behind the measured front is used.  This deliberately
    ignores the full platform footprint: its long axis can be distorted by
    occlusion, T-shaped geometry, or a neighbouring obstacle.  Ambiguous,
    nearly point-like, or isotropic samples return zero confidence so downstream
    code falls back to centring rather than inventing an orientation.
    """
    cells = np.asarray(selected_cells, dtype=np.float64)
    if cells.ndim != 2 or cells.shape[1] < 2 or len(cells) < 4:
        return 0.0, 0.0
    band_depth = max(0.16, 3.5 * max(0.02, float(cell_size)))
    front = cells[
        (cells[:, 0] >= float(distance) - max(0.02, float(cell_size)))
        & (cells[:, 0] <= float(distance) + band_depth)
    ]
    if len(front) < 4:
        return 0.0, 0.0
    xy = front[:, :2]
    centered = xy - np.mean(xy, axis=0)
    covariance = centered.T @ centered / max(1, len(centered) - 1)
    values, vectors = np.linalg.eigh(covariance)
    major_value = max(0.0, float(values[1]))
    minor_value = max(0.0, float(values[0]))
    tangent = vectors[:, 1]
    tangent_span = float(np.ptp(centered @ tangent))
    normal = np.asarray((-tangent[1], tangent[0]), dtype=np.float64)
    # Both normals describe the same entrance line.  Select the one that points
    # most forward, then fold numerical boundary cases into the documented range.
    if normal[0] < 0.0:
        normal = -normal
    heading = math.atan2(float(normal[1]), float(normal[0]))
    while heading > math.pi * 0.5:
        heading -= math.pi
    while heading < -math.pi * 0.5:
        heading += math.pi
    anisotropy = (major_value - minor_value) / max(major_value, 1e-9)
    span_score = min(1.0, tangent_span / 0.45)
    sample_score = min(1.0, len(front) / 10.0)
    confidence = float(np.clip(anisotropy * span_score * sample_score, 0.0, 1.0))
    if tangent_span < max(0.15, 2.5 * float(cell_size)) or confidence < 0.25:
        return 0.0, 0.0
    return heading, confidence


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
    ground_height_prior: float | None = None,
    ground_prior_max_height_shift: float = 0.10,
    clear_ground_corridor_half_width: float = 0.25,
    clear_ground_start_x: float = 0.10,
    clear_ground_required_distance: float = 0.80,
    clear_ground_max_gap: float = 0.15,
    clear_ground_min_lateral_fraction: float = 0.25,
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
    # 在连续运行中，base_link 下的脚下地面高度不会在一帧内突变 20～30 cm。若近场
    # 被墙顶或平台前缘完全遮住，当前直方图可能只剩障碍表面；使用上一段明确可通行
    # 地面的高度作短期锚点，可把该表面恢复为“正高度障碍”而不是倒置成坑。prior 仅由
    # TerrainAnalyzer 的 CLEAR 帧更新，离线纯函数调用不提供时保持原有无状态行为。
    prior_is_valid = (
        ground_height_prior is not None
        and math.isfinite(float(ground_height_prior))
    )
    if prior_is_valid and abs(c - float(ground_height_prior)) > max(
        float(ground_prior_max_height_shift), 3.0 * ground_bin_size
    ):
        prior_mask = np.abs(cells[:, 3] - float(ground_height_prior)) <= max(
            0.04, 2.0 * ground_bin_size
        )
        if np.count_nonzero(prior_mask) >= 6:
            ground_mask = prior_mask
            a, b, c, roughness = _fit_plane(cells[ground_mask])
        else:
            # A body-height change moves a broad floor as one piece in base_link.  Pinning that
            # surface to the old height produces a full-width false PIT that can never recover,
            # because the stateful prior is only refreshed by CLEAR frames.  If the newly fitted
            # surface gives continuous corridor support, mark this frame UNKNOWN and let the node
            # count conflicts before discarding the prior.  A thin wall/platform top normally fails
            # coverage and still uses the old height to expose the real
            # positive obstacle.  Missing returns are never converted into a pit here.
            current_coverage_valid, _ = _clear_corridor_coverage(
                cells,
                ground_mask,
                cell_size=cell_size,
                corridor_half_width=clear_ground_corridor_half_width,
                start_x=clear_ground_start_x,
                required_distance=clear_ground_required_distance,
                maximum_gap=clear_ground_max_gap,
                minimum_lateral_fraction=clear_ground_min_lateral_fraction,
            )
            if current_coverage_valid:
                return GeometryEstimate(
                    valid_points=len(points), ground_reference_conflict=True
                )
            # 地面被完全遮挡时不能拟合坡度；保留先验高度并把坡度置零，随后正障碍仍需
            # 连通格/原始点数门限才能成立。先验还受在线节点年龄上限约束，不会永久存在。
            a, b, c, roughness = 0.0, 0.0, float(ground_height_prior), 0.0
    # 当相机贴近墙面启动、地面完全不可见时，高度栅格可能把近乎竖直的墙面错拟合成
    # “地面”，从而在 2.5 m ROI 内外推出几十米相对高度。比赛坡面远低于 45°；超过
    # 该物理上限时，有可靠 CLEAR 历史就退回水平高度先验，否则本帧直接判无效并由
    # 上层 fail-closed 停车，绝不发布荒谬尺寸参与语义识别。
    if math.hypot(float(a), float(b)) > 1.0:
        if prior_is_valid:
            a, b, c, roughness = 0.0, 0.0, float(ground_height_prior), 0.0
        else:
            return GeometryEstimate(valid_points=len(points))
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
    # Candidate cells come from robust height quantiles, while support and confidence
    # must count only raw echoes that actually cross the corresponding threshold. This
    # prevents dense ground returns in a noisy near cell from hiding a real farther object.
    positive_echo_counts, negative_echo_counts = _anomaly_echo_counts_by_cell(
        points,
        cells,
        cell_size=cell_size,
        plane_a=a,
        plane_b=b,
        plane_c=c,
        step_height=step_height,
        pit_depth=pit_depth,
    )
    obstacle_height = max(0.0, float(np.quantile(high_relative, 0.98)))
    measured_pit = max(0.0, -float(np.quantile(low_relative, 0.02)))
    slope_pitch, slope_roll = math.atan(a), math.atan(b)
    ground_height = float(c)

    obstacle_type = CLEAR
    confidence = min(1.0, np.count_nonzero(ground_mask) / max(1.0, min_cells * 2.0))
    distance = float(np.max(points[:, 0]))
    width = 0.0
    lateral_offset = 0.0
    clearance = 0.0
    structure_heading = 0.0
    structure_heading_confidence = 0.0

    # 先给每个连通域单独做回波守卫，再在正高度/负高度候选中选前缘最近者。
    # 不能先选面积最大者：比如 0.55 m 的细杆与 1.5 m 的宽墙同帧出现时，
    # 机器人必须先报告并对准细杆。坑洞也不再因类别而无条件抢占更近台阶。
    candidates = []
    for region in _connected_regions(cells, negative, cell_size):
        if _region_has_support(
            region, negative_echo_counts, min_region_cells, min_region_points
        ):
            region_echoes = int(np.sum(negative_echo_counts[region]))
            # 距离相同时 PIT 排在正障碍前，保留对坑边的保守性。
            candidates.append(
                (
                    _region_front_distance(cells, region),
                    0,
                    "negative",
                    region,
                    False,
                    region_echoes,
                )
            )
    for region in _connected_regions(cells, positive, cell_size):
        ordinary_support = _region_has_support(
            region, positive_echo_counts, min_region_cells, min_region_points
        )
        region_points = (
            int(np.sum(positive_echo_counts[region])) if len(region) else 0
        )
        narrow_pole_support = (
            # 普通连通域已达标时无需再遍历原始点云；该例外仅用于
            # 1～2 格的细杆，避免多障碍帧在 RK3588 上对每个大区域重复做 O(N) 过滤。
            not ordinary_support
            and region_points >= max(12, int(min_region_points))
            and _narrow_region_is_pole_like(
                points,
                cells,
                region,
                plane_a=a,
                plane_b=b,
                plane_c=c,
                step_height=step_height,
                cell_size=cell_size,
            )
        )
        if ordinary_support or narrow_pole_support:
            candidates.append(
                (
                    _region_front_distance(cells, region),
                    1,
                    "positive",
                    region,
                    narrow_pole_support,
                    region_points,
                )
            )

    negative_region = np.empty(0, dtype=np.int64)
    positive_region = np.empty(0, dtype=np.int64)
    negative_supported = False
    positive_supported = False
    narrow_positive_supported = False
    selected_anomaly_echoes = 0
    if candidates:
        (
            _,
            _,
            selected_kind,
            selected_region,
            selected_narrow,
            selected_anomaly_echoes,
        ) = min(
            candidates,
            # 前缘同距离时先保守选坑，同类再选更接近机身中线且
            # 回波区更大者。这也消除 set 遍历顺序对完全对称场景的随机影响。
            key=lambda item: (
                item[0],
                item[1],
                abs(float(np.median(cells[item[3], 1]))),
                -len(item[3]),
                -item[5],
            ),
        )
        if selected_kind == "negative":
            negative_region = selected_region
            negative_supported = True
        else:
            positive_region = selected_region
            positive_supported = _region_has_support(
                positive_region,
                positive_echo_counts,
                min_region_cells,
                min_region_points,
            )
            narrow_positive_supported = bool(selected_narrow)

    if negative_supported:
        selected = cells[negative_region]
        # 当前输出描述的是最近 PIT 连通域，不能把远处墙/台阶的全局
        # 高分位带入 obstacle_height；否则类别虽是坑，Action 闸门却会看到墙高。
        obstacle_height = 0.0
        measured_pit = max(
            0.0, -float(np.quantile(low_relative[negative_region], 0.10))
        )
        obstacle_type = PIT
        distance = float(np.quantile(selected[:, 0], 0.10))
        # 使用整个连通区域的横向中位数而不是极值中心。中位数对边缘缺点、反光飞点
        # 和只看到障碍一侧更稳定，后续可直接计算入口对正角。
        lateral_offset = float(np.median(selected[:, 1]))
        width = float(np.ptp(selected[:, 1]) + cell_size)
        structure_heading, structure_heading_confidence = obstacle_front_heading(
            selected, distance, cell_size
        )
        # 连通格数衡量空间面积，异常回波数衡量重复观测；地面回波不再虚增坑洞置信度。
        confidence = min(
            0.96,
            0.38
            + 0.05 * len(selected)
            + 0.003 * selected_anomaly_echoes,
        )
    elif positive_supported or narrow_positive_supported:
        # 与 PIT 分支对称：选中近处正障碍后，远处坑底不再污染本条台阶/墙/杆
        # 的原子量测。后续帧移近坑区时会独立选中并发布 PIT。
        measured_pit = 0.0
        narrow_only = narrow_positive_supported and not positive_supported
        selected_cells = cells[positive_region]
        # 所有类别置信度和墙面支撑门均只使用高于阈值的原始回波；格内地面点
        # 仍参与地面拟合，但不能替正障碍增加可信度。
        supporting_points = selected_anomaly_echoes
        obstacle_height = max(
            0.0, float(np.quantile(high_relative[positive_region], 0.90))
        )
        distance = float(np.quantile(selected_cells[:, 0], 0.10))
        lateral_offset = float(np.median(selected_cells[:, 1]))
        width = float(np.ptp(selected_cells[:, 1]) + cell_size)
        structure_heading, structure_heading_confidence = obstacle_front_heading(
            selected_cells, distance, cell_size
        )
        # 宽连续凸起还可能是从平地开始的 10° 主斜坡或 14° 木桥引坡。只拟合最近的
        # 0.9 m 候选表面，并同时要求足够 x/y 跨度和低残差；阶梯的离散高度平台即使
        # 总体呈上升趋势，也会因平面残差过大而继续进入 STEP 分支。
        surface_limit = distance + 0.90
        surface_cells = selected_cells[selected_cells[:, 0] <= surface_limit]
        if len(surface_cells) >= 8:
            surface_a, surface_b, surface_c, surface_roughness = _fit_plane(
                surface_cells
            )
            surface_pitch = math.atan(surface_a)
            surface_roll = math.atan(surface_b)
            surface_x_span = float(np.ptp(surface_cells[:, 0]))
            surface_y_span = float(np.ptp(surface_cells[:, 1]))
            if (
                0.10 <= abs(surface_pitch) <= 0.32
                and abs(surface_roll) <= 0.10
                and surface_x_span >= 0.42
                and surface_y_span >= 0.40
                and surface_roughness <= 0.015
            ):
                surface_plane = (
                    surface_a * cells[:, 0]
                    + surface_b * cells[:, 1]
                    + surface_c
                )
                slope_support = ground_mask | (
                    np.abs(cells[:, 3] - surface_plane)
                    <= max(0.04, 2.0 * ground_bin_size)
                )
                coverage_valid, coverage_score = _clear_corridor_coverage(
                    cells,
                    slope_support,
                    cell_size=cell_size,
                    corridor_half_width=clear_ground_corridor_half_width,
                    start_x=clear_ground_start_x,
                    required_distance=clear_ground_required_distance,
                    maximum_gap=clear_ground_max_gap,
                    minimum_lateral_fraction=clear_ground_min_lateral_fraction,
                )
                if not coverage_valid:
                    return GeometryEstimate(valid_points=len(points))
                return GeometryEstimate(
                    valid=True,
                    obstacle_type=CLEAR,
                    confidence=float(
                        np.clip(
                            min(0.96, 0.62 + 0.02 * len(surface_cells))
                            * coverage_score,
                            0.0,
                            1.0,
                        )
                    ),
                    ground_height=float(surface_c),
                    obstacle_height=0.0,
                    pit_depth=0.0,
                    slope_pitch=surface_pitch,
                    slope_roll=surface_roll,
                    roughness=surface_roughness,
                    # 坡面仍然需要一个真实入口距离供任务层对正和 Action 交接。旧值
                    # 使用 ROI 最远点（通常恒为 2.5 m），机器人即使抵达坡脚也不会
                    # 进入 READY，只会反复提交同一个 Nav2 目标直至超时。
                    distance=max(0.0, distance),
                    structure_heading=structure_heading,
                    structure_heading_confidence=structure_heading_confidence,
                    valid_points=len(points),
                )
        # 只看障碍前缘附近的原始点，利用垂直/横向跨度区分几何类别。
        # 既然上面已选定单一最近连通域，这里还必须限制其 y 包围；否则同一
        # x 距离但不相连的另一面墙/杆会污染 z_span、y_span 和净空分类。
        selected_y_low = float(np.min(selected_cells[:, 1])) - cell_size
        selected_y_high = float(np.max(selected_cells[:, 1])) + cell_size
        front = points[
            (points[:, 0] >= distance - cell_size)
            & (points[:, 0] <= distance + 2.0 * cell_size)
            & (points[:, 1] >= selected_y_low)
            & (points[:, 1] <= selected_y_high)
        ]
        front_relative = front[:, 2] - (
            a * front[:, 0] + b * front[:, 1] + c
        )
        elevated_front = front[front_relative >= step_height]
        elevated_relative = front_relative[front_relative >= step_height]
        # 正视薄墙时 RGB-D 可能只能返回墙顶而缺少垂直立面，导致下面的 z_span 不足。
        # 高度图仍能可靠看出“达到墙高、横向连续、沿前进方向很薄”；台阶/平台的踏面
        # 在 x 方向明显更深。该补充判据解决规则 0.30 m 高墙被降级成 STEP 的实测样本。
        positive_x_span = float(np.ptp(selected_cells[:, 0]))
        # 仅用 x 方向厚度只能识别“正对”薄墙：机器人斜看墙面时，墙的长边同时投影到
        # x/y，positive_x_span 会随视角增大并被错误降级成 STEP。对正高度连通区的 XY
        # 做 PCA，最小主轴上的跨度就是与朝向无关的物体厚度。平台/台阶在两个方向都
        # 有明显面积，最小跨度较大；规则高墙约 0.10 m 厚，斜视时仍保持薄轮廓。
        xy = selected_cells[:, :2]
        oriented_thickness = float("inf")
        if len(xy) >= 3:
            centered_xy = xy - np.mean(xy, axis=0)
            covariance = centered_xy.T @ centered_xy / max(1, len(xy) - 1)
            _values, vectors = np.linalg.eigh(covariance)
            minor_axis = vectors[:, 0]
            oriented_thickness = float(np.ptp(centered_xy @ minor_axis))
        # 横杆下方仍能看到地面，同一 XY 格的低分位接近地平面；实体墙会遮挡其脚下
        # 地面，正回波格的低分位也被抬高。该可见性差异可区分“薄墙顶边”和悬空横杆。
        ground_coexistence = float(
            np.mean(
                np.abs(low_relative[positive_region])
                <= max(0.04, 2.0 * ground_bin_size)
            )
        )
        thin_wall_profile = (
            obstacle_height >= wall_height
            and width >= 0.25
            and min(positive_x_span, oriented_thickness)
            <= max(0.18, 3.0 * cell_size)
            and supporting_points >= min_region_points
            and ground_coexistence < 0.35
        )
        if thin_wall_profile:
            obstacle_type = WALL
            confidence = min(
                0.96,
                0.62
                + min(0.18, obstacle_height * 0.45)
                + min(0.12, width * 0.12),
            )
        elif len(elevated_front) >= 8:
            # 只在高于地面的物体回波中估计净空；否则同一 x 切片的地面点会把横杆
            # low_clearance 拉到零，导致比赛中的悬空细杆被误判为墙。
            z_span = float(np.ptp(elevated_front[:, 2]))
            y_span = float(np.ptp(elevated_front[:, 1]))
            low_clearance = float(np.quantile(elevated_relative, 0.10))
            x_span = float(np.ptp(elevated_front[:, 0]))
            vertical_score = min(1.0, z_span / max(wall_height, 1e-3))
            # 限高杆包含两根落地支柱，直接取所有高点的最低值会得到 0，并被误认为墙。
            # 在净空以上按 3 cm 高度箱寻找占优的横向窄带：真正横杆的大多数回波集中
            # 在同一高度，墙面则沿 z 连续分布，任何单一高度带都不会占多数。
            high_mask = elevated_relative >= max(bar_min_clearance, step_height)
            horizontal_band = False
            band_clearance = 0.0
            if np.count_nonzero(high_mask) >= 8:
                high_points = elevated_front[high_mask]
                high_relative = elevated_relative[high_mask]
                bin_size = 0.03
                bin_origin = float(np.min(high_relative))
                # 使用最近箱而不是向下取整，避免恰在 3 cm 边界上的浮点误差把两层
                # 均匀墙面回波挤进同一箱，制造并不存在的“横杆峰值”。
                high_bins = np.rint(
                    (high_relative - bin_origin) / bin_size
                ).astype(np.int32)
                values, counts = np.unique(high_bins, return_counts=True)
                dominant_bin = values[int(np.argmax(counts))]
                # 墙面在各高度箱中近似均匀；横杆则会在杆体高度形成明显峰值。仅看
                # “连续三个箱所占比例”会把低矮墙误判为横杆，因此还要求主箱相对
                # 其余高度箱有足够显著性。
                count_baseline = float(np.median(counts))
                peak_prominence = float(np.max(counts)) / max(1.0, count_baseline)
                band_mask = np.abs(high_bins - dominant_bin) <= 1
                band_points = high_points[band_mask]
                band_relative = high_relative[band_mask]
                band_fraction = len(band_points) / max(1.0, float(len(high_points)))
                band_y_span = (
                    float(np.ptp(band_points[:, 1])) if len(band_points) else 0.0
                )
                band_x_span = (
                    float(np.ptp(band_points[:, 0])) if len(band_points) else 0.0
                )
                horizontal_band = (
                    len(band_points) >= 8
                    and band_fraction >= 0.50
                    and peak_prominence >= 1.80
                    and band_y_span >= 0.25
                    and band_x_span <= 0.20
                    # 横杆整体沿前进方向也必须很薄。台阶顶面可能在最前切片形成同样
                    # 的单高度峰值，但其正高度连通区会向后延伸数十厘米。
                    and positive_x_span <= 0.25
                )
                if horizontal_band:
                    band_clearance = float(np.quantile(band_relative, 0.10))
            if horizontal_band:
                obstacle_type = BAR
                clearance = max(0.0, band_clearance)
                confidence = min(
                    0.96,
                    0.58
                    + min(0.25, float(np.ptp(elevated_front[:, 1])) * 0.20)
                    + 0.002 * supporting_points,
                )
            elif (
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
                and positive_x_span <= 0.25
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

        # “少格但回波多”的例外只为细长立柱保留。若三维前缘没有足够垂直跨度证明
        # POLE，就不能让一个高度恒定的密集深度斑点绕过普通障碍的连通区域门而触发 STEP。
        if narrow_only and obstacle_type != POLE:
            obstacle_type = CLEAR
            obstacle_height = 0.0
            distance = float(np.max(points[:, 0]))
            lateral_offset = 0.0
            width = 0.0
            clearance = 0.0
            confidence = min(
                1.0,
                np.count_nonzero(ground_mask) / max(1.0, min_cells * 2.0),
            )

    if obstacle_type == CLEAR:
        coverage_valid, coverage_score = _clear_corridor_coverage(
            cells,
            ground_mask,
            cell_size=cell_size,
            corridor_half_width=clear_ground_corridor_half_width,
            start_x=clear_ground_start_x,
            required_distance=clear_ground_required_distance,
            maximum_gap=clear_ground_max_gap,
            minimum_lateral_fraction=clear_ground_min_lateral_fraction,
        )
        if not coverage_valid:
            return GeometryEstimate(valid_points=len(points))
        # CLEAR confidence now expresses both plane support and verified path coverage.  A dense
        # side wall can no longer compensate for a missing strip directly in front of the body.
        confidence *= coverage_score

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
        lateral_offset=lateral_offset,
        width=max(0.0, width),
        structure_heading=structure_heading,
        structure_heading_confidence=structure_heading_confidence,
        clearance_height=max(0.0, clearance),
        valid_points=len(points),
    )

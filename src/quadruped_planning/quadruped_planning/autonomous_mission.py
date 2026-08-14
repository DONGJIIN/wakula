"""陌生场地前沿探索、越障入口对正与 Action 交接任务管理器。

节点只编排三个已有能力：从 ``/map`` 选择未知区域边界、用 Nav2 到达自由空间目标、在
``/traversal/guidance`` 连续确认 READY 后调用 ``TraverseObstacle``。它不读取 Gazebo world
坐标，也不生成关节命令；仿真和真机通过同一个 Action 合同替换越障执行器。
"""

from __future__ import annotations

from action_msgs.msg import GoalStatus
from collections import deque
from dataclasses import dataclass
from math import atan2, cos, floor, hypot, isfinite, pi, sin
import time
import signal
from typing import List, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from quadruped_interfaces.action import TraverseObstacle
from quadruped_interfaces.msg import TraversalGuidance
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class Frontier:
    """地图坐标中的一个连通前沿候选。"""

    x: float
    y: float
    cells: int
    distance: float
    score: float


def _origin_yaw(grid: OccupancyGrid) -> float:
    q = grid.info.origin.orientation
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _cell_to_world(grid: OccupancyGrid, col: float, row: float) -> Tuple[float, float]:
    """把栅格中心转换到 map；兼容带旋转的 OccupancyGrid 原点。"""
    resolution = float(grid.info.resolution)
    local_x = (col + 0.5) * resolution
    local_y = (row + 0.5) * resolution
    yaw = _origin_yaw(grid)
    origin = grid.info.origin.position
    return (
        origin.x + cos(yaw) * local_x - sin(yaw) * local_y,
        origin.y + sin(yaw) * local_x + cos(yaw) * local_y,
    )


def world_to_cell(grid: OccupancyGrid, x: float, y: float) -> Optional[Tuple[int, int]]:
    """把 map 坐标转回栅格索引；越界或地图元数据无效时返回 ``None``。

    越障执行器可以在一个地图发布周期内把机器人带到当前 OccupancyGrid 边缘之外。此时
    Nav2 的静态层尚未扩张，任何目标都会以“start outside bounds”立即失败。任务层先用
    本函数等待下一张地图覆盖当前位置，避免以 4 Hz 对 Action server 做无意义重试。
    """
    resolution = float(grid.info.resolution)
    width, height = int(grid.info.width), int(grid.info.height)
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return None
    origin = grid.info.origin.position
    dx, dy = float(x) - origin.x, float(y) - origin.y
    yaw = _origin_yaw(grid)
    # 逆旋转：R(yaw)^T * (world - origin)。
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    col, row = int(floor(local_x / resolution)), int(floor(local_y / resolution))
    return (col, row) if 0 <= col < width and 0 <= row < height else None


def distance_outside_grid(grid: OccupancyGrid, x: float, y: float) -> float:
    """返回点到当前地图矩形的最短外部距离；位于地图内时为 0。

    SLAM Toolbox 的 OccupancyGrid 边界由已插入的扫描端点决定，探索或仿真越障后，
    ``base_link`` 可能短暂比最后一个端点多走几厘米。该连续量让任务层只容忍很小的
    发布滞后，而不是把任意地图外位置误当成可导航区域。
    """
    resolution = float(grid.info.resolution)
    width, height = int(grid.info.width), int(grid.info.height)
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return float("inf")
    origin = grid.info.origin.position
    dx, dy = float(x) - origin.x, float(y) - origin.y
    yaw = _origin_yaw(grid)
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    maximum_x, maximum_y = width * resolution, height * resolution
    outside_x = max(0.0, -local_x, local_x - maximum_x)
    outside_y = max(0.0, -local_y, local_y - maximum_y)
    return hypot(outside_x, outside_y)


def extract_frontiers(
    grid: OccupancyGrid,
    robot_xy: Tuple[float, float],
    *,
    minimum_cells: int = 8,
    occupied_threshold: int = 50,
    minimum_distance: float = 0.55,
    maximum_distance: float = 7.0,
) -> List[Frontier]:
    """提取“自由格且四邻域接触未知格”的连通簇并按信息增益排序。

    只在已确认自由格上放目标，未知格只是探索方向，避免 Nav2 目标本身落入未观测区域。
    ``cells / (1 + distance)`` 优先较大的边界，同时抑制在机器人脚边来回选择碎前沿。
    """
    width, height = int(grid.info.width), int(grid.info.height)
    if width <= 2 or height <= 2 or len(grid.data) != width * height:
        return []
    data = grid.data
    frontier_cells = set()
    for row in range(1, height - 1):
        offset = row * width
        for col in range(1, width - 1):
            index = offset + col
            if not 0 <= int(data[index]) < occupied_threshold:
                continue
            if any(int(data[n]) < 0 for n in (index - 1, index + 1, index - width, index + width)):
                frontier_cells.add((col, row))

    clusters: List[List[Tuple[int, int]]] = []
    while frontier_cells:
        seed = frontier_cells.pop()
        queue = deque([seed])
        cluster = [seed]
        while queue:
            col, row = queue.popleft()
            for dx, dy in ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)):
                neighbor = (col + dx, row + dy)
                if neighbor in frontier_cells:
                    frontier_cells.remove(neighbor)
                    queue.append(neighbor)
                    cluster.append(neighbor)
        if len(cluster) >= max(1, int(minimum_cells)):
            clusters.append(cluster)

    frontiers = []
    for cluster in clusters:
        # 一个开放区域的前沿常是一整圈连通格。直接取全簇均值会落到已探索区域中央，
        # 机器人“到达目标”却完全没有接近未知区。按机器人周向切成 16 个扇区，每个扇区
        # 选一个真实的 frontier cell，目标必然位于已知自由区与未知区的边缘。
        sectors = {}
        for col, row in cluster:
            world_x, world_y = _cell_to_world(grid, col, row)
            angle = atan2(world_y - robot_xy[1], world_x - robot_xy[0])
            sector = int((angle + pi) / (2.0 * pi) * 16.0) % 16
            sectors.setdefault(sector, []).append((col, row, world_x, world_y))
        for cells in sectors.values():
            # 整个连通簇已经通过 minimum_cells；扇区边界只有 1 格也可能是狭窄门洞
            # 的唯一探索方向，不能在第二层过滤中丢掉。
            if len(cells) < max(1, int(minimum_cells) // 6):
                continue
            mean_col = sum(cell[0] for cell in cells) / len(cells)
            mean_row = sum(cell[1] for cell in cells) / len(cells)
            # 只能发布真实自由栅格，不能发布扇区几何均值（后者可能跨过障碍/未知格）。
            _, _, x, y = min(
                cells,
                key=lambda cell: (cell[0] - mean_col) ** 2 + (cell[1] - mean_row) ** 2,
            )
            distance = hypot(x - robot_xy[0], y - robot_xy[1])
            if minimum_distance <= distance <= maximum_distance:
                # 信息增益为主、距离轻微加权；这会先扩展较长边界，同时减少脚边抖动。
                score = len(cells) * (1.0 + min(distance, 3.0) * 0.12)
                frontiers.append(Frontier(x, y, len(cells), distance, score))
    return sorted(frontiers, key=lambda item: item.score, reverse=True)


def choose_frontier(candidates: Sequence[Frontier], blocked: Sequence[Tuple[float, float]], exclusion_radius: float) -> Optional[Frontier]:
    """跳过近期失败或刚访问的前沿，返回当前最高分候选。"""
    radius = max(0.0, float(exclusion_radius))
    for candidate in candidates:
        if all(hypot(candidate.x - x, candidate.y - y) > radius for x, y in blocked):
            return candidate
    return None


def action_obstacle_type(guidance: TraversalGuidance) -> int:
    """把感知类别转换为动作类别，显式区分 CLEAR 与可跨越坡面。

    感知合同用 ``OBSTACLE_CLEAR + traversal_required`` 表达没有凸起但坡度需要专用步态；
    Action 端必须收到独立 SLOPE 编码，不能把它误解成普通平地。
    """
    if (
        guidance.obstacle_type == TraversalGuidance.OBSTACLE_CLEAR
        and guidance.traversal_required
    ):
        return TraverseObstacle.Goal.OBSTACLE_SLOPE
    return int(guidance.obstacle_type)


def nav_status_allows_guarded_handoff(status: int) -> bool:
    """入口导航成功或被障碍边界中止时，允许继续执行严格的同障碍交叉验证。

    ``ABORTED`` 本身绝不代表可以越障；调用方仍必须验证最新点云、置信度、障碍在
    ``map`` 中的位置、距离和横向偏差。它只解决一个比赛特有矛盾：Nav2 正确地把实体
    障碍加入代价地图后，可能恰好在越障控制器应接管的位置中止局部跟踪。
    """
    return int(status) in (
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_ABORTED,
    )


def obstacle_was_completed(
    obstacle_type: int,
    position: Tuple[float, float],
    completed: Sequence[Tuple[int, float, float]],
    radius: float,
) -> bool:
    """仅去重同类别、同位置的已完成障碍，保留密集场地中的相邻异类目标。"""
    return any(
        int(completed_type) == int(obstacle_type)
        and hypot(position[0] - x, position[1] - y) <= max(0.0, float(radius))
        for completed_type, x, y in completed
    )


class AutonomousMission(Node):
    """可运行时启停的探索—对正—越障状态机。"""

    def __init__(self):
        super().__init__("autonomous_mission")
        defaults = {
            "autostart": False, "map_timeout": 2.0, "guidance_timeout": 1.0,
            "frontier_minimum_cells": 8, "frontier_minimum_distance": 0.55,
            "frontier_maximum_distance": 7.0, "frontier_exclusion_radius": 0.65,
            # 仅容忍 SLAM 栅格边界比机器人滞后几厘米；配合滚动全局代价地图继续选择
            # 已知自由前沿。超过该距离仍进入 WAITING_FOR_MAP，防止定位跳变后盲目规划。
            "map_boundary_tolerance": 0.30,
            # DWB 默认 goal checker 容差约 0.25 m。比它更近的入口目标会被立即判成功，
            # 表现为机器人只挪一下就停；任务层直接进入受控交接，不发送伪 Nav2 目标。
            "minimum_approach_goal_distance": 0.35,
            "nav_rejection_retry_delay": 1.0,
            "nav_failure_retry_delay": 1.0,
            "handoff_fallback_max_distance": 1.50,
            "handoff_fallback_max_lateral": 0.50,
            "handoff_fallback_spatial_tolerance": 0.90,
            "obstacle_lock_radius": 0.75,
            "goal_timeout": 45.0, "traversal_timeout": 15.0,
            "minimum_obstacle_confidence": 0.55, "obstacle_confirmation_frames": 3,
            "completed_obstacle_radius": 0.65, "post_traversal_cooldown": 3.0,
            "empty_frontier_confirmations": 20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.params = {name: self.get_parameter(name).value for name in defaults}
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, map_qos)
        self.create_subscription(TraversalGuidance, "/traversal/guidance", self._guidance_callback, 10)
        self.state_pub = self.create_publisher(String, "/autonomy/state", 10)
        self.event_pub = self.create_publisher(String, "/autonomy/event", 10)
        # 独立任务进程拥有自己的运动许可。TRANSIENT_LOCAL 让核心速度门记住最后状态：
        # 启动时解除自主停车，Ctrl-C 时先锁止，再取消异步 Action。核心 SLAM 不需要启动
        # 或管理本节点；它只把这一标准布尔量作为额外的失效安全输入。
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.autonomy_stop_pub = self.create_publisher(
            Bool, "/navigation/autonomy_stop", stop_qos
        )
        self.autonomy_stop_pub.publish(Bool(data=False))
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.traverse_client = ActionClient(self, TraverseObstacle, "/traverse_obstacle")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.enabled = bool(self.params["autostart"])
        self.state = "WAITING_FOR_INPUTS" if self.enabled else "IDLE"
        self.map_msg = None
        self.map_received = 0.0
        self.guidance = None
        self.guidance_received = 0.0
        self.nav_handle = None
        self.nav_send_pending = False
        self.nav_cancel_pending = False
        self.nav_purpose = ""
        self.nav_started = 0.0
        self.nav_target = None
        # approach goal 与其来源障碍绑定；Nav2 成功后用于交叉验证，防止前缘/类别抖动
        # 让严格 READY 漏报，也防止把相邻障碍误交给越障控制器。
        self.nav_obstacle_position = None
        self.nav_retry_until = 0.0
        self.pending_traverse = None
        # READY 时立即冻结障碍物的 map 坐标。Action 执行期间机器人会移动，如果完成后
        # 再用“当前位姿 + 旧相对距离”反算，会把已经越过的障碍错误登记到机器人前方。
        self.pending_traverse_position = None
        self.pending_traverse_started = 0.0
        # send_goal_async 到 goal-response 之间还没有 goal handle，必须另设 pending 锁；
        # 否则 4 Hz 定时器会在服务器响应前重复提交同一个障碍。
        self.traverse_send_pending = False
        self.traverse_cancel_pending = False
        self.traverse_handle = None
        self.traverse_started = 0.0
        self.obstacle_signature = None
        self.obstacle_frames = 0
        self.completed_obstacles = []
        self.blocked_frontiers = []
        self.empty_frontier_count = 0
        self.controller_wait_reported = False
        self.cooldown_until = 0.0
        self.locked_obstacle_position = None
        self.create_timer(0.25, self._tick)
        self._publish_state("mission node ready")

    def _publish_state(self, event=""):
        self.state_pub.publish(String(data=self.state))
        if event:
            self.event_pub.publish(String(data=event))
            self.get_logger().info(f"Autonomy {self.state}: {event}")

    def _publish_immediate_stop(self) -> None:
        """锁住核心速度门；用于独立 launch 退出时确定性停车。"""
        self.autonomy_stop_pub.publish(Bool(data=True))

    def _map_callback(self, msg):
        self.map_msg = msg
        self.map_received = time.monotonic()

    def _robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(t.x), float(t.y), yaw

    def _guidance_callback(self, msg):
        self.guidance = msg
        self.guidance_received = time.monotonic()
        position = self._obstacle_position(msg)
        if position is None or not msg.perception_valid or not msg.traversal_required:
            self.obstacle_signature, self.obstacle_frames = None, 0
            return
        # 不用 0.1 m 取整坐标做严格相等：机器人缓慢移动时障碍世界坐标会因测量噪声
        # 跨过取整边界，导致确认计数永久归零。相同类型且位置漂移不超过 0.35 m 即视为
        # 同一目标；该门限远小于 completed_obstacle_radius，不会把相邻障碍混为一谈。
        signature = (int(msg.obstacle_type), float(position[0]), float(position[1]))
        same_obstacle = (
            self.obstacle_signature is not None
            and signature[0] == self.obstacle_signature[0]
            and hypot(
                signature[1] - self.obstacle_signature[1],
                signature[2] - self.obstacle_signature[2],
            ) <= 0.35
        )
        if same_obstacle:
            self.obstacle_frames += 1
            # 低通更新参考位置，既跟随小幅感知修正，又不会随单帧跳变漂走。
            self.obstacle_signature = (
                signature[0],
                0.8 * self.obstacle_signature[1] + 0.2 * signature[1],
                0.8 * self.obstacle_signature[2] + 0.2 * signature[2],
            )
        else:
            self.obstacle_signature, self.obstacle_frames = signature, 1

    def _obstacle_position(self, msg):
        pose = self._robot_pose()
        if pose is None:
            return None
        return (pose[0] + cos(pose[2]) * msg.distance - sin(pose[2]) * msg.lateral_offset,
                pose[1] + sin(pose[2]) * msg.distance + cos(pose[2]) * msg.lateral_offset)

    def _already_completed(self, msg):
        position = self._obstacle_position(msg)
        if position is None:
            return True
        radius = float(self.params["completed_obstacle_radius"])
        obstacle_type = int(msg.obstacle_type)
        # 比赛障碍可能紧邻布置。旧实现只存坐标，会把刚完成台阶旁的限高杆一并屏蔽；
        # 现在只有“相同几何类别 + 相同 map 邻域”才视为已完成目标。
        return obstacle_was_completed(
            obstacle_type,
            position,
            self.completed_obstacles,
            radius,
        )

    def _matches_obstacle_lock(self, msg):
        """判断最新证据是否仍指向当前正在接近的同一障碍。"""
        if self.locked_obstacle_position is None:
            return True
        position = self._obstacle_position(msg)
        return position is not None and hypot(
            position[0] - self.locked_obstacle_position[0],
            position[1] - self.locked_obstacle_position[1],
        ) <= float(self.params["obstacle_lock_radius"])

    def _fresh_target(self):
        msg = self.guidance
        if (msg is None or time.monotonic() - self.guidance_received > float(self.params["guidance_timeout"])
                or not msg.perception_valid or not msg.traversal_required
                or not isfinite(float(msg.confidence))
                or msg.confidence < float(self.params["minimum_obstacle_confidence"])
                or self.obstacle_frames < int(self.params["obstacle_confirmation_frames"])
                or self._already_completed(msg)):
            return None
        return msg

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = sin(yaw * 0.5), cos(yaw * 0.5)
        return pose

    def _relative_approach_pose(self, guidance):
        robot = self._robot_pose()
        if robot is None:
            return None
        x = robot[0] + cos(robot[2]) * guidance.approach_x - sin(robot[2]) * guidance.approach_y
        y = robot[1] + sin(robot[2]) * guidance.approach_x + cos(robot[2]) * guidance.approach_y
        return self._make_pose(x, y, robot[2] + guidance.approach_yaw)

    def _cancel_nav(self, reason="replace"):
        """只发送一次取消请求，并等待 Nav2 result 回调释放句柄。

        Action 的 cancel future 只说明服务收到请求；真正可以提交新目标的时刻是旧 goal
        result 到达以后。显式锁可避免入口目标与仍在执行的前沿目标互相抢占。
        """
        if self.nav_handle is not None and not self.nav_cancel_pending:
            self.nav_cancel_pending = True
            self.nav_purpose = f"cancel_{reason}"
            self.nav_handle.cancel_goal_async()

    def _send_nav_goal(self, pose, purpose, guidance=None):
        if self.nav_handle is not None or self.nav_send_pending or not self.nav_client.server_is_ready():
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_send_pending, self.nav_purpose = True, purpose
        self.nav_target = (pose.pose.position.x, pose.pose.position.y)
        if purpose == "approach" and guidance is not None:
            self.nav_obstacle_position = self._obstacle_position(guidance)
        elif purpose != "approach":
            self.nav_obstacle_position = None
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        self.nav_send_pending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            # Action server 在 Nav2 lifecycle 激活之前已经可被发现，但会拒绝目标。这不是
            # 路径规划失败，不能污染 blocked_frontiers；短暂退避后原目标可重新选择。
            self.nav_target = None
            self.nav_obstacle_position = None
            self.nav_purpose = ""
            self.nav_retry_until = time.monotonic() + float(
                self.params["nav_rejection_retry_delay"]
            )
            if self.enabled:
                self.state = "EXPLORING"
            self._publish_state("Nav2 rejected goal")
            return
        self.nav_handle, self.nav_started = handle, time.monotonic()
        if not self.enabled:
            self._cancel_nav("stopped_before_accept")
        handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        result = future.result()
        status = int(result.status) if result is not None else 0
        purpose, target = self.nav_purpose, self.nav_target
        obstacle_position = self.nav_obstacle_position
        self.nav_handle, self.nav_target, self.nav_cancel_pending = None, None, False
        self.nav_obstacle_position = None
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        if purpose == "frontier" and not succeeded and target is not None:
            self.blocked_frontiers.append(target)
        if not succeeded:
            # ABORT/CANCEL 后给 costmap、行为树和地图更新留下恢复窗口。特别是刚越障时，
            # SLAM 地图边界可能比机器人落后一个发布周期，立即重发只会制造失败风暴。
            self.nav_retry_until = time.monotonic() + float(
                self.params["nav_failure_retry_delay"]
            )
        if (
            self.enabled
            and purpose == "approach"
            and obstacle_position is not None
        ):
            latest = self.guidance
            latest_position = (
                self._obstacle_position(latest)
                if latest is not None
                and time.monotonic() - self.guidance_received
                <= float(self.params["guidance_timeout"])
                and latest.perception_valid
                and latest.traversal_required
                and latest.confidence
                >= float(self.params["minimum_obstacle_confidence"])
                else None
            )
            same_entry = (
                latest_position is not None
                and hypot(
                    latest_position[0] - obstacle_position[0],
                    latest_position[1] - obstacle_position[1],
                ) <= float(self.params["handoff_fallback_spatial_tolerance"])
                and latest.distance
                <= float(self.params["handoff_fallback_max_distance"])
                and abs(latest.lateral_offset)
                <= float(self.params["handoff_fallback_max_lateral"])
            )
            if same_entry and nav_status_allows_guarded_handoff(status):
                now = time.monotonic()
                if not self.traverse_client.server_is_ready():
                    self._hold_for_traversal_controller(
                        latest, obstacle_position, now
                    )
                else:
                    self.pending_traverse = latest
                    self.pending_traverse_position = obstacle_position
                    self.pending_traverse_started = now
                    self.state = "HANDOFF"
                    self._publish_state(
                        "confirmed entry reached; guarded handoff fallback "
                        f"after Nav2 status={status}"
                    )
                return
            # 到达旧入口后若最新证据已经属于另一个障碍，释放目标锁再探索，不能把新障碍
            # 的相对目标误套到旧入口，也不能永久锁死。
            self.locked_obstacle_position = None
        if self.enabled:
            self.state = "EXPLORING"
        self._publish_state(f"Nav2 {purpose} finished with status={status}")

    def _start_traverse(self, guidance):
        if (
            self.traverse_handle is not None
            or self.traverse_send_pending
            or not self.traverse_client.server_is_ready()
        ):
            return
        goal = TraverseObstacle.Goal()
        goal.obstacle_type = action_obstacle_type(guidance)
        for field in ("confidence", "distance", "lateral_offset", "heading_error"):
            setattr(goal, field, getattr(guidance, field))
        self.state, self.traverse_started = "TRAVERSING", time.monotonic()
        self.traverse_send_pending = True
        self._publish_state(f"handoff obstacle type={guidance.obstacle_type}")
        future = self.traverse_client.send_goal_async(goal)
        future.add_done_callback(self._traverse_goal_response)

    def _hold_for_traversal_controller(self, target, position, now):
        """入口已到达但执行器未接入时保持原地，不让 Nav2把赛道障碍当作绕行物。

        Gazebo、真机 SDK 或未来运动控制器都只能通过同一个 Action 合同接入。任务管理器
        不猜测腿部动作，也不会因服务缺失继续选择障碍背后的前沿目标。
        """
        self.pending_traverse = target
        self.pending_traverse_position = position
        self.pending_traverse_started = now
        self.state = "WAITING_FOR_TRAVERSAL_CONTROLLER"
        self._cancel_nav("waiting_for_traversal_controller")
        if not self.controller_wait_reported:
            self.controller_wait_reported = True
            self._publish_state(
                "entry reached; waiting for /traverse_obstacle controller"
            )

    def _traverse_goal_response(self, future):
        self.traverse_send_pending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.pending_traverse = None
            self.pending_traverse_position = None
            self.state = "EXPLORING" if self.enabled else "STOPPED"
            self._publish_state("TraverseObstacle unavailable/rejected")
            return
        self.traverse_handle = handle
        if not self.enabled:
            self.traverse_cancel_pending = True
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self._traverse_result)

    def _traverse_result(self, future):
        wrapped = future.result()
        success = bool(wrapped and wrapped.result.success)
        completed_position = self.pending_traverse_position
        completed_type = (
            int(self.pending_traverse.obstacle_type)
            if self.pending_traverse is not None
            else TraversalGuidance.OBSTACLE_UNKNOWN
        )
        self.traverse_handle, self.pending_traverse = None, None
        self.traverse_cancel_pending = False
        self.pending_traverse_position = None
        if not self.enabled:
            self.state = "STOPPED"
            return
        if success and completed_position is not None:
            self.completed_obstacles.append(
                (completed_type, completed_position[0], completed_position[1])
            )
            self.locked_obstacle_position = None
            self.cooldown_until = time.monotonic() + float(self.params["post_traversal_cooldown"])
            self.state = "EXPLORING"
            self._publish_state(f"obstacle complete; total={len(self.completed_obstacles)}")
        else:
            self.locked_obstacle_position = None
            self.state = "RECOVERY"
            self._publish_state("traversal failed; resume exploration")

    def _tick(self):
        self.state_pub.publish(String(data=self.state))
        if not self.enabled:
            return
        now = time.monotonic()
        if self.nav_handle is not None and now - self.nav_started > float(self.params["goal_timeout"]):
            self._cancel_nav("timeout")
            self._publish_state("Nav2 goal timeout; cancelling")
            return
        if (
            self.traverse_handle is not None
            and not self.traverse_cancel_pending
            and now - self.traverse_started > float(self.params["traversal_timeout"])
        ):
            self.traverse_cancel_pending = True
            self.traverse_handle.cancel_goal_async()
            self._publish_state("traversal timeout; cancelling")
            return
        if self.map_msg is None or now - self.map_received > float(self.params["map_timeout"]):
            self.state = "WAITING_FOR_INPUTS"
            return
        robot = self._robot_pose()
        if robot is None:
            self.state = "WAITING_FOR_INPUTS"
            return
        if (
            world_to_cell(self.map_msg, robot[0], robot[1]) is None
            and distance_outside_grid(self.map_msg, robot[0], robot[1])
            > float(self.params["map_boundary_tolerance"])
        ):
            self.state = "WAITING_FOR_MAP"
            return
        if self.traverse_handle is not None or self.traverse_send_pending:
            return
        if now < self.nav_retry_until:
            return
        if self.pending_traverse is not None:
            if (
                self.state != "WAITING_FOR_TRAVERSAL_CONTROLLER"
                and now - self.pending_traverse_started
                > float(self.params["traversal_timeout"])
            ):
                self.pending_traverse = None
                self.pending_traverse_position = None
                self.state = "RECOVERY"
                self._publish_state("TraverseObstacle server unavailable/timeout")
                return
            if not self.traverse_client.server_is_ready():
                self.state = "WAITING_FOR_TRAVERSAL_CONTROLLER"
                return
            self.controller_wait_reported = False
            if self.nav_handle is None and not self.nav_send_pending:
                self._start_traverse(self.pending_traverse)
            return
        target = None if now < self.cooldown_until else self._fresh_target()
        if target is not None:
            if not self._matches_obstacle_lock(target):
                # 多个障碍同时进入宽视场时，保持已确认入口，不在两个 Nav2 目标之间来回
                # cancel。旧目标结束后会释放锁，再处理新目标。
                if self.nav_handle is not None or self.nav_send_pending:
                    return
                self.locked_obstacle_position = None
            if target.phase == TraversalGuidance.PHASE_READY and target.ready_for_handoff:
                obstacle_position = (
                    self.locked_obstacle_position or self._obstacle_position(target)
                )
                if not self.traverse_client.server_is_ready():
                    self._hold_for_traversal_controller(
                        target, obstacle_position, now
                    )
                else:
                    self.pending_traverse, self.state = target, "HANDOFF"
                    self.pending_traverse_position = obstacle_position
                    self.pending_traverse_started = now
                    self._cancel_nav("handoff")
                    self._publish_state("READY confirmed; cancelling Nav2 before handoff")
                return
            if target.phase in (TraversalGuidance.PHASE_APPROACH, TraversalGuidance.PHASE_ALIGN):
                approach = self._relative_approach_pose(target)
                if self.nav_handle is not None and self.nav_purpose == "frontier":
                    self._cancel_nav("approach")
                    return
                if approach is not None:
                    robot_to_goal = hypot(
                        approach.pose.position.x - robot[0],
                        approach.pose.position.y - robot[1],
                    )
                    if (
                        robot_to_goal
                        < float(self.params["minimum_approach_goal_distance"])
                        and self.nav_handle is not None
                        and self.nav_purpose == "approach"
                    ):
                        # 感知持续更新会让入口从“值得导航”缩短到 Nav2 容差以内；此时
                        # 取消正在运行的旧入口目标，不能继续等到 goal_timeout。
                        self._cancel_nav("approach_within_tolerance")
                        return
                if approach is not None and self.nav_handle is None and not self.nav_send_pending:
                    if self.locked_obstacle_position is None:
                        self.locked_obstacle_position = self._obstacle_position(target)
                    if robot_to_goal < float(self.params["minimum_approach_goal_distance"]):
                        # 已在入口容差内时不发一个必然被 Nav2 秒判成功的 8 cm 目标。
                        if not self.traverse_client.server_is_ready():
                            self._hold_for_traversal_controller(
                                target, self.locked_obstacle_position, now
                            )
                        else:
                            self.pending_traverse = target
                            self.pending_traverse_position = self.locked_obstacle_position
                            self.pending_traverse_started = now
                            self.state = "HANDOFF"
                            self._publish_state("entry already within approach tolerance")
                        return
                    self.state = "ALIGNING_OBSTACLE"
                    self._send_nav_goal(approach, "approach", target)
                return
        if self.nav_handle is not None or self.nav_send_pending:
            return
        candidates = extract_frontiers(self.map_msg, (robot[0], robot[1]),
            minimum_cells=int(self.params["frontier_minimum_cells"]),
            minimum_distance=float(self.params["frontier_minimum_distance"]),
            maximum_distance=float(self.params["frontier_maximum_distance"]))
        frontier = choose_frontier(candidates, self.blocked_frontiers,
                                   float(self.params["frontier_exclusion_radius"]))
        if frontier is None:
            self.empty_frontier_count += 1
            if self.empty_frontier_count >= int(self.params["empty_frontier_confirmations"]):
                self.enabled, self.state = False, "COMPLETED"
                self._publish_state(f"no reachable frontier; obstacles={len(self.completed_obstacles)}")
            return
        self.empty_frontier_count = 0
        yaw = atan2(frontier.y - robot[1], frontier.x - robot[0])
        self.state = "EXPLORING"
        self._send_nav_goal(self._make_pose(frontier.x, frontier.y, yaw), "frontier")
        self._publish_state(f"frontier cells={frontier.cells}, distance={frontier.distance:.2f} m")


def main(args=None):
    # 自主导航是独立进程，Ctrl-C/launch SIGTERM 必须先取消远端 Nav2 Action，不能让
    # rclpy 默认 handler 先关闭 context，否则任务节点退出后机器人仍可能执行旧目标。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = AutonomousMission()
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        if rclpy.ok():
            # 必须先锁速度再取消 Action。取消是跨进程异步 RPC，而停车布尔量可在一个
            # 速度门周期内生效，避免 Ctrl-C 后继续滑行到 cancel result 返回。
            node._publish_immediate_stop()
            node._cancel_nav("shutdown")
            if node.traverse_handle is not None and not node.traverse_cancel_pending:
                node.traverse_cancel_pending = True
                node.traverse_handle.cancel_goal_async()
            # 给 Action cancel 请求一个短暂 DDS 发送窗口；不等待远端控制器无限响应。
            deadline = time.monotonic() + 0.75
            while rclpy.ok() and time.monotonic() < deadline:
                node._publish_immediate_stop()
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

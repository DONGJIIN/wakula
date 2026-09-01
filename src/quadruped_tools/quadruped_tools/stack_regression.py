"""Gazebo—SLAM—Nav2 长时间联合回归与资源采样工具。

该工具只用于测试，不属于机器人运行时控制链。它通过仿真专用高优先级
``/cmd_vel_teleop`` 交替执行矩形异路闭合轨迹与“整圈旋转—前进—倒退—反向整圈旋转”，
持续监视 ``/map``、``/scan``、``/odom``、导航健康和正前方名称；随后可调用 Nav2 的
NavigateThroughPoses 与 NavigateToPose Action，验证多目标、障碍内不可达目标及恢复取消。

报告中的 ``closed_path_pose_consistency`` 只衡量命令轨迹结束后 map/odom 位姿是否回到
起点。Gazebo 近真值里程计本身就可能让该指标很好，因此它不证明 SLAM Toolbox 执行了
回环图优化；真正的 loop-closure 验收仍需带可控里程计漂移的 bag，并对关闭/开启回环做 A/B。

真机默认禁止运行运动阶段。只有显式 ``--allow-motion`` 才会发布速度，且命令话题默认是
Gazebo 专用接口；这避免诊断脚本被误当成真实机器狗运动控制器。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from quadruped_interfaces.msg import FusedObstacle, TerrainFeatures, TraversalGuidance
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class StreamStats:
    """一个 ROS 数据流的到达次数和最大墙钟间隔。"""

    monotonic: Callable[[], float] = field(default=time.monotonic, repr=False)
    count: int = 0
    max_gap_seconds: float = 0.0
    _last_time: Optional[float] = field(default=None, repr=False)

    def update(self) -> None:
        now = self.monotonic()
        if self._last_time is not None:
            self.max_gap_seconds = max(self.max_gap_seconds, now - self._last_time)
        self._last_time = now
        self.count += 1

    def current_age_seconds(self, now_seconds: Optional[float] = None) -> float:
        """Return tail silence at report time; missing/rewound time fails closed."""
        if self._last_time is None:
            return float("inf")
        now = self.monotonic() if now_seconds is None else float(now_seconds)
        age = now - self._last_time
        return age if math.isfinite(age) and age >= 0.0 else float("inf")

    def public(self, now_seconds: Optional[float] = None) -> dict:
        current_age = self.current_age_seconds(now_seconds)
        return {
            "count": self.count,
            "max_gap_seconds": round(self.max_gap_seconds, 4),
            "current_age_seconds": (
                None if not math.isfinite(current_age) else round(current_age, 4)
            ),
        }


CORE_PROCESS_EXECUTABLES = frozenset(
    {
        # SLAM、Nav2 服务器、生命周期与速度链；RViz/Gazebo 故意不计入算法预算。
        "async_slam_toolbox_node",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "bt_navigator",
        "lifecycle_manager",
        "nav2_readiness_monitor",
        "navigation_health_monitor",
        # 感知、融合、规划辅助以及真机占位模型进程。
        "robot_state_publisher",
        "terrain_analyzer",
        "vision_obstacle_detector",
        "perception_fusion",
        "terrain_safety_assessor",
        "traversal_guidance",
        "navigation_speed_gate",
    }
)


@dataclass(frozen=True)
class ProcessSample:
    """一次 ``/proc`` 读取形成的稳定进程身份和资源计数。"""

    pid: int
    start_ticks: int
    executable: str
    cpu_ticks: int
    rss_pages: int

    @property
    def identity(self) -> Tuple[int, int]:
        """PID 会被内核复用，必须结合启动 tick 才能识别真正的重启。"""
        return self.pid, self.start_ticks


def _matching_core_executable(raw_cmdline: bytes) -> Optional[str]:
    """Match the actual executable (or a Python console-script path), never arguments."""
    arguments = [token for token in raw_cmdline.split(b"\0") if token]
    if not arguments:
        return None
    executable = Path(arguments[0].decode(errors="ignore")).name
    if executable in CORE_PROCESS_EXECUTABLES:
        return executable

    # Linux shebang execution exposes ROS 2 Python entry points as
    # ``python3 /install/.../terrain_analyzer --ros-args ...``.  Only argv[1] is the
    # executable script; scanning later arguments would let an unrelated process such as
    # ``sleep velocity_smoother`` contaminate CPU/RSS totals.  /usr/bin/env replaces itself
    # with Python via exec, so the sampled argv has the same form.
    python_interpreter = (
        executable in {"python", "python3"}
        or executable.startswith("python3.")
    )
    if python_interpreter and len(arguments) >= 2:
        script = Path(arguments[1].decode(errors="ignore")).name
        if script in CORE_PROCESS_EXECUTABLES:
            return script
    return None


def _read_process_sample(entry: Path) -> Optional[ProcessSample]:
    """读取单个 Linux 进程；进程并发退出时返回 ``None`` 而不中断长测。"""
    try:
        pid = int(entry.name)
        executable = _matching_core_executable((entry / "cmdline").read_bytes())
        if executable is None:
            return None
        raw_stat = (entry / "stat").read_text()
        # comm 字段被括号包围且理论上允许空格，不能对整行直接 split 后使用固定下标。
        closing_parenthesis = raw_stat.rfind(")")
        if closing_parenthesis < 0:
            return None
        fields = raw_stat[closing_parenthesis + 1 :].split()
        # fields[0] 是原始字段 3(state)：utime=14、stime=15、starttime=22、rss=24。
        cpu_ticks = int(fields[11]) + int(fields[12])
        start_ticks = int(fields[19])
        rss_pages = max(0, int(fields[21]))
        return ProcessSample(pid, start_ticks, executable, cpu_ticks, rss_pages)
    except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
        return None


@dataclass
class ResourceStats:
    """核心进程合计资源峰值，并显式记录进程重启/退出。

    ``proc_root``、系统 tick 和墙钟均可注入，因此 CI 能构造确定性 ``/proc`` 快照验证
    白名单及 PID 重用，而不依赖测试机当时恰好运行哪些 ROS 节点。
    """

    proc_root: Path = field(default_factory=lambda: Path("/proc"))
    clock_ticks_per_second: float = field(
        default_factory=lambda: float(os.sysconf("SC_CLK_TCK"))
    )
    page_size_bytes: int = field(default_factory=lambda: int(os.sysconf("SC_PAGE_SIZE")))
    monotonic: Callable[[], float] = field(default=time.monotonic, repr=False)
    peak_rss_mib: float = 0.0
    peak_cpu_percent: float = 0.0
    peak_process_count: int = 0
    samples: int = 0
    pid_start_events: int = 0
    pid_exit_events: int = 0
    _previous_ticks: Dict[Tuple[int, int], int] = field(default_factory=dict, repr=False)
    _active_identities: set[Tuple[int, int]] = field(default_factory=set, repr=False)
    _observed_identities: set[Tuple[int, int]] = field(default_factory=set, repr=False)
    _matched_executables: set[str] = field(default_factory=set, repr=False)
    _previous_time: Optional[float] = field(default=None, repr=False)

    def sample(self) -> None:
        process_samples = []
        try:
            entries = tuple(self.proc_root.iterdir())
        except (FileNotFoundError, PermissionError):
            entries = ()
        for entry in entries:
            if entry.name.isdigit():
                process = _read_process_sample(entry)
                if process is not None:
                    process_samples.append(process)

        identities = {process.identity for process in process_samples}
        ticks = {process.identity: process.cpu_ticks for process in process_samples}
        if self.samples > 0:
            self.pid_start_events += len(identities - self._active_identities)
            self.pid_exit_events += len(self._active_identities - identities)

        total_rss_pages = sum(process.rss_pages for process in process_samples)
        now = self.monotonic()
        rss_mib = total_rss_pages * self.page_size_bytes / (1024.0 * 1024.0)
        self.peak_rss_mib = max(self.peak_rss_mib, rss_mib)
        self.peak_process_count = max(self.peak_process_count, len(identities))
        if self._previous_time is not None:
            elapsed = now - self._previous_time
            if elapsed > 0.0 and self.clock_ticks_per_second > 0.0:
                # 只对两个采样点都存在的同一进程身份求增量。若把所有 PID 的累计 tick
                # 直接相减，一个进程退出会制造负 CPU，新进程又会制造虚假尖峰。
                delta_ticks = sum(
                    max(0, current - self._previous_ticks[identity])
                    for identity, current in ticks.items()
                    if identity in self._previous_ticks
                )
                cpu = delta_ticks / self.clock_ticks_per_second / elapsed * 100.0
                self.peak_cpu_percent = max(self.peak_cpu_percent, max(0.0, cpu))
        self._previous_ticks = ticks
        self._active_identities = identities
        self._observed_identities.update(identities)
        self._matched_executables.update(
            process.executable for process in process_samples
        )
        self._previous_time = now
        self.samples += 1

    def public(self) -> dict:
        return {
            "peak_rss_mib": round(self.peak_rss_mib, 2),
            # 100% 表示约占满一个 CPU 核，不是整机百分比。
            "peak_cpu_percent_one_core": round(self.peak_cpu_percent, 1),
            "active_process_count": len(self._active_identities),
            "peak_process_count": self.peak_process_count,
            "observed_process_identities": len(self._observed_identities),
            "pid_start_events_after_baseline": self.pid_start_events,
            "pid_exit_events_after_baseline": self.pid_exit_events,
            "matched_executables": sorted(self._matched_executables),
            "samples": self.samples,
        }


@dataclass
class HeaderAgeStats:
    """统计 ROS Header 到回归节点接收时的端到端年龄。

    软预算只生成报告告警，不改变整次软件回归的 ``passed``。这样 x86 上的调度尖峰不会
    被误报为真机失败，同时仍能发现“频率正常但消息已经排队数秒”的隐蔽延迟。
    """

    soft_budget_seconds: float
    future_tolerance_seconds: float = 0.05
    ages: List[float] = field(default_factory=list, repr=False)
    invalid_stamp_count: int = 0

    def update(self, seconds: int, nanoseconds: int, now_seconds: float) -> None:
        stamp = float(seconds) + float(nanoseconds) * 1e-9
        if not all(
            math.isfinite(value)
            for value in (stamp, float(now_seconds), self.soft_budget_seconds)
        ) or stamp <= 0.0 or self.soft_budget_seconds <= 0.0:
            self.invalid_stamp_count += 1
            return
        age = float(now_seconds) - stamp
        if age < -max(0.0, float(self.future_tolerance_seconds)):
            self.invalid_stamp_count += 1
            return
        # 同一 ROS 时钟下极小负值只可能来自浮点换算，按零延迟记录。
        self.ages.append(max(0.0, age))

    @staticmethod
    def _percentile(values: Sequence[float], fraction: float) -> Optional[float]:
        if not values:
            return None
        ordered = sorted(values)
        index = max(0, math.ceil(float(fraction) * len(ordered)) - 1)
        return float(ordered[min(index, len(ordered) - 1)])

    def public(self) -> dict:
        p50 = self._percentile(self.ages, 0.50)
        p95 = self._percentile(self.ages, 0.95)
        maximum = max(self.ages) if self.ages else None
        over_budget = sum(age > self.soft_budget_seconds for age in self.ages)
        return {
            "sample_count": len(self.ages),
            "invalid_stamp_count": self.invalid_stamp_count,
            "p50_age_seconds": None if p50 is None else round(p50, 4),
            "p95_age_seconds": None if p95 is None else round(p95, 4),
            "max_age_seconds": None if maximum is None else round(maximum, 4),
            "soft_budget_seconds": round(self.soft_budget_seconds, 4),
            "samples_over_soft_budget": over_budget,
            "p95_within_soft_budget": (
                None if p95 is None else p95 <= self.soft_budget_seconds
            ),
        }


def health_measurement_passes(
    measurement_started: bool,
    true_samples: int,
    false_samples: int,
) -> bool:
    """Require continuous healthy evidence inside the actual motion/test window.

    Lifecycle startup normally emits several false samples before scan, odometry and TF are
    ready; those must not fail a later test.  Conversely, one startup ``true`` must never hide
    a frozen TF during motion, so the post-readiness window needs at least one fresh true and
    no false samples.
    """
    return bool(
        measurement_started
        and int(true_samples) > 0
        and int(false_samples) == 0
    )


@dataclass(frozen=True)
class MotionSegment:
    """一段可复现平面 Twist；速度单位 m/s、rad/s，时长单位 s。"""

    label: str
    linear_x: float
    angular_z: float
    duration: float


def mapping_cycle_segments(
    cycle_index: int,
    linear_speed: float,
    angular_speed: float,
) -> Tuple[str, Tuple[MotionSegment, ...]]:
    """按轮次交替生成矩形异路闭合与原有倒退/双向旋转轨迹。

    矩形四边的总平移时间与原有“前进 5 s + 倒退 5 s”相同，四个直角转弯合计一整圈，
    所以加入真实空间闭合路线后默认总时长反而略短。下一次矩形反向绕行，可覆盖左右转向
    不对称；偶数编号（从零开始）的矩形保证即使只跑一轮也包含异路回到起点。
    """
    linear = max(0.05, abs(float(linear_speed)))
    angular = max(0.05, abs(float(angular_speed)))
    if int(cycle_index) % 2 == 0:
        direction = 1.0 if (int(cycle_index) // 2) % 2 == 0 else -1.0
        side_duration = 2.5
        corner_duration = (math.pi / 2.0) / angular
        segments = []
        for side in range(4):
            segments.append(
                MotionSegment(f"rectangle_side_{side + 1}", linear, 0.0, side_duration)
            )
            segments.append(
                MotionSegment(
                    f"rectangle_corner_{side + 1}",
                    0.0,
                    direction * angular,
                    corner_duration,
                )
            )
        return "rectangle_closed_path", tuple(segments)

    full_turn_duration = 2.0 * math.pi / angular
    return (
        "bidirectional_rotation_and_reverse",
        (
            MotionSegment("counterclockwise_full_turn", 0.0, angular, full_turn_duration),
            MotionSegment("forward", linear, 0.0, 5.0),
            MotionSegment("reverse", -linear, 0.0, 5.0),
            MotionSegment("clockwise_full_turn", 0.0, -angular, full_turn_duration),
        ),
    )


def closed_path_consistency_report(
    cycles: Sequence[dict],
    expected_cycles: Optional[int] = None,
) -> Tuple[dict, bool]:
    """汇总 map/odom 起终误差，并返回现有软件回归所用的保守通过标志。

    函数名和 JSON 字段刻意不使用 ``loop_closure``：即使所有误差都很小，近真值 odom
    也可能是主要原因。这里不检查 SLAM 图节点、闭环约束或优化前后差值，因而必须在输出
    内永久保留限制声明，避免报告被脱离上下文引用成“已经验证回环”。
    """
    records = list(cycles)
    expected = len(records) if expected_cycles is None else max(0, int(expected_cycles))
    map_cycles = []
    odom_cycles = []
    missing_cycles = []
    invalid_cycles = []
    public_cycles = []
    for index, item in enumerate(records):
        cycle_number = int(item.get("cycle", index + 1))
        public_item = dict(item)
        map_fields = ("map_position_error_m", "map_yaw_error_rad")
        if not all(name in item for name in map_fields):
            missing_cycles.append(cycle_number)
        else:
            map_values = tuple(float(item[name]) for name in map_fields)
            if not all(math.isfinite(value) and value >= 0.0 for value in map_values):
                invalid_cycles.append(cycle_number)
                # Python's default JSON encoder emits non-standard NaN/Infinity tokens.
                # Replace them in the stored evidence while keeping the cycle explicitly
                # failed through ``invalid_cycles``.
                for name, value in zip(map_fields, map_values):
                    if not math.isfinite(value):
                        public_item[name] = None
            else:
                map_cycles.append(public_item)

        odom_fields = ("odom_position_error_m", "odom_yaw_error_rad")
        if all(name in item for name in odom_fields):
            odom_values = tuple(float(item[name]) for name in odom_fields)
            if all(math.isfinite(value) and value >= 0.0 for value in odom_values):
                odom_cycles.append(public_item)
            else:
                # Odom is diagnostic rather than the pass criterion, but its JSON must still
                # never present NaN/Inf as a meaningful measurement.
                for name, value in zip(odom_fields, odom_values):
                    if not math.isfinite(value):
                        public_item[name] = None
        public_cycles.append(public_item)

    recorded_numbers = {
        int(item.get("cycle", index + 1)) for index, item in enumerate(records)
    }
    missing_cycles.extend(
        cycle_number
        for cycle_number in range(1, expected + 1)
        if cycle_number not in recorded_numbers
    )
    missing_cycles = sorted(set(missing_cycles))
    invalid_cycles = sorted(set(invalid_cycles))
    max_map_position_error = max(
        (item["map_position_error_m"] for item in map_cycles), default=float("inf")
    )
    max_map_yaw_error = max(
        (item["map_yaw_error_rad"] for item in map_cycles), default=float("inf")
    )
    max_odom_position_error = max(
        (item["odom_position_error_m"] for item in odom_cycles), default=float("inf")
    )
    max_odom_yaw_error = max(
        (item["odom_yaw_error_rad"] for item in odom_cycles), default=float("inf")
    )
    report = {
        "cycles": public_cycles,
        "expected_cycles": expected,
        "completed_cycles": len(map_cycles),
        "missing_cycles": missing_cycles,
        "invalid_cycles": invalid_cycles,
        "map_max_position_error_m": (
            None if not map_cycles else max_map_position_error
        ),
        "map_max_yaw_error_rad": None if not map_cycles else max_map_yaw_error,
        "odom_max_position_error_m": (
            None if not odom_cycles else max_odom_position_error
        ),
        "odom_max_yaw_error_rad": None if not odom_cycles else max_odom_yaw_error,
        "proves_slam_toolbox_loop_closure_optimization": False,
        "limitation": (
            "Closed-path pose consistency is not loop-closure proof; run an "
            "odometry-drift bag with loop closing disabled/enabled for A/B evidence."
        ),
    }
    passed = bool(
        expected > 0
        and len(map_cycles) == expected
        and not missing_cycles
        and not invalid_cycles
        and max_map_position_error < 0.35
        and max_map_yaw_error < 0.50
    )
    return report, passed


def _yaw_from_quaternion(quaternion) -> float:
    """只提取平面偏航角；仿真测试载体不使用滚转和俯仰。"""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _angle_error(first: float, second: float) -> float:
    """返回两个偏航角的最小绝对差。"""
    return abs(math.atan2(math.sin(second - first), math.cos(second - first)))


class StackRegression(Node):
    """采集全栈健康指标，并在得到明确授权时执行仿真回归轨迹。"""

    def __init__(
        self,
        command_topic: str,
        pipeline_latency_budget: float = 0.35,
        use_sim_time: bool = True,
    ):
        # 本工具明确只允许驱动仓库自带 Gazebo 测试载体，因此默认与被测消息、TF 和
        # Action Pose 共用 /clock。否则系统墙钟减去仿真 Header 会产生数十亿秒假延迟，
        # 同时 Nav2 也会拒绝本节点用错误时钟生成的目标时间。离线纯系统时间排障可由
        # CLI 显式传 --no-use-sim-time，绝不依赖隐式环境猜测。
        super().__init__(
            "stack_regression",
            parameter_overrides=[Parameter("use_sim_time", value=bool(use_sim_time))],
        )
        self.command_pub = self.create_publisher(Twist, command_topic, 10)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.streams: Dict[str, StreamStats] = {
            name: StreamStats()
            for name in ("map", "scan", "odom", "front_name", "traversal_guidance")
        }
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, map_qos)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, "/navigation/healthy", self._health_callback, 10)
        self.create_subscription(String, "/perception/front_obstacle_name", self._name_callback, 10)
        self.create_subscription(
            TraversalGuidance,
            "/traversal/guidance",
            self._guidance_callback,
            10,
        )
        latency_budget = max(0.01, float(pipeline_latency_budget))
        self.pipeline_latency = {
            "terrain_features": HeaderAgeStats(latency_budget),
            "fused_obstacle": HeaderAgeStats(latency_budget),
        }
        # 两条消息均保留原始传感器 Header：与仅统计回调间隔相比，age 能直接暴露
        # TF 等待、点云计算或融合队列造成的历史帧积压。
        self.create_subscription(
            TerrainFeatures,
            "/terrain/features_stamped",
            self._terrain_features_callback,
            10,
        )
        self.create_subscription(
            FusedObstacle,
            "/perception/fused_obstacle",
            self._fused_obstacle_callback,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.through_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.latest_known_cells = 0
        self.maximum_known_cells = 0
        self.latest_odom_pose: Optional[Tuple[float, float, float]] = None
        self.health_true = 0
        self.health_false = 0
        self.latest_health = False
        self.health_measurement_started = False
        self.measurement_health_true = 0
        self.measurement_health_false = 0
        self.startup_health_true = 0
        self.startup_health_false = 0
        self.obstacle_names = set()
        self.guidance_phase_samples: Dict[str, int] = {}
        self.guidance_phase_transitions = 0
        self.guidance_ready_boundary_transitions = 0
        self.guidance_rapid_ready_boundary_transitions = 0
        self.guidance_contract_violations = 0
        self._last_guidance_phase: Optional[int] = None
        self._last_guidance_transition_time: Optional[float] = None
        self.resource = ResourceStats()
        self.closed_path_errors: List[dict] = []
        self.mapping_cycles_expected = 0
        self._last_resource_sample = 0.0

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.streams["map"].update()
        # -1 是未知；不复制整个数组，只在低频地图回调统计一次已观测面积。
        self.latest_known_cells = sum(value >= 0 for value in msg.data)
        self.maximum_known_cells = max(self.maximum_known_cells, self.latest_known_cells)

    def _scan_callback(self, _msg: LaserScan) -> None:
        self.streams["scan"].update()

    def _odom_callback(self, msg: Odometry) -> None:
        self.streams["odom"].update()
        values = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        # 报告只缓存数值完整的里程计位姿；坏帧仍会被导航健康节点独立报错，不能用
        # NaN 污染整轮闭合统计并让 JSON 产生非标准浮点字面量。
        if all(math.isfinite(value) for value in values):
            self.latest_odom_pose = (
                values[0],
                values[1],
                _yaw_from_quaternion(msg.pose.pose.orientation),
            )

    def _health_callback(self, msg: Bool) -> None:
        self.latest_health = bool(msg.data)
        if msg.data:
            self.health_true += 1
            if self.health_measurement_started:
                self.measurement_health_true += 1
        else:
            self.health_false += 1
            if self.health_measurement_started:
                self.measurement_health_false += 1

    def _begin_health_measurement(self) -> None:
        """Freeze startup counts and open a clean health window after readiness."""
        self.startup_health_true = self.health_true
        self.startup_health_false = self.health_false
        self.measurement_health_true = 0
        self.measurement_health_false = 0
        self.health_measurement_started = True

    def _name_callback(self, msg: String) -> None:
        self.streams["front_name"].update()
        if msg.data:
            self.obstacle_names.add(msg.data)

    def _record_pipeline_age(self, name: str, header) -> None:
        """使用节点 ROS 时钟记录一个保留源 Header 的流水线输出年龄。"""
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        self.pipeline_latency[name].update(
            header.stamp.sec,
            header.stamp.nanosec,
            now_seconds,
        )

    def _terrain_features_callback(self, msg: TerrainFeatures) -> None:
        """记录原始点云采样到几何特征输出的端到端年龄。"""
        self._record_pipeline_age("terrain_features", msg.header)

    def _fused_obstacle_callback(self, msg: FusedObstacle) -> None:
        """记录原始感知采样到相机/点云融合输出的端到端年龄。"""
        self._record_pipeline_age("fused_obstacle", msg.header)

    def _guidance_callback(self, msg: TraversalGuidance) -> None:
        """检查越障交接消息的持续性及最重要的安全不变量。

        ``ready_for_handoff`` 只有在感知有效、确认需要越障且阶段为 READY 时才允许为真。
        这条约束比“是否识别到某个固定障碍”更适合作为通用回归条件，因为机器人路线和
        比赛 world 坐标都可以独立更换。
        """
        self.streams["traversal_guidance"].update()
        phase_names = {
            TraversalGuidance.PHASE_INVALID: "INVALID",
            TraversalGuidance.PHASE_CLEAR: "CLEAR",
            TraversalGuidance.PHASE_APPROACH: "APPROACH",
            TraversalGuidance.PHASE_ALIGN: "ALIGN",
            TraversalGuidance.PHASE_READY: "READY",
        }
        name = phase_names.get(int(msg.phase), f"UNKNOWN_{int(msg.phase)}")
        self.guidance_phase_samples[name] = self.guidance_phase_samples.get(name, 0) + 1
        if msg.ready_for_handoff and (
            not msg.perception_valid
            or not msg.traversal_required
            or msg.phase != TraversalGuidance.PHASE_READY
        ):
            self.guidance_contract_violations += 1

        if self._last_guidance_phase is not None and msg.phase != self._last_guidance_phase:
            now = time.monotonic()
            self.guidance_phase_transitions += 1
            # 机器人主动旋转时 APPROACH/CLEAR 频繁切换通常只是视场依次扫过多个障碍，
            # 并非交接抖动。真正危险的是 ALIGN 与 READY 在阈值边缘快速往返，因此只对
            # 这一对状态单独计数，避免用错误指标“优化”掉正常环境变化。
            ready_pair = {
                int(self._last_guidance_phase),
                int(msg.phase),
            } == {
                int(TraversalGuidance.PHASE_ALIGN),
                int(TraversalGuidance.PHASE_READY),
            }
            if ready_pair:
                self.guidance_ready_boundary_transitions += 1
                if (
                    self._last_guidance_transition_time is not None
                    and now - self._last_guidance_transition_time < 0.35
                ):
                    self.guidance_rapid_ready_boundary_transitions += 1
                self._last_guidance_transition_time = now
        self._last_guidance_phase = int(msg.phase)

    def spin_for(self, seconds: float, command: Optional[Twist] = None) -> None:
        """在墙钟期限内处理回调，并以 20 Hz 重发速度防止仿真 mux 超时。"""
        deadline = time.monotonic() + max(0.0, seconds)
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if command is not None and now >= next_publish:
                self.command_pub.publish(command)
                next_publish = now + 0.05
            if now - self._last_resource_sample >= 1.0:
                self.resource.sample()
                self._last_resource_sample = now
            rclpy.spin_once(self, timeout_sec=0.03)

    def stop(self) -> None:
        """连续发送几帧零速度，保证 mux 在阶段切换处没有残余命令。"""
        self.spin_for(0.25, Twist())

    def pose(self) -> Optional[Tuple[float, float, float]]:
        """读取最新 map→base_link，用于回到同一点后的闭环一致性检查。"""
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            _yaw_from_quaternion(transform.transform.rotation),
        )

    def wait_ready(self, timeout: float) -> bool:
        """等待地图、传感器、越障引导、定位 TF 和健康状态全部出现。"""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin_for(0.1)
            if (
                all(
                    self.streams[name].count
                    for name in ("map", "scan", "odom", "traversal_guidance")
                )
                and self.pose() is not None
                # A historical true is insufficient: TF may have frozen while the other
                # readiness streams continued. Open the measurement window only on the
                # latest explicitly healthy sample.
                and self.latest_health
            ):
                # Startup false samples are expected while lifecycle nodes and TF become
                # ready.  Only health received after this exact boundary evaluates motion.
                self._begin_health_measurement()
                return True
        return False

    def run_mapping_cycles(self, cycles: int, linear_speed: float, angular_speed: float) -> None:
        """执行两类闭合轨迹，分别记录 map 与原始 odom 的起终一致性。"""
        # One invocation is one reportable suite.  Clearing here prevents a later manual
        # rerun from inheriting earlier good cycles and hiding missing TF in the new run.
        self.closed_path_errors.clear()
        self.mapping_cycles_expected = max(0, int(cycles))
        for index in range(self.mapping_cycles_expected):
            map_start = self.pose()
            odom_start = self.latest_odom_pose
            trajectory, segments = mapping_cycle_segments(
                index, linear_speed, angular_speed
            )
            for segment in segments:
                command = Twist()
                command.linear.x = segment.linear_x
                command.angular.z = segment.angular_z
                self.spin_for(segment.duration, command)
            self.stop()
            self.spin_for(1.0)
            map_end = self.pose()
            odom_end = self.latest_odom_pose
            result = {"cycle": index + 1, "trajectory": trajectory}
            if map_start is not None and map_end is not None:
                result.update(
                    map_position_error_m=round(
                        math.hypot(
                            map_end[0] - map_start[0],
                            map_end[1] - map_start[1],
                        ),
                        4,
                    ),
                    map_yaw_error_rad=round(
                        _angle_error(map_start[2], map_end[2]), 4
                    ),
                )
            if odom_start is not None and odom_end is not None:
                result.update(
                    odom_position_error_m=round(
                        math.hypot(
                            odom_end[0] - odom_start[0],
                            odom_end[1] - odom_start[1],
                        ),
                        4,
                    ),
                    odom_yaw_error_rad=round(
                        _angle_error(odom_start[2], odom_end[2]), 4
                    ),
                )
            self.closed_path_errors.append(result)

    def _relative_pose(self, origin, forward: float, left: float, yaw_delta: float) -> PoseStamped:
        """把相对初始机身的测试点转换成 map 坐标，避免依赖固定 SLAM 原点。"""
        x, y, yaw = origin
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x + math.cos(yaw) * forward - math.sin(yaw) * left
        pose.pose.position.y = y + math.sin(yaw) * forward + math.cos(yaw) * left
        target_yaw = yaw + yaw_delta
        pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        pose.pose.orientation.w = math.cos(target_yaw / 2.0)
        return pose

    def _wait_future(self, future, timeout: float):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self.spin_for(0.05)
        return future.result() if future.done() else None

    def run_nav2_scenarios(self, timeout: float) -> List[dict]:
        """验证多目标闭环以及指向高墙内部的不可达目标和恢复/取消路径。"""
        origin = self.pose()
        results: List[dict] = []
        if origin is None:
            return [{"scenario": "all", "result": "no_map_pose"}]
        if not self.through_client.wait_for_server(timeout_sec=10.0):
            return [{"scenario": "multi_goal", "result": "action_unavailable"}]

        # 三点折线路径同时覆盖前进、转弯和返回；相对距离保持在初始 360° 雷达已观测区。
        goal = NavigateThroughPoses.Goal()
        goal.poses = [
            self._relative_pose(origin, 0.70, 0.00, 0.0),
            self._relative_pose(origin, 0.70, 0.60, math.pi / 2.0),
            self._relative_pose(origin, 0.05, 0.05, math.pi),
        ]
        handle = self._wait_future(self.through_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            results.append({"scenario": "multi_goal", "result": "rejected"})
        else:
            result = self._wait_future(handle.get_result_async(), timeout)
            if result is None:
                cancel = self._wait_future(handle.cancel_goal_async(), 5.0)
                # 等旧 Action 真正结束后再提交下一目标，否则 bt_navigator 会以
                # “another navigator is processing”拒绝后续恢复场景，形成伪失败。
                self._wait_future(handle.get_result_async(), 5.0)
                results.append(
                    {
                        "scenario": "multi_goal",
                        "result": "timeout_cancelled",
                        "cancel_acknowledged": cancel is not None,
                    }
                )
            else:
                results.append(
                    {
                        "scenario": "multi_goal",
                        "result": "succeeded" if result.status == GoalStatus.STATUS_SUCCEEDED else f"status_{result.status}",
                    }
                )

        # 比赛参考布局中，出生点西南方的直角绕杆区柱间距为 1.0 m。下面的折线路径
        # 从两根柱之间穿过、绕过直角拐点后返回，覆盖“窄通道 + 连续转角 + 返航”。
        # 坐标相对本轮初始位姿表达，因此 SLAM 的 map 原点发生平移也不影响测试。
        narrow = NavigateThroughPoses.Goal()
        narrow.poses = [
            self._relative_pose(origin, -1.50, -0.80, -2.65),
            self._relative_pose(origin, -2.20, -1.35, math.pi),
            self._relative_pose(origin, -2.20, -0.55, math.pi / 2.0),
            self._relative_pose(origin, 0.05, 0.05, 0.0),
        ]
        handle = self._wait_future(self.through_client.send_goal_async(narrow), 10.0)
        if handle is None or not handle.accepted:
            results.append({"scenario": "narrow_pole_passage", "result": "rejected"})
        else:
            result = self._wait_future(handle.get_result_async(), timeout)
            if result is None:
                cancel = self._wait_future(handle.cancel_goal_async(), 5.0)
                self._wait_future(handle.get_result_async(), 5.0)
                results.append(
                    {
                        "scenario": "narrow_pole_passage",
                        "result": "timeout_cancelled",
                        "cancel_acknowledged": cancel is not None,
                    }
                )
            else:
                results.append(
                    {
                        "scenario": "narrow_pole_passage",
                        "result": (
                            "succeeded"
                            if result.status == GoalStatus.STATUS_SUCCEEDED
                            else f"status_{result.status}"
                        ),
                    }
                )

        if not self.pose_client.wait_for_server(timeout_sec=10.0):
            results.append({"scenario": "blocked_high_wall", "result": "action_unavailable"})
            return results
        # 参考 world 中高墙相对出生点约为前 1.95 m、右 1.15 m。目标位于占用体内部，
        # 正确结果是规划失败/恢复后失败，而不是穿墙成功。正式坐标变化时可只改测试参数。
        blocked = NavigateToPose.Goal()
        blocked.pose = self._relative_pose(origin, 1.95, -1.15, 0.0)
        handle = self._wait_future(self.pose_client.send_goal_async(blocked), 10.0)
        if handle is None or not handle.accepted:
            results.append({"scenario": "blocked_high_wall", "result": "rejected_as_expected"})
        else:
            result = self._wait_future(handle.get_result_async(), min(timeout, 45.0))
            if result is None:
                cancel = self._wait_future(handle.cancel_goal_async(), 5.0)
                results.append(
                    {
                        "scenario": "blocked_high_wall",
                        "result": "recovery_timeout_cancelled",
                        "cancel_acknowledged": cancel is not None,
                    }
                )
            else:
                results.append(
                    {
                        "scenario": "blocked_high_wall",
                        "result": "unsafe_success" if result.status == GoalStatus.STATUS_SUCCEEDED else "failed_as_expected",
                        "status": int(result.status),
                    }
                )
        self.stop()
        return results

    def report(self, nav2_results: Sequence[dict]) -> dict:
        """形成可存档报告；pass 只代表本次仿真软件回归，不代表真机验收。"""
        closed_path_report, mapping_ok = closed_path_consistency_report(
            self.closed_path_errors,
            self.mapping_cycles_expected,
        )
        # Reuse the existing inter-arrival limits for tail freshness.  max_gap alone cannot
        # notice a publisher that emitted one message and then died, because no later callback
        # arrives to close that final interval.
        stream_limits = {
            "map": 2.0,
            "scan": 1.0,
            "odom": 1.0,
            "traversal_guidance": 2.0,
        }
        stream_report_time = time.monotonic()
        stream_reports = {}
        for name, stats in self.streams.items():
            values = stats.public(stream_report_time)
            limit = stream_limits.get(name)
            if limit is not None:
                current_age = stats.current_age_seconds(stream_report_time)
                values["maximum_allowed_gap_seconds"] = limit
                values["passed"] = bool(
                    stats.count > 0
                    and stats.max_gap_seconds < limit
                    and current_age < limit
                )
            stream_reports[name] = values
        streams_ok = all(
            stream_reports[name]["passed"] for name in stream_limits
        )
        nav2_ok = all(
            item.get("result") in {
                "succeeded",
                "failed_as_expected",
                "rejected_as_expected",
                "recovery_timeout_cancelled",
            }
            for item in nav2_results
        ) if nav2_results else True
        latency_report = {
            name: stats.public() for name, stats in self.pipeline_latency.items()
        }
        latency_warnings = [
            name
            for name, stats in latency_report.items()
            if stats["p95_within_soft_budget"] is False
            or stats["invalid_stamp_count"] > 0
        ]
        measurement_health_ok = health_measurement_passes(
            self.health_measurement_started,
            self.measurement_health_true,
            self.measurement_health_false,
        )
        return {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "clock": {
                "use_sim_time": bool(self.get_parameter("use_sim_time").value),
            },
            "passed": bool(
                streams_ok
                and mapping_ok
                and nav2_ok
                and measurement_health_ok
                and self.guidance_contract_violations == 0
            ),
            "streams": stream_reports,
            "navigation_health": {
                "startup": {
                    "true_samples": self.startup_health_true,
                    "false_samples": self.startup_health_false,
                },
                "measurement": {
                    "started": self.health_measurement_started,
                    "true_samples": self.measurement_health_true,
                    "false_samples": self.measurement_health_false,
                    "passed": measurement_health_ok,
                },
                "total": {
                    "true_samples": self.health_true,
                    "false_samples": self.health_false,
                },
            },
            "map": {
                "latest_known_cells": self.latest_known_cells,
                "maximum_known_cells": self.maximum_known_cells,
            },
            "closed_path_pose_consistency": closed_path_report,
            "front_obstacle_names_seen": sorted(self.obstacle_names),
            "traversal_guidance": {
                "phase_samples": dict(sorted(self.guidance_phase_samples.items())),
                "phase_transitions": self.guidance_phase_transitions,
                "ready_boundary_transitions": self.guidance_ready_boundary_transitions,
                "rapid_ready_boundary_transitions_under_0_35s": (
                    self.guidance_rapid_ready_boundary_transitions
                ),
                "contract_violations": self.guidance_contract_violations,
            },
            "nav2": list(nav2_results),
            "resources": self.resource.public(),
            "pipeline_latency": latency_report,
            # 这是跨机器可比较的软告警，不纳入 passed；RK3588 的硬阈值必须在目标板上
            # 结合温升、RMW 和是否启动 RViz 单独验收。
            "pipeline_latency_soft_warnings": latency_warnings,
            "scope": "Gazebo software regression only; not a real-quadruped safety certification",
        }


def parse_args(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-motion", action="store_true", help="明确允许发布仿真测试速度")
    parser.add_argument(
        "--cycles",
        type=int,
        default=5,
        help="矩形异路闭合与双向旋转/倒退交替轮数，默认约 3 分钟",
    )
    parser.add_argument("--linear-speed", type=float, default=0.16)
    parser.add_argument("--angular-speed", type=float, default=0.45)
    parser.add_argument("--command-topic", default="/cmd_vel_teleop")
    parser.add_argument("--ready-timeout", type=float, default=45.0)
    parser.add_argument("--nav2-timeout", type=float, default=90.0)
    parser.add_argument(
        "--pipeline-latency-budget",
        type=float,
        default=0.35,
        help="Terrain/Fused Header 年龄的软预算秒数；超限写报告但不改变 passed",
    )
    parser.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="默认使用 Gazebo /clock；仅在系统时间数据源测试时传 --no-use-sim-time",
    )
    parser.add_argument("--skip-nav2", action="store_true")
    parser.add_argument("--report", default="reports/stack_regression.json")
    return parser.parse_args(argv)


def main(args=None):
    options = parse_args(args)
    if not options.allow_motion:
        raise SystemExit("Refusing to publish motion without --allow-motion")
    rclpy.init()
    node = StackRegression(
        options.command_topic,
        pipeline_latency_budget=max(0.01, options.pipeline_latency_budget),
        use_sim_time=options.use_sim_time,
    )
    try:
        if not node.wait_ready(max(1.0, options.ready_timeout)):
            raise RuntimeError("stack did not provide map/scan/odom/TF/healthy before timeout")
        node.run_mapping_cycles(
            max(1, options.cycles),
            min(0.25, max(0.05, abs(options.linear_speed))),
            min(0.65, max(0.10, abs(options.angular_speed))),
        )
        nav2_results = [] if options.skip_nav2 else node.run_nav2_scenarios(max(10.0, options.nav2_timeout))
        report = node.report(nav2_results)
        destination = Path(options.report).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if not report["passed"]:
            raise SystemExit(1)
    except KeyboardInterrupt:
        # 人工中止长测时仍先走 finally 的零速度与 ROS 清理，不打印误导性的 traceback。
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

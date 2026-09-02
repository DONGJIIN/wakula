#!/usr/bin/env python3
"""Gazebo 通用测试狗的 TraverseObstacle Action 适配器。

此节点只属于仿真包。通用狗没有腿部动力学，普通平面速度会被高墙/坑沿的碰撞体挡住；
因此它在任务层完成语义确认和入口对正后，通过 Gazebo 标准 SetEntityPose 服务一次性
传送到障碍出口，用来验证任务编排、账本和后续探索。传送不模拟接触、步态或中间轨迹；
真机绝不能启动它。运动团队应以同名 Action server 替换，核心导航节点无需修改。
"""

from collections import deque
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin
import json
from pathlib import Path
import signal
from threading import Lock
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from quadruped_interfaces.action import TraverseObstacle
from quadruped_interfaces.msg import FusedObstacle, NavigationSafety
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RosTime
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


# Controller-envelope defaults are kept together so the simulator and the mission
# configuration can be checked as one contract.  ENTRY_READY retains its deliberately
# tighter 0.10 m lateral bound below; only ENTRY_PREPARING uses the 0.35 m envelope.
DEFAULT_MAXIMUM_ENTRY_DISTANCE = 2.50
DEFAULT_MAXIMUM_LATERAL_OFFSET = 0.35
DEFAULT_MAXIMUM_ALIGNMENT_ERROR = 0.22
DEFAULT_READY_ENTRY_DISTANCE = 0.45
DEFAULT_READY_LATERAL_OFFSET = 0.10
DEFAULT_READY_ALIGNMENT_ERROR = 0.08
DEFAULT_PREPARATION_STANDOFF = 0.20
DEFAULT_PREPARATION_LINEAR_SPEED = 0.16
DEFAULT_PREPARATION_ANGULAR_SPEED = 0.35

# At the broad server envelope, a 2.5 m snapshot leaves up to 2.3 m of planar
# approach after the 0.20 m standoff.  2.33 m / 0.16 m/s is already about 14.6 s;
# a worst-direction quarter turn plus 0.22 rad final alignment adds about 5.1 s.
# Twenty-five seconds leaves bounded margin for the proportional final approach and
# odometry scheduling while remaining below the mission's 45 s traversal timeout.
DEFAULT_PREPARATION_TIMEOUT = 25.0
DEFAULT_ODOMETRY_HISTORY_DURATION = 2.0
DEFAULT_ODOMETRY_HISTORY_MAX_SAMPLES = 256
# Gazebo publishes the staged semantic hint at 10 Hz while its bridged odometry may arrive at
# roughly 5 Hz under full SLAM/Nav2 load.  A 0.15 s bound sat just below the observed 0.16 s phase
# offset and rejected an otherwise current, valid traversal snapshot, sending autonomy into
# repeated recovery turns.  0.25 s spans one 5 Hz odometry period while remaining well below the
# server's 0.75 s maximum snapshot age.  This parameter belongs only to the simulation adapter;
# real controllers must use their synchronized state estimator history.
DEFAULT_ODOMETRY_SNAPSHOT_MAX_GAP = 0.25

# A finish pose is a synchronization contract, not merely a TF cache lookup.  Once the
# simulator returns the model home, the localization transform must advance beyond both
# the pose-service commit and the first home odometry sample, and must still be current.
# One second matches the navigation health timeout while leaving several SLAM update
# periods at the shipped rates.  This remains local to the Gazebo-only benchmark helper.
BENCHMARK_FINISH_MAX_TF_AGE = 1.0
BENCHMARK_FINISH_FUTURE_TOLERANCE = 0.10

# This is the simulation controller's explicit capability table, not a perception
# compatibility table.  The mission may preserve a semantic name while its near-field
# classifier briefly reports a coarser type, but ``action_type_for_semantic`` must
# canonicalize the final Action Goal before this server accepts it.  The unresolved
# ``wooden_bridge_unknown`` label is intentionally absent because it is not actionable.
SIM_TRAVERSAL_CAPABILITIES = {
    "right_angle_poles": TraverseObstacle.Goal.OBSTACLE_POLE,
    "gravel_wood_pit": TraverseObstacle.Goal.OBSTACLE_PIT,
    "height_bar": TraverseObstacle.Goal.OBSTACLE_BAR,
    "main_slope": TraverseObstacle.Goal.OBSTACLE_SLOPE,
    "wooden_bridge_a": TraverseObstacle.Goal.OBSTACLE_STEP,
    "wooden_bridge_b": TraverseObstacle.Goal.OBSTACLE_STEP,
    "t_shaped_stairs": TraverseObstacle.Goal.OBSTACLE_STEP,
    "high_wall": TraverseObstacle.Goal.OBSTACLE_WALL,
}


# 三分钟仿真回归的观察位均以各障碍 ``layout_*`` frame 为原点，不是全局坐标。
# x/y 是测试狗相对障碍中心的安全站位，yaw 是它朝向障碍的局部航向。正式坐标发布后
# 只需在 world 集中布局区修改 frame pose；这里会自动叠加 frame 的平移和旋转。
# 若障碍自身几何尺寸改变，才需要重新检查对应局部站位，而不是修改核心算法。
BENCHMARK_OBSERVATION_POSES = {
    "right_angle_poles": (2.20, 0.50, pi),
    # The east side is dominated by the 150 mm orange guard rail and looks like a
    # generic step/pole.  This south-east oblique view was measured in the combined
    # run: it exposes both the 100 mm depression and the L-shaped inner rail.
    "gravel_wood_pit": (1.20, -0.45, pi / 2.0),
    "height_bar": (-1.60, 0.00, 0.0),
    "main_slope": (-2.50, 0.00, 0.0),
    # At -3.2 m the near platform hides most of the 14-degree approach plane.  The
    # measured -3.6 m view retains enough ramp cells for the portable classifier.
    "wooden_bridge_a": (-3.60, 0.00, 0.0),
    "wooden_bridge_b": (3.50, 0.00, pi),
    "t_shaped_stairs": (-2.20, 0.00, 0.0),
    "high_wall": (-1.80, 0.00, 0.0),
}

BENCHMARK_TASK_ORDER = tuple(BENCHMARK_OBSERVATION_POSES)

# Gazebo-only benchmark observations expressed through the *same public contracts*
# used by the real perception stack.  These values sit well inside (not exactly on)
# the conservative metric gates in autonomous_mission.py.  They do not contain world
# coordinates and they never assert success: the mission must still identify the
# semantic name, queue TraverseObstacle, observe real model displacement and pass its
# post-traversal stability window before updating the completion ledger.
#
# Keep this table local to the simulator.  Changing it must never be used to tune the
# real OpenCV/point-cloud classifiers or their production thresholds.
BENCHMARK_CONTRACTS = {
    "right_angle_poles": {
        "obstacle_type": NavigationSafety.OBSTACLE_POLE,
        "height": 0.55,
        "width": 0.12,
    },
    "gravel_wood_pit": {
        "obstacle_type": NavigationSafety.OBSTACLE_PIT,
        "height": 0.15,
        "pit_depth": 0.10,
        "roughness": 0.04,
        "width": 1.00,
    },
    "height_bar": {
        "obstacle_type": NavigationSafety.OBSTACLE_BAR,
        "height": 0.32,
        # V2.0 retains the published 300 mm ground-to-crossbar-bottom clearance.
        # This synthetic simulator observation must match both the SDF and the rule;
        # changing it to satisfy a classifier would invalidate the benchmark.
        "clearance_height": 0.30,
        "width": 1.00,
    },
    "main_slope": {
        "obstacle_type": NavigationSafety.OBSTACLE_CLEAR,
        # Official 2026 V2.0 main-ramp pitch.  Keep this simulator truth aligned
        # with the world geometry; production perception must still measure it.
        "slope_pitch": 11.3 * pi / 180.0,
        "roughness": 0.01,
        "width": 2.00,
    },
    "wooden_bridge_a": {
        "obstacle_type": NavigationSafety.OBSTACLE_CLEAR,
        "slope_pitch": 14.0 * pi / 180.0,
        "roughness": 0.01,
        "width": 1.00,
    },
    "wooden_bridge_b": {
        "obstacle_type": NavigationSafety.OBSTACLE_STEP,
        "height": 0.22,
        "roughness": 0.09,
        "width": 1.00,
    },
    "t_shaped_stairs": {
        "obstacle_type": NavigationSafety.OBSTACLE_STEP,
        "height": 0.36,
        "roughness": 0.04,
        "width": 1.00,
    },
    "high_wall": {
        "obstacle_type": NavigationSafety.OBSTACLE_WALL,
        "height": 0.30,
        "roughness": 0.02,
        "width": 1.00,
    },
}


def benchmark_fused_obstacle(obstacle_id):
    """Build one synthetic fused-sensor sample for a staged Gazebo model.

    Header stamps are assigned immediately before publication because this pure
    helper is also imported by unit tests without starting an rclpy context.
    """
    values = BENCHMARK_CONTRACTS.get(str(obstacle_id))
    if values is None:
        return None

    fused = FusedObstacle()
    fused.header.frame_id = "base_link"
    fused.obstacle_type = int(values["obstacle_type"])
    fused.confidence = 0.99
    fused.geometry_confirmed = True
    fused.vision_confirmed = True
    fused.obstacle_height = float(values.get("height", 0.0))
    fused.pit_depth = float(values.get("pit_depth", 0.0))
    fused.slope_pitch = float(values.get("slope_pitch", 0.0))
    fused.slope_roll = float(values.get("slope_roll", 0.0))
    fused.roughness = float(values.get("roughness", 0.0))
    fused.distance = 1.00
    fused.lateral_offset = 0.0
    fused.width = float(values.get("width", 1.0))
    fused.structure_heading = 0.0
    fused.structure_heading_confidence = 0.99
    fused.clearance_height = float(values.get("clearance_height", 0.0))
    fused.time_skew = 0.0
    fused.valid_points = 1000
    return fused


def load_layout_poses(world_path):
    """Read the eight centralized layout frames from an SDF world.

    The return value is deliberately plain Python so unit tests can verify the
    Gazebo-only route helper without starting ROS or importing any core algorithm.
    """
    path = Path(str(world_path)).expanduser()
    if not path.is_file():
        return {}
    try:
        world = ET.parse(path).getroot().find("world")
    except (ET.ParseError, OSError):
        return {}
    if world is None:
        return {}
    poses = {}
    for obstacle_id in BENCHMARK_TASK_ORDER:
        node = world.find(f"frame[@name='layout_{obstacle_id}']/pose")
        if node is None or not node.text:
            continue
        try:
            values = tuple(float(value) for value in node.text.split())
        except ValueError:
            continue
        if len(values) == 6:
            poses[obstacle_id] = values
    return poses


def benchmark_observation_pose(obstacle_id, layout_poses):
    """Transform one obstacle-local observation pose into Gazebo world coordinates."""
    obstacle_id = str(obstacle_id)
    layout = layout_poses.get(obstacle_id)
    local = BENCHMARK_OBSERVATION_POSES.get(obstacle_id)
    if layout is None or local is None:
        return None
    frame_x, frame_y, _z, _roll, _pitch, frame_yaw = layout
    local_x, local_y, local_yaw = local
    return (
        frame_x + cos(frame_yaw) * local_x - sin(frame_yaw) * local_y,
        frame_y + sin(frame_yaw) * local_x + cos(frame_yaw) * local_y,
        frame_yaw + local_yaw,
    )


def next_benchmark_target(completed_ids):
    """Return the first pending rule task, or ``__home__`` after all eight."""
    completed = {str(item) for item in completed_ids}
    for obstacle_id in BENCHMARK_TASK_ORDER:
        if obstacle_id not in completed:
            return obstacle_id
    return "__home__"


def benchmark_ledger_transition(previous_ids, completed_ids):
    """Return ``(changed, reset, next_target)`` for one mission-ledger sample.

    The Gazebo backend is intentionally allowed to outlive the independent autonomous
    launch.  A newly started mission first publishes an empty transient-local ledger;
    that empty set is therefore a real session reset, not a message to discard.  The
    simulator still never invents completion: it only chooses the next observation pose
    from the ledger supplied by the core mission.
    """
    previous = frozenset(str(item) for item in previous_ids)
    completed = frozenset(str(item) for item in completed_ids)
    if completed == previous:
        return False, False, ""
    return True, not previous.issubset(completed), next_benchmark_target(completed)


def yaw_from_odometry(msg: Odometry) -> float:
    """提取平面航向；仿真替身不依赖 TF，避免与 SLAM 的 map 修正形成闭环。"""
    q = msg.pose.pose.orientation
    return atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def traversal_landing_pose(
    start_x,
    start_y,
    start_yaw,
    distance,
    l_turn=0,
):
    """返回一次性传送的 world/odom 出口 ``(x, y, yaw)``。

    普通结构沿对正方向落到另一侧；L 形坑和直角绕杆把总位移分成互相垂直的两臂。
    本函数没有 progress 或中间位姿，因而不会在 Gazebo 中模拟穿过碰撞体的过程。
    它完全不含固定 world 坐标，起点仍来自实时里程计。
    """
    total_distance = max(0.0, float(distance))
    forward = total_distance
    lateral = 0.0
    heading_delta = 0.0
    if int(l_turn):
        # 48%/52% only defines the landing displacement of the two rule arms; no
        # intermediate corner pose is sent to Gazebo.
        first_leg = total_distance * 0.48
        forward = first_leg
        second_leg = total_distance - first_leg
        lateral = float(int(l_turn)) * second_leg
        heading_delta = float(int(l_turn)) * (pi / 2.0)
    x = start_x + cos(start_yaw) * forward - sin(start_yaw) * lateral
    y = start_y + sin(start_yaw) * forward + cos(start_yaw) * lateral
    return x, y, start_yaw + heading_delta


def _landing_distances(desired: float, minimum: float, step: float = 0.10):
    """Return longest-first teleport distances, always including the minimum."""
    desired = max(0.0, float(desired))
    minimum = max(0.0, min(desired, float(minimum)))
    values = []
    current = desired
    while current > minimum + 1e-9:
        values.append(current)
        current -= max(0.01, float(step))
    values.append(minimum)
    return tuple(values)


def choose_safe_l_traversal(
    start_x,
    start_y,
    requested_yaw,
    distance,
    half_length,
    half_width,
    margin,
    minimum_distance=None,
    maximum_adjustment=0.35,
):
    """选择不会越界的 L 形传送落点，返回 ``(航向, 转向, 位移)``。

    传送模式没有中间轨迹，所以只检查最终落点。优先小航向修正，并按左/右两种出口
    都尝试；算法任务层不知道这些仿真几何细节。
    """
    offsets = [0.0]
    maximum_steps = max(0, int(float(maximum_adjustment) / (pi / 36.0)))
    for step in range(1, maximum_steps + 1):
        angle = step * pi / 36.0
        offsets.extend((angle, -angle))
    for offset in offsets:
        candidate = float(requested_yaw) + offset
        for turn in (-1, 1):
            for landing_distance in _landing_distances(
                distance,
                distance if minimum_distance is None else minimum_distance,
            ):
                x, y, _yaw = traversal_landing_pose(
                    start_x, start_y, candidate, landing_distance, l_turn=turn
                )
                if pose_inside_arena(x, y, half_length, half_width, margin):
                    return candidate, turn, landing_distance
    return None


def pose_inside_arena(x, y, half_length, half_width, margin) -> bool:
    """检查仿真替身是否仍在 14 m × 6 m 规则场地的安全内缩区域。"""
    usable_x = max(0.1, float(half_length) - max(0.0, float(margin)))
    usable_y = max(0.1, float(half_width) - max(0.0, float(margin)))
    return abs(float(x)) <= usable_x and abs(float(y)) <= usable_y


def choose_safe_traversal_heading(
    start_x,
    start_y,
    requested_yaw,
    distance,
    half_length,
    half_width,
    margin,
    minimum_distance=None,
    maximum_adjustment=0.35,
):
    """选择仍在赛台内、偏转最小且尽量完整的直线落点。

    真正四足控制器会在 PREPARING 阶段结合足端和边界状态生成轨迹；通用 Gazebo 狗
    只能做位姿覆盖。若机器人从障碍外侧接近，沿当前法向走完整个规则长度可能越出
    14 m × 6 m 赛台。这里依次尝试小角度到大角度的入口调整，只使用当前位姿和赛台
    尺寸，不读取任何障碍坐标或固定路线。
    """
    # 5° 分辨率只允许小幅修正。若必须转 90° 才能留在场内，说明入口方向或障碍身份
    # 尚未可靠确认；仿真后端应返回失败让任务换视角，绝不能为了“完成”而横穿场地。
    offsets = [0.0]
    maximum_steps = max(0, int(float(maximum_adjustment) / (pi / 36.0)))
    for step in range(1, maximum_steps + 1):
        angle = step * pi / 36.0
        offsets.extend((angle, -angle))
    for offset in offsets:
        candidate = float(requested_yaw) + offset
        for landing_distance in _landing_distances(
            distance,
            distance if minimum_distance is None else minimum_distance,
        ):
            end_x = float(start_x) + cos(candidate) * landing_distance
            end_y = float(start_y) + sin(candidate) * landing_distance
            if pose_inside_arena(
                end_x, end_y, half_length, half_width, margin
            ):
                return candidate, landing_distance
    return None


def gazebo_pose_service(world_name, override=""):
    """Resolve the simulator pose service without hiding a world-name constant."""
    explicit = str(override).strip()
    if explicit:
        if not explicit.startswith("/") or any(
            character.isspace() for character in explicit
        ):
            raise ValueError("pose_service must be an absolute ROS service name")
        return explicit
    name = str(world_name).strip()
    if (
        not name
        or "/" in name
        or name in {".", ".."}
        or any(character.isspace() for character in name)
    ):
        raise ValueError("world_name must be one non-empty Gazebo name")
    return f"/world/{name}/set_pose"


def normalize_angle(angle):
    """Wrap one planar angle to [-pi, pi] without hiding its unit (radians)."""
    return atan2(sin(float(angle)), cos(float(angle)))


@dataclass(frozen=True)
class PlanarPoseSample:
    """One validated odometry pose in the same clock domain as an Action Header."""

    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class FrozenEntryTarget:
    """Obstacle-entry geometry resolved once at the Goal measurement timestamp."""

    stamp: float
    snapshot_x: float
    snapshot_y: float
    snapshot_yaw: float
    target_x: float
    target_y: float
    target_yaw: float
    remaining_distance: float


def header_stamp_error(header):
    """Validate ROS ``Time`` fields before converting them to floating seconds."""
    try:
        seconds = int(header.stamp.sec)
        nanoseconds = int(header.stamp.nanosec)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return "header.stamp fields must be integers"
    if seconds < 0:
        return "header.stamp.sec must be non-negative"
    if nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        return "header.stamp.nanosec must be in [0, 1000000000)"
    if seconds == 0 and nanoseconds == 0:
        return "header.stamp must be non-zero"
    return ""


def _header_stamp_seconds(header):
    """Convert a ROS Header stamp to seconds; callers still decide its clock domain."""
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1.0e-9


def planar_pose_sample_from_odometry(msg):
    """Return a finite stamped planar pose, or ``None`` for unusable odometry."""
    if header_stamp_error(msg.header):
        return None
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    values = (
        position.x,
        position.y,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not all(isfinite(float(value)) for value in values):
        return None
    quaternion_norm = sum(
        float(value) ** 2
        for value in (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
    )
    if quaternion_norm < 1.0e-12:
        return None
    return PlanarPoseSample(
        stamp=_header_stamp_seconds(msg.header),
        x=float(position.x),
        y=float(position.y),
        yaw=yaw_from_odometry(msg),
    )


def odometry_pose_at_stamp(history, stamp, maximum_gap):
    """Resolve a Goal-time pose by exact match, interpolation, or bounded nearest.

    Interpolation uses the shortest yaw arc.  A single nearest sample is acceptable
    only inside ``maximum_gap``; this supports normal sensor scheduling jitter without
    letting an old pose silently reinterpret a newer obstacle distance.
    """
    query = float(stamp)
    gap = max(0.0, float(maximum_gap))
    samples = tuple(history)
    if not isfinite(query) or not samples:
        return None
    samples = tuple(
        sample
        for sample in sorted(samples, key=lambda item: float(item.stamp))
        if all(
            isfinite(float(value))
            for value in (sample.stamp, sample.x, sample.y, sample.yaw)
        )
    )
    if not samples:
        return None

    before = None
    after = None
    for sample in samples:
        delta = float(sample.stamp) - query
        if abs(delta) <= 1.0e-9:
            return sample
        if delta < 0.0:
            before = sample
            continue
        after = sample
        break

    if before is not None and after is not None:
        before_gap = query - float(before.stamp)
        after_gap = float(after.stamp) - query
        if before_gap <= gap and after_gap <= gap:
            span = float(after.stamp) - float(before.stamp)
            if span <= 1.0e-12:
                return before
            ratio = before_gap / span
            return PlanarPoseSample(
                stamp=query,
                x=float(before.x) + ratio * (float(after.x) - float(before.x)),
                y=float(before.y) + ratio * (float(after.y) - float(before.y)),
                yaw=normalize_angle(
                    float(before.yaw)
                    + ratio * normalize_angle(float(after.yaw) - float(before.yaw))
                ),
            )

    nearest = min(samples, key=lambda item: abs(float(item.stamp) - query))
    if abs(float(nearest.stamp) - query) <= gap:
        return nearest
    return None


def benchmark_hint_odometry_is_ready(
    sample_sequence, minimum_sequence, sample, expected_pose, tolerance=0.25
):
    """Return whether odometry has observed the simulator's staged pose.

    The sequence requirement distinguishes a genuinely post-SetEntityPose sample from a cached
    pre-teleport pose.  Position *and* wrapped yaw are checked because a correct location with the
    previous heading would still make the synthetic body-frame obstacle snapshot inconsistent.
    """
    if expected_pose is None:
        return True
    if int(sample_sequence) <= int(minimum_sequence) or sample is None:
        return False
    return bool(
        hypot(
            float(sample.x) - float(expected_pose[0]),
            float(sample.y) - float(expected_pose[1]),
        )
        <= float(tolerance)
        and abs(normalize_angle(float(sample.yaw) - float(expected_pose[2])))
        <= float(tolerance)
    )


def frozen_entry_from_history(goal, history, maximum_gap, standoff):
    """Freeze the body-frame entry geometry against Goal-time odometry history."""
    stamp = _header_stamp_seconds(goal.header)
    pose = odometry_pose_at_stamp(history, stamp, maximum_gap)
    if pose is None:
        return None, "odometry history unavailable near Goal header.stamp"

    distance = max(0.0, float(goal.distance))
    remaining_distance = min(distance, max(0.0, float(standoff)))
    target_forward = distance - remaining_distance
    target_lateral = float(goal.lateral_offset)
    return (
        FrozenEntryTarget(
            stamp=stamp,
            snapshot_x=float(pose.x),
            snapshot_y=float(pose.y),
            snapshot_yaw=float(pose.yaw),
            target_x=(
                float(pose.x)
                + cos(float(pose.yaw)) * target_forward
                - sin(float(pose.yaw)) * target_lateral
            ),
            target_y=(
                float(pose.y)
                + sin(float(pose.yaw)) * target_forward
                + cos(float(pose.yaw)) * target_lateral
            ),
            target_yaw=normalize_angle(
                float(pose.yaw) + float(goal.heading_error)
            ),
            remaining_distance=remaining_distance,
        ),
        "",
    )


def validate_traversal_goal(
    goal,
    *,
    now_seconds,
    required_frame,
    maximum_snapshot_age,
    maximum_future_skew,
    maximum_entry_distance,
    maximum_lateral_offset,
    maximum_alignment_error,
    ready_entry_distance,
    ready_lateral_offset,
    ready_alignment_error,
):
    """Return an empty string for a safe atomic Goal, otherwise a rejection reason.

    Validation deliberately happens before Action acceptance: a real controller must
    never start a final approach and only later discover that one metric was NaN, stale,
    expressed in another frame, or copied from an unrelated sensor frame.  The broad
    100 m geometry ceiling catches corrupted units while remaining independent of the
    competition layout; tighter entry/alignment bounds are explicit server parameters.
    """
    stamp_reason = header_stamp_error(goal.header)
    if stamp_reason:
        return stamp_reason

    frame_id = str(goal.header.frame_id).strip()
    if not frame_id or frame_id != str(required_frame).strip():
        return f"header.frame_id must be {required_frame!r}"
    if any(character.isspace() for character in frame_id):
        return "header.frame_id contains whitespace"

    stamp_seconds = _header_stamp_seconds(goal.header)
    now_seconds = float(now_seconds)
    if not isfinite(stamp_seconds) or not isfinite(now_seconds):
        return "header.stamp and server time must be finite"
    age = now_seconds - stamp_seconds
    if age < -max(0.0, float(maximum_future_skew)):
        return f"measurement is {-age:.3f} s in the future"
    if age > max(0.0, float(maximum_snapshot_age)):
        return f"measurement is stale by {age:.3f} s"

    if int(goal.obstacle_type) not in (
        TraverseObstacle.Goal.OBSTACLE_STEP,
        TraverseObstacle.Goal.OBSTACLE_PIT,
        TraverseObstacle.Goal.OBSTACLE_WALL,
        TraverseObstacle.Goal.OBSTACLE_BAR,
        TraverseObstacle.Goal.OBSTACLE_POLE,
        TraverseObstacle.Goal.OBSTACLE_SLOPE,
    ):
        return "obstacle_type is not traversable"
    semantic_id = str(goal.obstacle_id).strip()
    canonical_type = SIM_TRAVERSAL_CAPABILITIES.get(semantic_id)
    if canonical_type is None:
        return "obstacle_id is unknown or not actionable by this controller"
    if int(goal.obstacle_type) != int(canonical_type):
        return (
            f"obstacle_id {semantic_id!r} requires canonical obstacle_type "
            f"{int(canonical_type)}"
        )
    if int(goal.entry_stage) not in (
        TraverseObstacle.Goal.ENTRY_READY,
        TraverseObstacle.Goal.ENTRY_PREPARING,
    ):
        return "entry_stage must be ENTRY_READY or ENTRY_PREPARING"

    finite_fields = (
        "confidence",
        "distance",
        "lateral_offset",
        "heading_error",
        "obstacle_height",
        "pit_depth",
        "slope_pitch",
        "slope_roll",
        "roughness",
        "width",
        "structure_heading",
        "structure_heading_confidence",
        "clearance_height",
    )
    for field in finite_fields:
        if not isfinite(float(getattr(goal, field))):
            return f"{field} is not finite"

    if not 0.0 < float(goal.confidence) <= 1.0:
        return "confidence must be in (0, 1]"
    if not 0.0 <= float(goal.structure_heading_confidence) <= 1.0:
        return "structure_heading_confidence must be in [0, 1]"
    if not 0.0 <= float(goal.distance) <= max(0.0, float(maximum_entry_distance)):
        return "distance is outside the controller entry envelope"
    if abs(float(goal.lateral_offset)) > max(0.0, float(maximum_lateral_offset)):
        return "lateral_offset is outside the controller entry envelope"
    if abs(float(goal.heading_error)) > max(0.0, float(maximum_alignment_error)):
        return "heading_error is outside the controller entry envelope"
    if abs(float(goal.slope_pitch)) > pi / 2.0:
        return "slope_pitch is outside [-pi/2, pi/2]"
    if abs(float(goal.slope_roll)) > pi / 2.0:
        return "slope_roll is outside [-pi/2, pi/2]"
    if abs(float(goal.structure_heading)) > pi:
        return "structure_heading is outside [-pi, pi]"
    for field in (
        "obstacle_height",
        "pit_depth",
        "roughness",
        "width",
        "clearance_height",
    ):
        value = float(getattr(goal, field))
        if value < 0.0 or value > 100.0:
            return f"{field} must be in [0, 100] m"
    if int(goal.valid_points) <= 0:
        return "valid_points must be greater than zero"

    if int(goal.entry_stage) == TraverseObstacle.Goal.ENTRY_READY:
        if float(goal.distance) > max(0.0, float(ready_entry_distance)):
            return "ENTRY_READY distance exceeds the lift-ready envelope"
        if abs(float(goal.lateral_offset)) > max(
            0.0, float(ready_lateral_offset)
        ):
            return "ENTRY_READY lateral offset exceeds the lift-ready envelope"
        if abs(float(goal.heading_error)) > max(
            0.0, float(ready_alignment_error)
        ):
            return "ENTRY_READY heading error exceeds the lift-ready envelope"
    return ""


class MonotonicFeedback:
    """Publish the canonical PREPARING -> TRAVERSING -> STABILIZING sequence.

    Progress is global Action progress in [0, 1], not progress local to each state.
    Rejecting regressions here makes simulator tests exercise the same feedback
    contract expected from the future whole-body controller.
    """

    _ORDER = {
        TraverseObstacle.Feedback.STATE_PREPARING: 1,
        TraverseObstacle.Feedback.STATE_TRAVERSING: 2,
        TraverseObstacle.Feedback.STATE_STABILIZING: 3,
    }

    def __init__(self, handle):
        self.handle = handle
        self.last_order = 0
        self.last_progress = 0.0

    def publish(self, state, progress, message):
        """Publish one protocol-valid sample or reject a state/progress regression."""
        state = int(state)
        progress = float(progress)
        order = self._ORDER.get(state, 0)
        if (
            order <= 0
            or order < self.last_order
            or not isfinite(progress)
            or progress < self.last_progress
            or progress < 0.0
            or progress > 1.0
        ):
            raise RuntimeError("TraverseObstacle feedback must be monotonic")
        feedback = TraverseObstacle.Feedback()
        feedback.state = state
        feedback.progress = progress
        feedback.message = str(message)
        self.handle.publish_feedback(feedback)
        self.last_order = order
        self.last_progress = progress


class SimTraverseObstacle(Node):
    def __init__(self):
        """Create the simulation-only Action boundary and optional benchmark helpers.

        The server accepts one atomic traversal snapshot at a time.  It owns only
        planar final preparation plus one irreversible Gazebo pose update; it never
        claims leg dynamics.  The transient-local software-stop input can reject a
        new Goal or poison an active Goal, but it is not a physical emergency stop.
        """
        super().__init__("sim_traverse_obstacle")
        self.declare_parameter("command_topic", "/cmd_vel_teleop")
        # 任务层通常要求约 7° 对正；停滞交接最多允许约 12.6°，由控制器在
        # PREPARING 中闭环修正。仿真替身只实现低速平面对正、没有腿部闭环，因此超过
        # 该范围直接拒绝，防止“看见障碍但没对准”也被伪造为成功。
        self.declare_parameter(
            "maximum_alignment_error", DEFAULT_MAXIMUM_ALIGNMENT_ERROR
        )
        # Action Goal 必须是同一时刻、同一车体坐标系下的原子快照。
        # 对真机，时效窗口应根据 rosbag 中感知端到 Action server 的 P99 延迟
        # 设置；窗口过小会频繁拒绝，过大则可能用移动前的障碍位姿执行。
        self.declare_parameter("goal_frame_id", "base_link")
        self.declare_parameter("maximum_snapshot_age", 0.75)
        self.declare_parameter("maximum_future_skew", 0.05)
        # PREPARING 是运动控制器可自行消除的最终入口包络；READY 是更小的
        # 起身/落足窗口。这些是仿真控制器验证值，核心感知阈值不得从此反向调整。
        self.declare_parameter(
            "maximum_entry_distance", DEFAULT_MAXIMUM_ENTRY_DISTANCE
        )
        self.declare_parameter(
            "maximum_lateral_offset", DEFAULT_MAXIMUM_LATERAL_OFFSET
        )
        self.declare_parameter("ready_entry_distance", DEFAULT_READY_ENTRY_DISTANCE)
        self.declare_parameter(
            "ready_lateral_offset", DEFAULT_READY_LATERAL_OFFSET
        )
        self.declare_parameter(
            "ready_alignment_error", DEFAULT_READY_ALIGNMENT_ERROR
        )
        # ENTRY_PREPARING 使用通用测试狗的平面速度做最后低速接近/对正。
        # 若超时或里程计长时不变，Action 失败，不允许继续传送到出口。
        self.declare_parameter(
            "preparation_standoff", DEFAULT_PREPARATION_STANDOFF
        )
        self.declare_parameter(
            "preparation_linear_speed", DEFAULT_PREPARATION_LINEAR_SPEED
        )
        self.declare_parameter(
            "preparation_angular_speed", DEFAULT_PREPARATION_ANGULAR_SPEED
        )
        self.declare_parameter("preparation_position_tolerance", 0.05)
        self.declare_parameter("preparation_heading_tolerance", 0.04)
        self.declare_parameter("preparation_timeout", DEFAULT_PREPARATION_TIMEOUT)
        self.declare_parameter("preparation_stall_timeout", 2.0)
        # SetEntityPose 成功后短暂等待里程计发布新位姿，让任务层的越过/稳定后验读取
        # 到出口位置。它是墙钟等待，不代表真实越障耗时。
        self.declare_parameter("teleport_settle_duration", 0.25)
        # 完整规则跨度会在侧向入口或边界附近把出口投影到场外。传送替身允许缩短，
        # 但至少要越过在线入口距离并额外前进 0.60 m，满足任务层独立的位移/入口平面
        # 后验；这不代表真实机器人可以跳过结构中段。
        self.declare_parameter("minimum_exit_clearance", 0.60)
        self.declare_parameter("model_name", "generic_quadruped")
        self.declare_parameter("world_name", "robocon_obstacle_field")
        # 非空 pose_service 只用于特殊桥接命名；正常情况由 world_name 唯一推导。
        self.declare_parameter("pose_service", "")
        self.declare_parameter("odometry_topic", "/odom")
        # Goal geometry is measured in base_link at header.stamp.  Keep enough odom
        # history to resolve that timestamp exactly/interpolated; current odometry is
        # reserved for closing the loop toward the resulting frozen world target.
        self.declare_parameter(
            "odometry_history_duration", DEFAULT_ODOMETRY_HISTORY_DURATION
        )
        self.declare_parameter(
            "odometry_history_max_samples", DEFAULT_ODOMETRY_HISTORY_MAX_SAMPLES
        )
        self.declare_parameter(
            "odometry_snapshot_max_gap", DEFAULT_ODOMETRY_SNAPSHOT_MAX_GAP
        )
        self.declare_parameter("emergency_stop_topic", "/teleop/emergency_stop")
        # 仅供 robocon_field_teleport.launch.py 的限时全场回归。一次障碍仍必须由核心
        # 感知确认、Nav2 对正、Action 成功和位移后验共同完成；本开关只在清单已经变化
        # 后把模型放到下一障碍的观察位，避免随机前沿探索占用三分钟验收预算。
        # 关闭后保留完全自然探索行为；真机和纯场地 launch 从不启动本节点。
        # Disabled in the reusable backend itself. Only the explicit timed launch
        # opts in, so sim_traversal_controller.launch.py remains a plain Action
        # adapter and cannot unexpectedly inject benchmark observations.
        self.declare_parameter("benchmark_staging_enabled", False)
        self.declare_parameter("benchmark_staging_delay", 0.0)
        # Gazebo RGB-D at the rule bridge-B boards does not reliably preserve every
        # 400 mm gap after rasterization.  In the three-minute *workflow* benchmark,
        # publish the staged model as a simulator-only, internally consistent standard
        # perception contract.  It never publishes Action success or task progress.
        # Disable this parameter for SLAM/OpenCV/point-cloud accuracy tests.
        self.declare_parameter("benchmark_semantic_hint_enabled", False)
        self.declare_parameter("benchmark_semantic_hint_settle", 0.25)
        self.declare_parameter("benchmark_world_path", "")
        self.declare_parameter("benchmark_home_x", -2.50)
        self.declare_parameter("benchmark_home_y", -0.20)
        self.declare_parameter("benchmark_home_yaw", pi)
        # 从 Action 触发点到障碍表面仍有 request.distance；跨越后不仅要让约 0.75 m 长的
        # 测试机身完全离开碰撞体，还要越过 Nav2 inflation layer。0.75 m 的旧值会让
        # base_link 落在高墙旁的 lethal cell 中，随后所有全局规划都会从非法起点失败。
        # 1.20 m 只属于无腿仿真替身；真机 Action server 必须用接触/里程计判断完成。
        self.declare_parameter("exit_clearance", 1.20)
        # 对长度已包含完整结构的坡、桥、坑和 T 台，只需让 0.75 m 测试机身离开出口；
        # 若仍叠加 1.20 m inflation 余量，会跨过出口附近尚未巡检的其他障碍。薄墙仍
        # 使用上面的 1.20 m，避免落点处于墙脚的 lethal inflation cell。
        self.declare_parameter("long_structure_exit_clearance", 0.75)
        # 木桥的 request.distance 已经是机身到入口前缘的距离，span 之后只需留下较短
        # 的可导航出口余量。若仍使用 0.75 m，B 桥参考布局会把 base_link 放到在线地图
        # 边缘，Nav2 对返航目标立即 REJECT。该值只修正无腿位姿替身的落点。
        self.declare_parameter("wooden_bridge_exit_clearance", 0.35)
        # 规则结构沿通过方向的最小长度。Action 仍使用实时入口距离/航向；这些长度只让
        # 无腿动力学的测试替身落到结构另一侧，不包含任何 world 坐标或固定任务顺序。
        # 三根杆的 L 形中心线纵向包络是 1.00 m；S 形绕行产生的额外曲线长度已经由
        # traversal_pose 的横移表达，不能再把 1.80 m 曲线长度当成直线位移，否则从
        # 西侧参考布局进入时会错误预测落点越界并立即拒绝 Action。
        self.declare_parameter("right_angle_poles_span", 1.00)
        self.declare_parameter("gravel_wood_pit_span", 2.00)
        self.declare_parameter("height_bar_span", 0.05)
        self.declare_parameter("high_wall_span", 0.05)
        self.declare_parameter("main_slope_span", 3.00)
        self.declare_parameter("wooden_bridge_a_span", 4.35)
        # B 桥从西侧入口平台前缘到东侧出口坡末端约 5.20 m；规则中的 5.70 m 是模型
        # 总体参考包络，不能在 request.distance 后再次完整相加，否则重复计算入口平台。
        self.declare_parameter("wooden_bridge_b_span", 5.20)
        self.declare_parameter("t_shaped_stairs_span", 2.80)
        # 仅为 SetEntityPose 仿真替身提供最后一道越界保护；核心任务管理器仍完全不读取
        # 这些尺寸。正式坐标或真机 Action server 都不会使用这里的边界参数。
        self.declare_parameter("arena_half_length", 7.0)
        self.declare_parameter("arena_half_width", 3.0)
        # 测试狗横向半宽约 0.30 m；物理越界检查留 0.35 m（机身 + 5 cm）。Nav2 的
        # 0.45 m inflation 是规划代价，不是机身实体，不能再次叠加到规则场地边界；
        # 否则紧邻北侧合法布置的 L 形坑不存在任何可通过轨迹。
        self.declare_parameter("arena_margin", 0.35)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        # A true stop sample permanently poisons the currently accepted Action even
        # if Start clears the topic before the execute loop observes it.  Therefore
        # clearing the software stop can only enable a newly submitted Goal.
        self.stop_state_lock = Lock()
        self.emergency_stop = False
        self.active_action_stop_latched = False
        self.busy = False
        self.active_entry_snapshot = None
        self.odom_history_lock = Lock()
        history_capacity = max(
            16,
            min(
                4096,
                int(self.get_parameter("odometry_history_max_samples").value),
            ),
        )
        self.odom_history = deque(maxlen=history_capacity)
        self.latest_odom = None
        self.odom_sample_sequence = 0
        # Action execution, odometry, services and stop callbacks share a re-entrant
        # group.  ENTRY_PREPARING blocks briefly in its closed-loop execute callback;
        # re-entrancy lets another executor thread deliver the /odom and Action cancel
        # callbacks it needs.  ``stop_state_lock`` still serializes goal admission and
        # the irreversible SetEntityPose safety boundary.
        self.sim_callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._odom_callback,
            10,
            callback_group=self.sim_callback_group,
        )
        # Timer -> asynchronous SetEntityPose must be re-entrant: with the default
        # mutually-exclusive group the timer blocks its own service response until
        # the 0.30 s wall-clock deadline, so benchmark staging silently retries forever.
        pose_service = gazebo_pose_service(
            self.get_parameter("world_name").value,
            self.get_parameter("pose_service").value,
        )
        self.pose_client = self.create_client(
            SetEntityPose,
            pose_service,
            callback_group=self.sim_callback_group,
        )
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self._emergency_stop_callback,
            stop_qos,
            # The Action execute callback sleeps while approaching and stabilizing.
            # A re-entrant group is required so the stop sample is still processed.
            callback_group=self.sim_callback_group,
        )
        self.layout_poses = load_layout_poses(
            self.get_parameter("benchmark_world_path").value
        )
        self.last_completed_ids = frozenset()
        self.pending_benchmark_target = ""
        self.pending_benchmark_deadline = 0.0
        # The bundled start pose already faces the first rule task.  Activate its
        # fused observation immediately; later targets still advance only when the
        # core mission's completion ledger grows.
        benchmark_enabled = bool(
            self.get_parameter("benchmark_staging_enabled").value
        )
        self.active_benchmark_target = (
            BENCHMARK_TASK_ORDER[0] if benchmark_enabled else ""
        )
        # Later targets are teleported to an observation station.  Do not publish their synthetic
        # semantic frame until odometry has actually observed that SetEntityPose commit; a wall
        # delay alone can expire while Gazebo's sensor update is late under full SLAM/Nav2 load.
        # Otherwise the mission freezes a perfectly fresh perception Header for which the Action
        # server has no same-pose odometry sample and enters an avoidable rotate/retry loop.
        self.benchmark_hint_min_odom_sequence = 0
        self.benchmark_hint_expected_pose = None
        self.benchmark_hint_ready_at = time.monotonic() + (
            max(
                0.0,
                float(self.get_parameter("benchmark_semantic_hint_settle").value),
            )
            if benchmark_enabled
            else 0.0
        )
        self.benchmark_fused_pub = self.create_publisher(
            FusedObstacle, "/perception/fused_obstacle", 10
        )
        # The timed benchmark physically stages the model back at the world start.
        # A SLAM map can jump after seven non-physical SetEntityPose operations, so
        # the original map-frame start captured by the core need not still describe
        # that same physical world point. Once final staging has settled, publish the
        # current localized pose through the existing optional finish contract. The
        # simulator owns this correction; it never writes the completion ledger.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        finish_qos = QoSProfile(depth=1)
        finish_qos.reliability = ReliabilityPolicy.RELIABLE
        finish_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.benchmark_finish_pub = self.create_publisher(
            PoseStamped, "/autonomy/finish_pose", finish_qos
        )
        self.benchmark_finish_pending_after = 0.0
        self.benchmark_finish_min_odom_sequence = 0
        self.benchmark_finish_min_tf_stamp = None
        self.benchmark_finish_commit_stamp = None
        self.benchmark_finish_home_odom_stamp = None
        self.benchmark_finish_expected_pose = None
        inventory_qos = QoSProfile(depth=1)
        inventory_qos.reliability = ReliabilityPolicy.RELIABLE
        inventory_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            "/autonomy/completed_obstacles",
            self._completed_obstacles_callback,
            inventory_qos,
            callback_group=self.sim_callback_group,
        )
        self.create_timer(
            0.10,
            self._benchmark_staging_tick,
            callback_group=self.sim_callback_group,
        )
        self.create_timer(
            # 10 Hz is intentionally above the three-frame mission confirmation rate
            # but low enough for its single-threaded executor to drain name, safety
            # and guidance callbacks in order instead of starving semantic votes.
            0.10,
            self._benchmark_semantic_hint_tick,
            callback_group=self.sim_callback_group,
        )
        self.shutdown_requested = False
        self.server = ActionServer(
            self,
            TraverseObstacle,
            "/traverse_obstacle",
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.sim_callback_group,
        )
        self.get_logger().warning(
            "SIMULATION ONLY TraverseObstacle adapter active; no leg dynamics"
        )
        if bool(self.get_parameter("benchmark_staging_enabled").value):
            if len(self.layout_poses) == len(BENCHMARK_TASK_ORDER):
                self.get_logger().warning(
                    "SIMULATION ONLY three-minute benchmark staging is enabled"
                )
            else:
                self.get_logger().error(
                    "benchmark staging disabled at runtime: world layout frames unavailable"
                )

    def _odom_callback(self, msg):
        """Store only finite stamped odometry in a bounded, single-clock history."""
        sample = planar_pose_sample_from_odometry(msg)
        if sample is None:
            return
        history_duration = max(
            0.25,
            float(self.get_parameter("odometry_history_duration").value),
        )
        with self.odom_history_lock:
            if self.odom_history and sample.stamp < self.odom_history[-1].stamp:
                # A simulation reset changes the clock epoch.  Never interpolate a
                # Goal across poses from before and after that reset.
                self.odom_history.clear()
            if self.odom_history and abs(
                sample.stamp - self.odom_history[-1].stamp
            ) <= 1.0e-9:
                self.odom_history[-1] = sample
            else:
                self.odom_history.append(sample)
            while (
                len(self.odom_history) > 1
                and sample.stamp - self.odom_history[0].stamp > history_duration
            ):
                self.odom_history.popleft()
            self.latest_odom = msg
            self.odom_sample_sequence += 1

    def _completed_obstacles_callback(self, msg):
        """Queue a new observation station only after the core ledger changes.

        The simulator never publishes or edits the ledger.  Consequently a bad
        recognition, failed Action, or failed traversal posterior remains pending and
        cannot be hidden by this benchmark helper.
        """
        if not bool(self.get_parameter("benchmark_staging_enabled").value):
            return
        try:
            payload = json.loads(str(msg.data))
            completed = frozenset(str(item) for item in payload.get("ids", []))
        except (json.JSONDecodeError, AttributeError, TypeError):
            return
        changed, reset, next_target = benchmark_ledger_transition(
            self.last_completed_ids, completed
        )
        if not changed:
            return
        self.last_completed_ids = completed
        self.active_benchmark_target = ""
        self.pending_benchmark_target = next_target
        self.pending_benchmark_deadline = time.monotonic() + max(
            0.0, float(self.get_parameter("benchmark_staging_delay").value)
        )
        if reset:
            # The independent mission launch has begun a new session.  Cancel every
            # finish-pose latch from the old run and actively restage the first pending
            # observation instead of skipping one completion after restart.
            self.benchmark_finish_pending_after = 0.0
            self.benchmark_finish_min_odom_sequence = 0
            self.benchmark_finish_min_tf_stamp = None
            self.benchmark_finish_commit_stamp = None
            self.benchmark_finish_home_odom_stamp = None
            self.benchmark_finish_expected_pose = None
            self.get_logger().warning(
                "SIMULATION ONLY benchmark ledger reset; starting a new session"
            )

    def _benchmark_staging_tick(self):
        """Move to the next sensor observation pose without claiming task success."""
        self._publish_benchmark_finish_if_ready()
        target = self.pending_benchmark_target
        if (
            not target
            or self.busy
            or time.monotonic() < self.pending_benchmark_deadline
            or len(self.layout_poses) != len(BENCHMARK_TASK_ORDER)
        ):
            return
        self.pending_benchmark_target = ""
        if target == "__home__":
            pose = (
                float(self.get_parameter("benchmark_home_x").value),
                float(self.get_parameter("benchmark_home_y").value),
                float(self.get_parameter("benchmark_home_yaw").value),
            )
        else:
            pose = benchmark_observation_pose(target, self.layout_poses)
        if pose is None or not pose_inside_arena(
            pose[0],
            pose[1],
            self.get_parameter("arena_half_length").value,
            self.get_parameter("arena_half_width").value,
            self.get_parameter("arena_margin").value,
        ):
            self.get_logger().error(
                f"benchmark observation pose invalid for {target}; leaving model unchanged"
            )
            return
        if not self.pose_client.wait_for_service(timeout_sec=0.10):
            self.pending_benchmark_target = target
            self.pending_benchmark_deadline = time.monotonic() + 0.50
            return
        with self.odom_history_lock:
            precommit_odom_sequence = self.odom_sample_sequence
        precommit_tf_stamp = None
        if target == "__home__":
            precommit_tf_stamp = self._latest_map_base_stamp()
        if not self._set_model_pose(*pose):
            self.pending_benchmark_target = target
            self.pending_benchmark_deadline = time.monotonic() + 0.50
            return
        # The service response is the earliest point at which the home pose commit is
        # known to have succeeded.  Even when no TF was available before the request,
        # this ROS-clock sample provides a non-optional lower bound for later TF data.
        # A transform already cached before/while the request was in flight must not be
        # published as the map coordinate of the newly committed home pose.
        commit_ros_stamp = self.get_clock().now().nanoseconds * 1.0e-9
        self._stop()
        self.active_benchmark_target = target if target != "__home__" else ""
        self.benchmark_hint_min_odom_sequence = precommit_odom_sequence
        self.benchmark_hint_expected_pose = pose if target != "__home__" else None
        self.benchmark_hint_ready_at = time.monotonic() + max(
            0.0,
            float(self.get_parameter("benchmark_semantic_hint_settle").value),
        )
        if target == "__home__":
            # Give odometry and SLAM one settling interval to observe SetEntityPose;
            # publication below additionally requires both streams to advance beyond
            # their pre-commit samples, so a cached pre-home TF cannot become finish.
            self.benchmark_finish_pending_after = self.benchmark_hint_ready_at
            self.benchmark_finish_min_odom_sequence = precommit_odom_sequence
            self.benchmark_finish_min_tf_stamp = precommit_tf_stamp
            self.benchmark_finish_commit_stamp = (
                commit_ros_stamp
                if isfinite(commit_ros_stamp) and commit_ros_stamp > 0.0
                else None
            )
            self.benchmark_finish_home_odom_stamp = None
            self.benchmark_finish_expected_pose = pose
        self.get_logger().warning(
            f"SIMULATION ONLY benchmark staged at {target}: "
            f"({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})"
        )

    def _latest_map_base_stamp(self):
        """Return the newest valid map→base_link TF stamp, or ``None``."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", RosTime()
            )
        except TransformException:
            return None
        if header_stamp_error(transform.header):
            return None
        stamp = _header_stamp_seconds(transform.header)
        return stamp if isfinite(stamp) and stamp > 0.0 else None

    def _publish_benchmark_finish_if_ready(self):
        """Bind Gazebo home to a map pose only after post-commit odom and TF arrive."""
        if (
            self.benchmark_finish_pending_after <= 0.0
            or time.monotonic() < self.benchmark_finish_pending_after
        ):
            return
        expected = self.benchmark_finish_expected_pose
        if expected is None:
            return
        with self.odom_history_lock:
            odom_sequence = self.odom_sample_sequence
            odom = self.latest_odom
        # Sequence, rather than only Header seconds, proves this callback ran after the
        # SetEntityPose response even if Gazebo publishes two samples at one sim tick.
        if odom is None or odom_sequence <= self.benchmark_finish_min_odom_sequence:
            return
        odom_sample = planar_pose_sample_from_odometry(odom)
        if odom_sample is None:
            return
        if (
            hypot(odom_sample.x - expected[0], odom_sample.y - expected[1]) > 0.25
            or abs(normalize_angle(odom_sample.yaw - expected[2])) > 0.35
        ):
            return
        # Latch the first validated post-commit home odometry stamp.  Updating this on
        # every timer tick could keep moving the lower bound ahead of a slower SLAM TF
        # forever; one immutable sample proves the odometry stream observed the commit.
        if self.benchmark_finish_home_odom_stamp is None:
            self.benchmark_finish_home_odom_stamp = float(odom_sample.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", RosTime()
            )
        except TransformException:
            return
        if header_stamp_error(transform.header):
            return
        transform_stamp = _header_stamp_seconds(transform.header)
        minimum_tf_candidates = tuple(
            stamp
            for stamp in (
                self.benchmark_finish_min_tf_stamp,
                self.benchmark_finish_commit_stamp,
                self.benchmark_finish_home_odom_stamp,
            )
            if stamp is not None and isfinite(stamp) and stamp > 0.0
        )
        minimum_tf_stamp = (
            max(minimum_tf_candidates) if minimum_tf_candidates else None
        )
        now_seconds = self.get_clock().now().nanoseconds * 1.0e-9
        tf_age = now_seconds - transform_stamp
        if (
            not isfinite(transform_stamp)
            or transform_stamp <= 0.0
            or minimum_tf_stamp is None
            or transform_stamp <= minimum_tf_stamp + 1.0e-9
            or not isfinite(now_seconds)
            or not (
                -BENCHMARK_FINISH_FUTURE_TOLERANCE
                <= tf_age
                <= BENCHMARK_FINISH_MAX_TF_AGE
            )
        ):
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        finish = PoseStamped()
        finish.header.stamp = self.get_clock().now().to_msg()
        finish.header.frame_id = "map"
        finish.pose.position.x = float(translation.x)
        finish.pose.position.y = float(translation.y)
        finish.pose.position.z = float(translation.z)
        finish.pose.orientation = rotation
        self.benchmark_finish_pub.publish(finish)
        self.benchmark_finish_pending_after = 0.0
        self.benchmark_finish_min_odom_sequence = 0
        self.benchmark_finish_min_tf_stamp = None
        self.benchmark_finish_commit_stamp = None
        self.benchmark_finish_home_odom_stamp = None
        self.benchmark_finish_expected_pose = None
        self.get_logger().warning(
            "SIMULATION ONLY finish contract synchronized after staging home"
        )

    def _benchmark_semantic_hint_tick(self):
        """Publish one coherent simulator observation at the fusion-layer input.

        The normal safety assessor and guidance node remain the only downstream
        publishers.  Raw-perception accuracy is tested with this feature disabled.
        """
        target = self.active_benchmark_target
        if (
            not target
            or not bool(
                self.get_parameter("benchmark_semantic_hint_enabled").value
            )
            or time.monotonic() < self.benchmark_hint_ready_at
        ):
            return
        expected = self.benchmark_hint_expected_pose
        if expected is not None:
            with self.odom_history_lock:
                odom_sequence = self.odom_sample_sequence
                odom_sample = (
                    planar_pose_sample_from_odometry(self.latest_odom)
                    if self.latest_odom is not None
                    else None
                )
            if not benchmark_hint_odometry_is_ready(
                odom_sequence,
                self.benchmark_hint_min_odom_sequence,
                odom_sample,
                expected,
            ):
                return
        fused = benchmark_fused_obstacle(target)
        if fused is None:
            return
        fused.header.stamp = self.get_clock().now().to_msg()
        self.benchmark_fused_pub.publish(fused)

    def _emergency_stop_callback(self, msg):
        """Latch a global simulation stop and poison any already accepted Action.

        ``false`` clears only the global admission gate.  It intentionally does not
        clear ``active_action_stop_latched``: once B interrupted an active Goal, that
        Goal must terminate without success and the client must submit a new snapshot.
        """
        requested = bool(msg.data)
        with self.stop_state_lock:
            self.emergency_stop = requested
            if requested and self.busy:
                self.active_action_stop_latched = True
        if requested:
            self._stop()

    def _software_stop_blocks_motion(self):
        """Return true when no velocity command or new pose update may be issued."""
        with self.stop_state_lock:
            return bool(self.emergency_stop or self.active_action_stop_latched)

    def _reserve_action_if_safe(self, entry_snapshot=None):
        """Atomically reserve one Action slot and its immutable entry snapshot."""
        with self.stop_state_lock:
            if self.emergency_stop:
                return "simulation software emergency stop is latched"
            if self.busy:
                return "another traversal is active"
            self.busy = True
            self.active_action_stop_latched = False
            self.active_entry_snapshot = entry_snapshot
        return ""

    def _release_action_reservation(self):
        """Release an Action slot without accidentally clearing the global stop."""
        with self.stop_state_lock:
            self.busy = False
            self.active_action_stop_latched = False
            self.active_entry_snapshot = None

    def goal_callback(self, goal):
        """Validate one immutable Goal, then reserve execution unless stopped/busy.

        Admission requires a supported semantic/canonical Action type and odometry close
        enough to ``header.stamp`` to freeze the entry in world coordinates.  Validation
        alone does not authorize movement: reservation and the software-stop check are
        atomic.  A later stop sample poisons this reservation and every execute phase
        rechecks it before issuing a command or SetEntityPose request.
        """
        reason = validate_traversal_goal(
            goal,
            now_seconds=self.get_clock().now().nanoseconds * 1.0e-9,
            required_frame=self.get_parameter("goal_frame_id").value,
            maximum_snapshot_age=self.get_parameter("maximum_snapshot_age").value,
            maximum_future_skew=self.get_parameter("maximum_future_skew").value,
            maximum_entry_distance=self.get_parameter("maximum_entry_distance").value,
            maximum_lateral_offset=self.get_parameter("maximum_lateral_offset").value,
            maximum_alignment_error=self.get_parameter("maximum_alignment_error").value,
            ready_entry_distance=self.get_parameter("ready_entry_distance").value,
            ready_lateral_offset=self.get_parameter("ready_lateral_offset").value,
            ready_alignment_error=self.get_parameter("ready_alignment_error").value,
        )
        entry_snapshot = None
        if not reason:
            with self.odom_history_lock:
                history = tuple(self.odom_history)
            entry_snapshot, reason = frozen_entry_from_history(
                goal,
                history,
                self.get_parameter("odometry_snapshot_max_gap").value,
                self.get_parameter("preparation_standoff").value,
            )
        if not reason:
            reason = self._reserve_action_if_safe(entry_snapshot)
        if reason:
            self.get_logger().warning(
                "rejecting TraverseObstacle goal: "
                f"{reason}; type={int(goal.obstacle_type)}, "
                f"entry_stage={int(goal.entry_stage)}, id={str(goal.obstacle_id)!r}"
            )
        return GoalResponse.REJECT if reason else GoalResponse.ACCEPT

    def _stop(self):
        # SIGINT 可能先使 rcl context 失效，再进入 finally；此时最后一帧零速度已经无法
        # 进入 DDS。主动检查可避免正常关闭被 Ubuntu 误报成节点崩溃。
        if rclpy.ok() and self.publisher is not None:
            try:
                self.publisher.publish(Twist())
            except Exception:  # rclpy Jazzy 暴露的底层 RCLError 未提供稳定公共导入路径。
                # ``rclpy.ok()`` 与 publish 之间仍可能收到 launch 的第二个关闭信号；
                # 这是正常退出竞争，不应把仿真适配器报告成崩溃。
                pass

    def _publish_preparation_command(self, linear_x, angular_z):
        """Publish one bounded planar command used only by ENTRY_PREPARING."""
        command = Twist()
        command.linear.x = float(linear_x)
        command.angular.z = float(angular_z)
        self.publisher.publish(command)

    def _prepare_entry(self, handle, feedback, entry_snapshot):
        """Complete the final low-speed approach/alignment before simulated traversal.

        ``entry_snapshot`` was already transformed with odometry at ``header.stamp``
        before Goal acceptance.  This routine uses live odometry only as feedback toward
        that frozen world target; execution latency can never translate ``distance`` a
        second time from a newer body pose.

        Returns ``(status, message, remaining_distance, remaining_heading_error)``.
        ``status`` is ``ok``, ``cancel``, or ``abort``.  A timeout/stall is fail-closed:
        no Gazebo pose service is called, so PREPARING can never silently become a lift.
        """
        start_odom = self.latest_odom
        if start_odom is None:
            return "abort", "simulation odometry unavailable", 0.0, 0.0
        position = start_odom.pose.pose.position
        start_x, start_y = float(position.x), float(position.y)
        start_yaw = yaw_from_odometry(start_odom)
        target_x = float(entry_snapshot.target_x)
        target_y = float(entry_snapshot.target_y)
        final_yaw = float(entry_snapshot.target_yaw)

        linear_limit = max(
            0.01, float(self.get_parameter("preparation_linear_speed").value)
        )
        angular_limit = max(
            0.01, float(self.get_parameter("preparation_angular_speed").value)
        )
        position_tolerance = max(
            0.01,
            float(self.get_parameter("preparation_position_tolerance").value),
        )
        heading_tolerance = max(
            0.005,
            float(self.get_parameter("preparation_heading_tolerance").value),
        )
        deadline = time.monotonic() + max(
            0.1, float(self.get_parameter("preparation_timeout").value)
        )
        stall_timeout = max(
            0.1, float(self.get_parameter("preparation_stall_timeout").value)
        )

        initial_position_error = hypot(target_x - start_x, target_y - start_y)
        initial_heading_error = abs(normalize_angle(final_yaw - start_yaw))
        initial_error = max(
            1.0e-6, initial_position_error + 0.25 * initial_heading_error
        )
        # Stall means the body pose does not change, not that Euclidean target error
        # decreases every cycle.  During a legitimate turn-in-place the final-yaw
        # error may temporarily grow while the robot first faces a lateral target.
        last_motion_x = start_x
        last_motion_y = start_y
        last_motion_yaw = start_yaw
        last_motion_time = time.monotonic()

        while rclpy.ok() and time.monotonic() < deadline:
            if self._software_stop_blocks_motion():
                self._stop()
                return (
                    "abort",
                    "simulation software emergency stop interrupted entry preparation",
                    0.0,
                    0.0,
                )
            if self.shutdown_requested:
                self._stop()
                return "abort", "simulation shutting down during entry preparation", 0.0, 0.0
            if handle.is_cancel_requested:
                self._stop()
                return "cancel", "entry preparation cancelled", 0.0, 0.0
            current_odom = self.latest_odom
            if current_odom is None:
                self._stop()
                return "abort", "simulation odometry lost during entry preparation", 0.0, 0.0

            current_position = current_odom.pose.pose.position
            current_x = float(current_position.x)
            current_y = float(current_position.y)
            current_yaw = yaw_from_odometry(current_odom)
            delta_x = target_x - current_x
            delta_y = target_y - current_y
            position_error = hypot(delta_x, delta_y)
            final_heading_error = normalize_angle(final_yaw - current_yaw)
            total_error = position_error + 0.25 * abs(final_heading_error)

            if (
                hypot(current_x - last_motion_x, current_y - last_motion_y) >= 0.005
                or abs(normalize_angle(current_yaw - last_motion_yaw)) >= 0.01
            ):
                last_motion_x = current_x
                last_motion_y = current_y
                last_motion_yaw = current_yaw
                last_motion_time = time.monotonic()
            if time.monotonic() - last_motion_time > stall_timeout:
                self._stop()
                return (
                    "abort",
                    "entry preparation stalled; request another observation angle",
                    0.0,
                    0.0,
                )

            completion = max(0.0, min(1.0, 1.0 - total_error / initial_error))
            feedback.publish(
                TraverseObstacle.Feedback.STATE_PREPARING,
                max(feedback.last_progress, 0.02 + 0.28 * completion),
                "simulation final low-speed entry approach/alignment",
            )

            if position_error > position_tolerance:
                path_heading = atan2(delta_y, delta_x)
                path_error = normalize_angle(path_heading - current_yaw)
                angular = max(
                    -angular_limit, min(angular_limit, 1.5 * path_error)
                )
                # Rotate first when the target lies outside the forward corridor.
                # This prevents a differential/planar test model from cutting the
                # obstacle corner while claiming that lateral alignment completed.
                linear = 0.0
                if abs(path_error) <= 0.25:
                    linear = min(linear_limit, max(0.03, 0.7 * position_error))
                    linear *= max(0.20, cos(path_error))
                self._publish_preparation_command(linear, angular)
            elif abs(final_heading_error) > heading_tolerance:
                angular = max(
                    -angular_limit,
                    min(angular_limit, 1.5 * final_heading_error),
                )
                self._publish_preparation_command(0.0, angular)
            else:
                self._stop()
                feedback.publish(
                    TraverseObstacle.Feedback.STATE_PREPARING,
                    max(feedback.last_progress, 0.30),
                    "simulation entry preparation complete",
                )
                return (
                    "ok",
                    "",
                    float(entry_snapshot.remaining_distance),
                    normalize_angle(final_yaw - current_yaw),
                )
            time.sleep(0.05)

        self._stop()
        return "abort", "entry preparation timed out", 0.0, 0.0

    def _set_model_pose(self, x, y, yaw) -> bool:
        """Dispatch one irreversible pose commit unless the software stop blocks it.

        The stop lock makes “latched before dispatch” and “arrived after dispatch” an
        explicit boundary.  Gazebo offers no transaction that can recall an already
        accepted SetEntityPose request, so a later stop terminates the Action without
        success but cannot pretend that the committed pose was rolled back.
        """
        request = SetEntityPose.Request()
        request.entity.name = str(self.get_parameter("model_name").value)
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = 0.0
        request.pose.orientation.z = sin(float(yaw) * 0.5)
        request.pose.orientation.w = cos(float(yaw) * 0.5)
        with self.stop_state_lock:
            if self.emergency_stop or self.active_action_stop_latched:
                return False
            future = self.pose_client.call_async(request)
        deadline = time.monotonic() + 0.30
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            return False
        try:
            response = future.result()
        except Exception:
            return False
        return bool(response and response.success)

    def execute(self, handle):
        """Run PREPARING, one pose commit, and STABILIZING for an accepted Goal.

        Goal distance/lateral/heading have already been frozen using Goal-time odometry;
        live odometry is feedback only and cannot move that target.  Every reversible
        phase observes cancellation, shutdown and the latched software stop.
        SetEntityPose is the single irreversible boundary; if a stop arrives after
        Gazebo accepted it, the Action still fails and documents that the pose cannot
        be rolled back.
        """
        result = TraverseObstacle.Result()
        feedback = MonotonicFeedback(handle)
        try:
            with self.stop_state_lock:
                entry_snapshot = self.active_entry_snapshot
            if entry_snapshot is None:
                self._stop()
                result.success = False
                result.message = "Goal-time odometry snapshot reservation is unavailable"
                handle.abort()
                return result
            if self._software_stop_blocks_motion():
                self._stop()
                result.success = False
                result.message = (
                    "simulation software emergency stop blocked traversal execution"
                )
                handle.abort()
                return result
            if self.latest_odom is None:
                result.success = False
                result.message = "simulation odometry unavailable"
                handle.abort()
                return result
            feedback.publish(
                TraverseObstacle.Feedback.STATE_PREPARING,
                0.0,
                "simulation validating atomic traversal snapshot",
            )
            if not self.pose_client.wait_for_service(timeout_sec=2.0):
                result.success = False
                result.message = "Gazebo set_pose service unavailable"
                handle.abort()
                return result

            entry_distance = float(entry_snapshot.remaining_distance)
            if int(handle.request.entry_stage) == TraverseObstacle.Goal.ENTRY_PREPARING:
                (
                    preparation_status,
                    preparation_message,
                    entry_distance,
                    _remaining_heading_error,
                ) = self._prepare_entry(handle, feedback, entry_snapshot)
                if preparation_status != "ok":
                    result.success = False
                    result.message = preparation_message
                    if preparation_status == "cancel":
                        handle.canceled()
                    else:
                        handle.abort()
                    return result
            else:
                # READY still performs preflight and publishes the canonical first
                # state; it merely skips planar approach because the snapshot already
                # proves that the body lies inside the lift-ready entry envelope.
                feedback.publish(
                    TraverseObstacle.Feedback.STATE_PREPARING,
                    0.30,
                    "simulation entry already READY; preflight complete",
                )

            # The teleport starts from the exact frozen standoff target, not from the
            # odometry pose at execute time.  Adding ``remaining_distance`` therefore
            # reaches the same obstacle entrance even if Action scheduling was delayed.
            start_x = float(entry_snapshot.target_x)
            start_y = float(entry_snapshot.target_y)
            start_yaw = float(entry_snapshot.target_yaw)
            semantic_span_parameter = {
                "right_angle_poles": "right_angle_poles_span",
                "gravel_wood_pit": "gravel_wood_pit_span",
                "height_bar": "height_bar_span",
                "high_wall": "high_wall_span",
                "main_slope": "main_slope_span",
                "wooden_bridge_a": "wooden_bridge_a_span",
                "wooden_bridge_b": "wooden_bridge_b_span",
                "t_shaped_stairs": "t_shaped_stairs_span",
            }.get(str(handle.request.obstacle_id))
            semantic_span = (
                max(0.0, float(self.get_parameter(semantic_span_parameter).value))
                if semantic_span_parameter
                else 0.0
            )
            exit_clearance = max(
                0.20, float(self.get_parameter("exit_clearance").value)
            )
            if str(handle.request.obstacle_id) in {
                "right_angle_poles",
                "gravel_wood_pit",
                "main_slope",
                "wooden_bridge_a",
                "wooden_bridge_b",
                "t_shaped_stairs",
            }:
                exit_clearance = max(
                    0.20,
                    float(
                        self.get_parameter("long_structure_exit_clearance").value
                    ),
                )
            if str(handle.request.obstacle_id) in {
                "wooden_bridge_a", "wooden_bridge_b"
            }:
                exit_clearance = max(
                    0.20,
                    float(
                        self.get_parameter("wooden_bridge_exit_clearance").value
                    ),
                )
            travel_distance = (
                entry_distance
                + semantic_span
                + exit_clearance
            )
            desired_travel_distance = travel_distance
            minimum_travel_distance = (
                entry_distance
                + max(
                    0.20,
                    float(self.get_parameter("minimum_exit_clearance").value),
                )
            )
            # ``target_yaw`` already includes the frozen Goal-time heading error.
            # Re-applying the same field against current odometry here would rotate the
            # entrance twice and violate the atomic-snapshot contract.
            requested_yaw = start_yaw
            l_turn = 0
            if str(handle.request.obstacle_id) in {
                "right_angle_poles", "gravel_wood_pit"
            }:
                # Both rule tasks are non-straight. The pit follows its L-shaped
                # arms; the three mandatory pole zones also form a right angle.
                # The old S curve returned to zero lateral offset and always left
                # the model beside the west boundary after the first task.
                safe_l_path = choose_safe_l_traversal(
                    start_x,
                    start_y,
                    requested_yaw,
                    travel_distance,
                    self.get_parameter("arena_half_length").value,
                    self.get_parameter("arena_half_width").value,
                    self.get_parameter("arena_margin").value,
                    minimum_distance=minimum_travel_distance,
                )
                if safe_l_path is None:
                    traversal_yaw = None
                else:
                    traversal_yaw, l_turn, travel_distance = safe_l_path
            else:
                safe_landing = choose_safe_traversal_heading(
                    start_x,
                    start_y,
                    requested_yaw,
                    travel_distance,
                    self.get_parameter("arena_half_length").value,
                    self.get_parameter("arena_half_width").value,
                    self.get_parameter("arena_margin").value,
                    minimum_distance=minimum_travel_distance,
                )
                if safe_landing is None:
                    traversal_yaw = None
                else:
                    traversal_yaw, travel_distance = safe_landing
            if traversal_yaw is None:
                self._stop()
                # Keep enough measured context in the rosout log to distinguish a
                # wrong semantic span from a side-on entry.  These are live odometry
                # and Action fields, not hidden world/model coordinates, so the test
                # remains representative of the real controller contract.
                projected_x = start_x + cos(requested_yaw) * travel_distance
                projected_y = start_y + sin(requested_yaw) * travel_distance
                obstacle_label = str(handle.request.obstacle_id) or "unclassified"
                self.get_logger().warning(
                    f"rejecting sim traversal {obstacle_label}: "
                    f"start=({start_x:.2f}, {start_y:.2f}, {start_yaw:.2f}), "
                    f"entry={entry_distance:.2f} m "
                    f"(snapshot={float(handle.request.distance):.2f} m), "
                    f"travel={travel_distance:.2f} m, requested_yaw={requested_yaw:.2f}, "
                    f"projected_end=({projected_x:.2f}, {projected_y:.2f})"
                )
                handle.abort()
                result.success = False
                result.message = (
                    "confirmed heading has no safe simulation landing; "
                    "request another observation angle"
                )
                return result
            self.get_logger().info(
                f"sim traversal {str(handle.request.obstacle_id) or 'unclassified'}: "
                f"start=({start_x:.2f}, {start_y:.2f}, {start_yaw:.2f}), "
                f"entry={entry_distance:.2f} m "
                f"(snapshot={float(handle.request.distance):.2f} m), "
                f"span={semantic_span:.2f} m, travel={travel_distance:.2f}/"
                f"{desired_travel_distance:.2f} m, "
                f"heading_adjust={traversal_yaw - start_yaw:.2f} rad, "
                f"l_turn={l_turn}"
            )
            if self._software_stop_blocks_motion():
                self._stop()
                handle.abort()
                result.success = False
                result.message = (
                    "simulation software emergency stop prevented pose commit"
                )
                return result
            if self.shutdown_requested:
                self._stop()
                handle.abort()
                result.success = False
                result.message = "simulation teleport stopped before pose change"
                return result
            if handle.is_cancel_requested:
                self._stop()
                handle.canceled()
                result.success = False
                result.message = "simulation teleport cancelled before pose change"
                return result
            feedback.publish(
                TraverseObstacle.Feedback.STATE_TRAVERSING,
                0.50,
                "simulation teleporting to obstacle exit",
            )
            # Only the landing pose matters: the bundled model has no leg dynamics and
            # is intentionally allowed to pass through collision geometry. This single
            # service call is the complete simulation traversal; there is no hidden
            # velocity command or intermediate pose sequence.
            final_x, final_y, final_yaw = traversal_landing_pose(
                start_x,
                start_y,
                traversal_yaw,
                travel_distance,
                l_turn=l_turn,
            )
            if not pose_inside_arena(
                final_x,
                final_y,
                self.get_parameter("arena_half_length").value,
                self.get_parameter("arena_half_width").value,
                self.get_parameter("arena_margin").value,
            ):
                handle.abort()
                result.success = False
                result.message = "final simulation pose outside competition arena"
                return result
            if not self._set_model_pose(final_x, final_y, final_yaw):
                handle.abort()
                result.success = False
                result.message = (
                    "simulation software emergency stop prevented pose commit"
                    if self._software_stop_blocks_motion()
                    else "final Gazebo pose update failed"
                )
                return result
            self._stop()
            feedback.publish(
                TraverseObstacle.Feedback.STATE_STABILIZING,
                0.75,
                "simulation waiting for landing odometry to settle",
            )
            if not rclpy.ok():
                result.success = False
                result.message = "simulation shutting down"
                return result
            settle = max(
                0.0,
                float(self.get_parameter("teleport_settle_duration").value),
            )
            deadline = time.monotonic() + settle
            while rclpy.ok() and time.monotonic() < deadline:
                self._stop()
                if self._software_stop_blocks_motion():
                    handle.abort()
                    result.success = False
                    result.message = (
                        "simulation software emergency stop interrupted stabilization; "
                        "an already committed Gazebo pose cannot be rolled back"
                    )
                    return result
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.success = False
                    result.message = "simulation traversal cancelled while stabilizing"
                    return result
                elapsed_fraction = (
                    1.0
                    if settle <= 0.0
                    else max(0.0, min(1.0, 1.0 - (deadline - time.monotonic()) / settle))
                )
                feedback.publish(
                    TraverseObstacle.Feedback.STATE_STABILIZING,
                    max(feedback.last_progress, 0.75 + 0.24 * elapsed_fraction),
                    "simulation landing stabilization in progress",
                )
                time.sleep(0.05)
            if self.shutdown_requested:
                handle.abort()
                result.success = False
                result.message = "simulation traversal stopped during shutdown"
                return result
            if self._software_stop_blocks_motion():
                handle.abort()
                result.success = False
                result.message = (
                    "simulation software emergency stop interrupted stabilization; "
                    "an already committed Gazebo pose cannot be rolled back"
                )
                return result
            feedback.publish(
                TraverseObstacle.Feedback.STATE_STABILIZING,
                1.0,
                "simulation landing stabilization complete",
            )
            # Serialize the terminal transition with the stop callback.  Whichever
            # acquires this lock first defines the boundary: a prior stop aborts this
            # Goal; a later stop applies to subsequent motion after success.
            with self.stop_state_lock:
                terminal_stop = bool(
                    self.emergency_stop or self.active_action_stop_latched
                )
                if not terminal_stop:
                    handle.succeed()
            if terminal_stop:
                handle.abort()
                result.success = False
                result.message = (
                    "simulation software emergency stop arrived at completion; "
                    "an already committed Gazebo pose cannot be rolled back"
                )
                return result
            result.success = True
            result.message = "simulation teleport reached obstacle exit"
            return result
        finally:
            self._stop()
            self._release_action_reservation()


def main(args=None):
    """Run the simulation Action server with bounded, stop-aware SIGINT teardown."""
    # launch 向整组进程发送 SIGINT 时，默认 rclpy handler 会先销毁 Action publisher，
    # execute callback 随后调用 abort/canceled 就产生 Ubuntu 崩溃弹窗。保持 context 存活，
    # 先让活动 goal 在 executor 内完成取消状态，再统一销毁节点。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimTraverseObstacle()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    stopping = False
    shutdown_deadline = 0.0

    def request_stop(_signum, _frame):
        nonlocal stopping, shutdown_deadline
        stopping = True
        node.shutdown_requested = True
        shutdown_deadline = time.monotonic() + 3.0

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        # Jazzy 的 MultiThreadedExecutor 在 Action 第一次完成后，某些 RMW 组合会让
        # ``spin()`` 的主线程持续命中已就绪 waitable，空闲时占满一个 CPU 核。这里给
        # 调度循环一个明确的 20 ms 让步：100 Hz 传感器由 Gazebo 独立发布、一次性位姿
        # 服务仍有足够余量，
        # 但健康监控、SLAM 和 Nav2 不会再被仿真替身饿死。仍保留两个 executor 线程，
        # 因为 Action 执行期间必须由另一线程接收 SetEntityPose 的异步服务响应。
        while rclpy.ok() and (
            not stopping or (node.busy and time.monotonic() < shutdown_deadline)
        ):
            executor.spin_once(timeout_sec=0.05)
            time.sleep(0.020)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

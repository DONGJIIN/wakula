"""陌生场地前沿探索、越障入口对正与 Action 交接任务管理器。

节点只编排三个已有能力：从 ``/map`` 选择未知区域边界、用 Nav2 到达自由空间目标、在
``/traversal/guidance`` 连续确认 READY 后调用 ``TraverseObstacle``。它不读取 Gazebo world
坐标，也不生成关节命令；仿真和真机通过同一个 Action 合同替换越障执行器。

真机探索、5 秒停滞恢复、入口交接和成功后验参数统一位于
``config/autonomous_mission.yaml``，文件顶部给出按运行现象排查和调整的顺序。Python 中的
纯函数保持硬件无关；不得将某个 world 的障碍坐标、顺序或仿真实体名写入任务判断。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
from math import atan2, ceil, cos, degrees, floor, hypot, isfinite, pi, sin
import signal
import time
from typing import Dict, List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from quadruped_interfaces.action import TraverseObstacle
from quadruped_interfaces.msg import NavigationSafety, TraversalGuidance
from quadruped_planning.parameter_validation import validate_mission_parameters
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


# 规则 V1.0 的八项障碍。这里保存的是硬件无关语义 ID，不包含 Gazebo 模型名、world
# 坐标或固定顺序；正式赛场改变布局后任务状态机仍只依赖在线感知结果。
COMPETITION_OBSTACLE_IDS = (
    "right_angle_poles",
    "gravel_wood_pit",
    "height_bar",
    "main_slope",
    "wooden_bridge_a",
    "wooden_bridge_b",
    "t_shaped_stairs",
    "high_wall",
)

# If a long structure is rejected, changing yaw at exactly the same station cannot
# change its exit envelope.  A genuinely new observation station is required before
# another Action; short wall/bar/pole entries may be corrected by heading alone.
LONG_TRAVERSAL_IDS = {
    "gravel_wood_pit", "main_slope", "wooden_bridge_a", "wooden_bridge_b",
    "t_shaped_stairs",
}


def navigation_purpose_allows_yaw_only_recovery(purpose: str) -> bool:
    """Return whether a live Nav2 goal may rotate while forward motion is stopped.

    This permission never forwards linear velocity: :mod:`cmd_vel_gate` reduces the
    command to bounded ``angular.z`` and still requires fresh scan, terrain and Nav2
    health heartbeats.  ``approach`` is intentionally included.  A competition
    obstacle often drives the terrain speed limit to zero just before DWB has removed
    the last few degrees of heading error; denying yaw there creates a deterministic
    STOP -> cannot align -> five-second cancellation loop.

    Handoff and traversal are absent because Nav2 no longer owns motion in those
    states.  ``entry_escape`` is absent because it is a deliberate translation and
    must pass the ordinary terrain speed gate.
    """
    return str(purpose) in {
        "return_home",
        "frontier",
        "coverage",
        "revisit_obstacle",
        "approach",
        "prealign_obstacle",
        "verify_obstacle",
        "entry_recovery",
        "search_turn",
    }

# 机器接口始终使用上面的稳定英文 ID；中文仅用于终端和任务清单，不能参与控制判断。
# 这样其他队员或上位机可以可靠解析 ID，同时现场人员无需对照枚举表。
COMPETITION_OBSTACLE_NAMES_ZH = {
    "right_angle_poles": "直角绕杆区",
    "gravel_wood_pit": "砂砾与碎木坑",
    "height_bar": "限高杆",
    "main_slope": "主斜坡",
    "wooden_bridge_a": "木桥 A",
    "wooden_bridge_b": "木桥 B",
    "t_shaped_stairs": "T 字形台阶",
    "high_wall": "高墙",
}


def mission_inventory(
    expected_ids: Sequence[str], completed_ids: Sequence[str]
) -> Tuple[Tuple[str, ...], Tuple[str, ...]]:
    """Return deterministic completed/pending lists in configured task order.

    A set alone is sufficient for scoring but unsuitable for a dashboard: its order
    changes between processes and duplicates can leak into display strings.  The
    configured order is therefore the single presentation order, while unknown
    diagnostic markers (for example an unresolved wooden bridge) remain internal.
    """
    expected = tuple(dict.fromkeys(str(item) for item in expected_ids if str(item)))
    completed_set = set(str(item) for item in completed_ids)
    completed = tuple(item for item in expected if item in completed_set)
    pending = tuple(item for item in expected if item not in completed_set)
    return completed, pending


def inventory_message(ids: Sequence[str]) -> str:
    """Serialize one obstacle list as UTF-8 JSON for humans and programs.

    ``std_msgs/String`` keeps this optional reporting interface lightweight.  JSON is
    deliberately used instead of an ad-hoc comma-separated sentence so a future UI,
    referee bridge or logger can consume it without depending on Chinese punctuation.
    """
    stable_ids = [str(item) for item in ids]
    return json.dumps(
        {
            "count": len(stable_ids),
            "ids": stable_ids,
            "names_zh": [
                COMPETITION_OBSTACLE_NAMES_ZH.get(item, item)
                for item in stable_ids
            ],
        },
        ensure_ascii=False,
        separators=(",", ":"),
    )


def inventory_display(completed: Sequence[str], pending: Sequence[str]) -> str:
    """Return one concise Chinese terminal line for the competition task ledger."""
    completed_names = [
        COMPETITION_OBSTACLE_NAMES_ZH.get(str(item), str(item)) for item in completed
    ]
    pending_names = [
        COMPETITION_OBSTACLE_NAMES_ZH.get(str(item), str(item)) for item in pending
    ]
    total = len(completed_names) + len(pending_names)
    return (
        f"[任务清单] 已越过({len(completed_names)}/{total}): "
        f"{', '.join(completed_names) or '无'} | "
        f"未越过({len(pending_names)}/{total}): "
        f"{', '.join(pending_names) or '无'}"
    )


def timeout_reached(started: float, now: float, timeout: float) -> bool:
    """Safely evaluate a monotonic inactivity deadline; invalid clocks fail closed."""
    values = (started, now, timeout)
    return (
        all(isfinite(float(value)) for value in values)
        and float(started) > 0.0
        and float(timeout) > 0.0
        and float(now) - float(started) >= float(timeout)
    )


def terminal_pose_reached(
    robot_pose: Optional[Tuple[float, float, float]],
    terminal_pose: Optional[Tuple[float, float, float]],
    tolerance: float,
) -> bool:
    """Return whether the robot is already inside the finish position tolerance.

    Nav2 may report ``ABORTED`` when its costmap cannot construct the final few
    centimetres of a path, even though localization already places the base inside
    the configured finish circle.  Mission completion is a position fact, not an
    Action-status fact, so both the result callback and the periodic loop use this
    helper.  Yaw is intentionally ignored: the competition finish is an area and a
    real quadruped must not rotate unnecessarily after it has returned safely.
    Invalid/non-finite poses fail closed.
    """
    if robot_pose is None or terminal_pose is None:
        return False
    values = (*robot_pose[:2], *terminal_pose[:2], tolerance)
    if not all(isfinite(float(value)) for value in values):
        return False
    if float(tolerance) < 0.0:
        return False
    return hypot(
        float(robot_pose[0]) - float(terminal_pose[0]),
        float(robot_pose[1]) - float(terminal_pose[1]),
    ) <= float(tolerance)


def canonical_obstacle_id(name: str) -> str:
    """把终端显示名称转换为稳定比赛语义 ID；未知名称返回空字符串。

    名称来自通用点云/视觉分类，不读取场地坐标。桥 A/B 的单侧 14° 引坡无法仅凭一帧
    局部点云可靠区分，先记为 ``wooden_bridge_unknown``，任务层在看到第二座不同位置的
    木桥后再补齐 A/B，避免为了凑任务数而编造类别。
    """
    text = str(name).strip()
    if "直角绕杆" in text:
        return "right_angle_poles"
    if "砂砾" in text or "坑区护栏" in text:
        return "gravel_wood_pit"
    if "限高杆" in text:
        return "height_bar"
    if "主斜坡" in text:
        return "main_slope"
    if "木桥 B" in text:
        return "wooden_bridge_b"
    if "木桥 A" in text:
        return "wooden_bridge_a"
    # 该显示文案明确表示点云尚不能在“普通台阶/木桥踏板”之间判定；把它直接折算成
    # wooden_bridge_unknown 会让任意低边缘都能得分并触发仿真直行，必须继续换视角。
    if "台阶或木桥" in text:
        return ""
    if "木桥" in text:
        return "wooden_bridge_unknown"
    if "T 字形台阶" in text:
        return "t_shaped_stairs"
    if "高墙" in text:
        return "high_wall"
    # 场地边缘可能具有与坑洞相同的负高度，但不是八项比赛障碍，明确保持无语义 ID。
    if "场地边界" in text:
        return ""
    return ""


def semantic_id_for_action(candidate: str, obstacle_type: int) -> str:
    """用几何 Action 类型校验语义名称，避免异步话题把上一帧名称记到下一障碍。

    坑、限高杆、绕杆、坡和高墙各有唯一几何类型，可以在名称冲突时确定性纠正。
    STEP 同时覆盖两座木桥和 T 字台阶，不能仅凭动作类型猜三者；这时只接受兼容名称。
    """
    candidate = str(candidate)
    obstacle_type = int(obstacle_type)
    # Near clipping can reduce a thin crossbar to one post / a short wall. Preserve
    # only an already-derived specific name: a pit rail, bridge side or arena edge can
    # produce the same coarse WALL/POLE label, so filling an absent name from type alone
    # created false high-wall/pole completions during the 2026-08-28 full-field run.
    if obstacle_type == TraverseObstacle.Goal.OBSTACLE_WALL:
        return candidate if candidate in {"height_bar", "high_wall"} else ""
    if obstacle_type == TraverseObstacle.Goal.OBSTACLE_POLE:
        return candidate if candidate in {"right_angle_poles", "height_bar"} else ""
    compatible_by_type = {
        # 木桥 B 的规则板间隙可形成 PIT；砂砾坑自身当然也是 PIT。
        TraverseObstacle.Goal.OBSTACLE_PIT: {
            "gravel_wood_pit",
            "main_slope",
            "wooden_bridge_a",
            "wooden_bridge_b",
            "t_shaped_stairs",
            "wooden_bridge_unknown",
            "height_bar",
            "high_wall",
        },
        # 坑区护栏有时先形成 BAR 几何，名称层会继续等待坑底回波。
        TraverseObstacle.Goal.OBSTACLE_BAR: {
            "height_bar", "gravel_wood_pit", "high_wall",
        },
        TraverseObstacle.Goal.OBSTACLE_STEP: {
            "height_bar",
            "high_wall",
            "main_slope",
            "wooden_bridge_a",
            "wooden_bridge_b",
            "wooden_bridge_unknown",
            "t_shaped_stairs",
            "gravel_wood_pit",
        },
        # 10° 主坡和 14° 木桥引坡都由 CLEAR+traversal_required 映射为 SLOPE。
        TraverseObstacle.Goal.OBSTACLE_SLOPE: {
            "main_slope", "wooden_bridge_a", "wooden_bridge_unknown",
        },
    }
    if candidate in compatible_by_type.get(obstacle_type, set()):
        return candidate
    return ""


def semantic_after_approach_stall(
    initial_id: str,
    live_id: str,
    obstacle_type: int,
    spatial_match: bool,
) -> str:
    """Keep a confirmed obstacle identity through near-field classifier flicker.

    The far/medium-range view is usually the best semantic view.  At the Nav2 inflation
    boundary a crossbar may be cropped to one post, or a wall edge may look like a step.
    Throwing away the identity captured when the approach goal was created caused the
    mission to cancel and immediately approach the same physical entry forever.

    Preservation is deliberately conditional: the live observation must still project to
    the same map entry and its coarse action type must remain compatible.  A nearby but
    different obstacle therefore cannot inherit the old identity.
    """
    initial_id = str(initial_id)
    live_id = str(live_id)
    if (
        bool(spatial_match)
        and is_actionable_semantic_id(initial_id)
        and semantic_id_for_action(initial_id, int(obstacle_type)) == initial_id
    ):
        return initial_id
    return live_id if is_actionable_semantic_id(live_id) else ""


def action_type_for_semantic(semantic_id: str, fallback: int) -> int:
    """把已锁定比赛语义映射为稳定 Action 粗类型。

    接近到相机近裁剪区后，最后一帧局部点云可能只看到障碍后方表面；任务层已经通过
    多帧和空间锁确认语义时，应保持原控制合同，而不是随退化帧在 WALL/STEP/PIT 之间
    跳变。未知语义仍返回实时几何 fallback，不做猜测。
    """
    by_semantic = {
        "right_angle_poles": TraverseObstacle.Goal.OBSTACLE_POLE,
        "gravel_wood_pit": TraverseObstacle.Goal.OBSTACLE_PIT,
        "height_bar": TraverseObstacle.Goal.OBSTACLE_BAR,
        "high_wall": TraverseObstacle.Goal.OBSTACLE_WALL,
        "main_slope": TraverseObstacle.Goal.OBSTACLE_SLOPE,
        "wooden_bridge_a": TraverseObstacle.Goal.OBSTACLE_STEP,
        "wooden_bridge_b": TraverseObstacle.Goal.OBSTACLE_STEP,
        "wooden_bridge_unknown": TraverseObstacle.Goal.OBSTACLE_STEP,
        "t_shaped_stairs": TraverseObstacle.Goal.OBSTACLE_STEP,
    }
    return int(by_semantic.get(str(semantic_id), int(fallback)))


def is_actionable_semantic_id(semantic_id: str) -> bool:
    """判断语义是否已唯一对应一项比赛障碍。

    ``wooden_bridge_unknown`` 只表示局部点云看到了平整踏板；主坡长侧和 T 台
    的局部踏面都可能产生同样轮廓。2026-08-28 联合回归已经实际观测到主坡
    侧面短暂输出该名称，因此它不能触发 Action。任务必须换观察站，直到
    看到 A 的 14° 引坡或 B 的周期板缝；不靠 world 坐标补齐类别。
    """
    return str(semantic_id) in COMPETITION_OBSTACLE_IDS


def semantic_task_is_complete(
    semantic_id: str, completed_ids: Sequence[str]
) -> bool:
    """Return whether this semantic is no longer eligible for another Action.

    Every named competition obstacle occurs once, so a completed ``gravel_wood_pit``
    must be rejected even if a later false positive appears elsewhere.  This guard is
    intentionally repeated immediately before handoff because a target can change
    semantic while Nav2 is cancelling.  The unresolved bridge label never authorises
    an Action, but its ledger state remains incomplete until both named bridge tasks
    are complete; spatial/segment guards independently prevent double counting.
    """
    candidate = str(semantic_id)
    completed = set(str(item) for item in completed_ids)
    if candidate == "wooden_bridge_unknown":
        return {"wooden_bridge_a", "wooden_bridge_b"}.issubset(completed)
    return candidate in COMPETITION_OBSTACLE_IDS and candidate in completed


def traversal_segment_matches(
    candidate_id: str,
    completed_id: str,
    position: Tuple[float, float],
    start: Tuple[float, float],
    end: Tuple[float, float],
    radius: float,
) -> bool:
    """Return whether a detection belongs to an already traversed structure.

    Long obstacles need segment-shaped de-duplication because the perceived
    front edge moves from entrance to exit.  Spatial overlap alone is unsafe
    in a dense competition field: the end of one bridge/slope can be beside a
    stair or wall.  A segment therefore suppresses only the *same stable
    competition semantic*.  An empty/ambiguous candidate is deliberately not
    suppressed; it must be observed until the normal semantic gate resolves
    it, rather than being silently discarded as an adjacent completed task.
    """
    candidate = (
        str(candidate_id)
        if str(candidate_id) in COMPETITION_OBSTACLE_IDS
        else canonical_obstacle_id(candidate_id)
    )
    completed = (
        str(completed_id)
        if str(completed_id) in COMPETITION_OBSTACLE_IDS
        else canonical_obstacle_id(completed_id)
    )
    if not candidate or not completed or candidate != completed:
        return False
    return distance_to_segment(position, start, end) <= float(radius)


def resolve_completed_semantics(
    completed: Sequence[str], new_id: str
) -> Tuple[str, ...]:
    """合并一次成功结果，并保守解决局部点云无法区分的木桥 A/B。

    两个不同入口的成功结果由位置去重逻辑保证。若一个结果只能确认“某座木桥”，先保留
    unknown；当随后确认 B 或出现第二个独立 unknown 时，八项清单才能补齐 A/B。
    """
    result = list(dict.fromkeys(str(item) for item in completed if item))
    if not new_id:
        return tuple(result)
    if new_id == "wooden_bridge_unknown":
        unknown_count = sum(item.startswith("wooden_bridge_unknown") for item in result)
        marker = f"wooden_bridge_unknown_{unknown_count + 1}"
        result.append(marker)
    elif new_id not in result:
        result.append(new_id)

    bridge_unknowns = [item for item in result if item.startswith("wooden_bridge_unknown")]
    if "wooden_bridge_b" in result and bridge_unknowns and "wooden_bridge_a" not in result:
        result.append("wooden_bridge_a")
        result.remove(bridge_unknowns[0])
    elif len(bridge_unknowns) >= 2:
        for bridge_id in ("wooden_bridge_a", "wooden_bridge_b"):
            if bridge_id not in result:
                result.append(bridge_id)
        result = [item for item in result if not item.startswith("wooden_bridge_unknown")]
    return tuple(result)


def normalized_angle(angle: float) -> float:
    """把任意弧度折叠到 [-pi, pi]，供停滞检测和终点航向判断共用。"""
    return atan2(sin(float(angle)), cos(float(angle)))


def bounded_alignment_delta(
    heading_error: float,
    trigger_angle: float,
    maximum_step: float,
) -> float:
    """Return a safe rotation-only correction, or zero when already aligned.

    Obstacle entry orientation is estimated from a fitted ramp axis or the PCA
    normal of its front edge.  Even after temporal filtering it can contain a
    short outlier, so the mission never applies an arbitrarily large turn from
    one observation.  Values below ``trigger_angle`` are left to the normal
    Nav2 approach controller; larger values are clipped to ``maximum_step``.

    Keeping this decision as a pure helper makes the important invariant easy
    to test: a confirmed obstacle is approached only after a bounded in-place
    turn, while NaN/Inf can never become a navigation goal.
    """
    values = (heading_error, trigger_angle, maximum_step)
    if not all(isfinite(float(value)) for value in values):
        return 0.0
    error = normalized_angle(float(heading_error))
    threshold = max(0.01, abs(float(trigger_angle)))
    limit = max(threshold, min(pi * 0.5, abs(float(maximum_step))))
    if abs(error) < threshold:
        return 0.0
    return max(-limit, min(limit, error))


def distance_to_segment(
    point: Tuple[float, float],
    start: Tuple[float, float],
    end: Tuple[float, float],
) -> float:
    """返回点到已通过路线段的二维最短距离。

    长木桥的入口和出口可能相距数米，而且桥板间隙会让粗几何在 STEP/PIT 间切换。
    只按“同类型入口圆”去重会在出口把同一座桥再计一次；保存 Action 实际走过的线段
    能解决该问题，同时不依赖障碍 world 坐标。
    """
    px, py = float(point[0]), float(point[1])
    ax, ay = float(start[0]), float(start[1])
    bx, by = float(end[0]), float(end[1])
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-9:
        return hypot(px - ax, py - ay)
    projection = max(
        0.0,
        min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared),
    )
    return hypot(px - (ax + projection * dx), py - (ay + projection * dy))


def target_is_in_heading_cone(
    robot: Tuple[float, float, float],
    target: Optional[Tuple[float, float]],
    half_angle: float = 0.75,
) -> bool:
    """判断 Nav2 平移目标是否仍位于机身当前朝向的前方锥内。"""
    if target is None:
        return False
    dx, dy = float(target[0]) - robot[0], float(target[1]) - robot[1]
    if hypot(dx, dy) < 0.05:
        return True
    error = normalized_angle(atan2(dy, dx) - robot[2])
    return abs(error) <= max(0.05, min(pi, float(half_angle)))


def mission_score(completed_ids: Sequence[str], returned_home: bool) -> int:
    """按规则 V1.0 计算完全自主模式的基础得分，不包含裁判罚分。"""
    unique = set(completed_ids).intersection(COMPETITION_OBSTACLE_IDS)
    return 150 * len(unique) + (100 if returned_home else 0)


def select_semantic_vote(votes: Sequence[str], fallback: str = "") -> str:
    """对同一空间目标做“近场优先、全窗口兜底”的时序语义投票。

    接近障碍时，远场轮廓可能先连续产生一个粗语义，近场才看到墙体厚度、桥板缝等
    判别结构。若永远累计全历史多数票，正确的近场证据会被早期票永久压住。最近四个
    有效观测中同类至少出现两次时优先采用它；否则才使用全窗口多数票。单帧跳变仍然
    不能改类，而且算法不依赖 Gazebo 坐标或障碍顺序。
    """
    cleaned = [str(item) for item in votes if str(item)]
    recent = cleaned[-4:]
    if recent:
        recent_counts = {item: recent.count(item) for item in set(recent)}
        recent_last = {
            item: max(index for index, value in enumerate(recent) if value == item)
            for item in recent_counts
        }
        recent_best = max(
            recent_counts,
            key=lambda item: (recent_counts[item], recent_last[item]),
        )
        if recent_counts[recent_best] >= 2:
            return recent_best
    counts = {}
    last_index = {}
    for index, item in enumerate(cleaned):
        counts[item] = counts.get(item, 0) + 1
        last_index[item] = index
    if not counts:
        return str(fallback)
    best = max(counts, key=lambda item: (counts[item], last_index[item]))
    return best if counts[best] >= 2 else str(fallback or best)


def select_full_semantic_vote(votes: Sequence[str], fallback: str = "") -> str:
    """Select the majority across the complete bounded target history.

    This is used only once, when an approach first becomes the locked mission
    entry.  Unlike the display-oriented recent vote, it preserves the richer
    far/mid-range structure while Nav2 finishes cancelling its frontier goal.
    The deque remains spatially bounded and finite, so a genuinely new target
    still starts with an empty history.
    """
    cleaned = [str(item) for item in votes if str(item)]
    if not cleaned:
        return str(fallback)
    counts = {item: cleaned.count(item) for item in set(cleaned)}
    last = {
        item: max(i for i, value in enumerate(cleaned) if value == item)
        for item in counts
    }
    best = max(counts, key=lambda item: (counts[item], last[item]))
    return best if counts[best] >= 3 else str(fallback or best)


def semantic_vote_is_confirmed(
    votes: Sequence[str],
    candidate: str,
    *,
    minimum_votes: int = 3,
    recent_window: int = 5,
) -> bool:
    """Return whether a class has enough repeated *recent* evidence.

    A compact arena can place several structures in one wide depth ROI.  A
    majority over the complete approach history is therefore unsuitable for
    authorising motion through an obstacle: an old far-field label could win
    after the robot has already turned towards another surface.  This bounded
    window is the final semantic gate used before TraverseObstacle.
    """
    value = str(candidate)
    if not value:
        return False
    count = max(1, int(minimum_votes))
    window = max(count, int(recent_window))
    recent = [str(item) for item in votes if str(item)][-window:]
    return recent.count(value) >= count


def replacement_semantic_vote(
    votes: Sequence[str],
    locked_id: str,
    *,
    minimum_votes: int = 3,
    recent_window: int = 6,
) -> str:
    """Return a sustained *different* semantic, otherwise an empty string.

    A semantic lock protects an approach from one-frame near-clipping noise, but it
    must not become permanent.  During the 2026-08-28 full-field run the live safety
    output had changed from ``high_wall`` to ``t_shaped_stairs`` for several seconds,
    while the mission still handed the old wall name to the Action server.  Requiring
    at least three votes in a bounded recent window lets a genuinely better viewpoint
    correct that lock, while two alternating frames still cannot change a task.

    This helper only decides temporal agreement.  Callers must additionally validate
    the candidate against the current metric geometry before changing a lock or
    authorising ``TraverseObstacle``.
    """
    locked = str(locked_id)
    count = max(3, int(minimum_votes))
    window = max(count, int(recent_window))
    recent = [str(item) for item in votes if str(item)][-window:]
    candidate = select_semantic_vote(recent, "")
    if (
        not candidate
        or candidate == locked
        or not semantic_vote_is_confirmed(
            recent,
            candidate,
            minimum_votes=count,
            recent_window=window,
        )
    ):
        return ""
    return candidate


def dominant_planar_vote(votes: Sequence[str], minimum_votes: int = 3) -> str:
    """Preserve repeated ramp evidence while a close side crop is ambiguous.

    A continuous measured 10/14-degree plane is stronger structural evidence
    than a later flat STEP crop.  The latter can be the long side of that same
    ramp and previously caused a false T-stair / bridge-B traversal.  Return a
    planar semantic only while it still strictly outnumbers every competing
    actionable class.  Sustained new evidence therefore ages the old class out
    of the bounded deque and can still correct a genuinely wrong far view.
    """
    cleaned = [
        str(item)
        for item in votes
        if is_actionable_semantic_id(str(item))
    ]
    if not cleaned:
        return ""
    counts = {item: cleaned.count(item) for item in set(cleaned)}
    planar = max(
        ("main_slope", "wooden_bridge_a"),
        key=lambda item: counts.get(item, 0),
    )
    planar_count = counts.get(planar, 0)
    competing_count = max(
        (count for item, count in counts.items() if item != planar),
        default=0,
    )
    return (
        planar
        if planar_count >= max(1, int(minimum_votes))
        and planar_count > competing_count
        else ""
    )


@dataclass(frozen=True)
class Frontier:
    """地图坐标中的一个连通前沿候选。"""

    x: float
    y: float
    cells: int
    distance: float
    score: float


@dataclass
class ObservedObstacle:
    """One confirmed but not necessarily traversed competition obstacle.

    ``obstacle_*`` is the filtered map position of the structure. ``view_*`` is a
    known-safe robot pose from which the sensors actually observed it.  Revisit goals
    go to the viewpoint, never to the solid obstacle coordinate; Nav2 therefore keeps
    its normal collision semantics and the traversal controller receives control only
    after the regular alignment/handoff gates pass again.

    This record contains no Gazebo entity name or fixed arena coordinate.  It is built
    entirely from map->base_link and live perception, so the same mission code can be
    copied to another robot or an unknown real venue.
    """

    semantic_id: str
    obstacle_x: float
    obstacle_y: float
    view_x: float
    view_y: float
    view_yaw: float
    confidence: float
    last_seen: float
    retry_after: float = 0.0
    # 连续回访尝试次数只影响任务调度，不改变感知置信度。Nav2 到达观察位并不等于
    # 重新识别成功，因此每次尝试都指数退避，避免旧记录每隔固定时间抢占任务。
    revisit_failures: int = 0


@dataclass(frozen=True)
class FailedEntry:
    """One controller-rejected observation station and approach heading.

    Long structures can expose several valid-looking front edges.  A failed Action
    proves only that this particular station/heading pair was unsuitable; it must not
    blacklist the whole obstacle or assume a known venue coordinate.  The record is
    therefore local, semantic-specific and automatically expires.
    """

    semantic_id: str
    robot_x: float
    robot_y: float
    robot_yaw: float
    expires: float


def failed_entry_matches(
    records: Sequence[FailedEntry],
    semantic_id: str,
    robot_pose: Tuple[float, float, float],
    current_time: float,
    maximum_station_distance: float,
    maximum_heading_difference: float,
    require_new_station: bool = False,
) -> bool:
    """Return whether an Action would repeat a still-active rejected entry."""
    return any(
        str(record.semantic_id) == str(semantic_id)
        and float(record.expires) > float(current_time)
        and hypot(
            float(record.robot_x) - float(robot_pose[0]),
            float(record.robot_y) - float(robot_pose[1]),
        ) <= max(0.0, float(maximum_station_distance))
        and (
            bool(require_new_station)
            or abs(normalized_angle(
                float(record.robot_yaw) - float(robot_pose[2])
            )) <= max(0.0, float(maximum_heading_difference))
        )
        for record in records
    )


@dataclass
class TraversalVerification:
    """Frozen evidence awaiting task-level confirmation after Action success.

    The motion controller owns contact, attitude and actuator checks.  This task-level
    record independently checks that the base actually moved from the entry side to
    the far side and then became stationary.  Keeping the two layers independent
    prevents either a stale TF or an incorrectly optimistic Action result from marking
    a competition obstacle complete by itself.
    """

    semantic_id: str
    obstacle_type: int
    obstacle_position: Tuple[float, float]
    robot_start: Tuple[float, float]
    controller_message: str
    started: float
    last_pose: Optional[Tuple[float, float, float]] = None
    stable_since: float = 0.0


def traversal_crossing_evidence(
    start: Tuple[float, float],
    obstacle: Tuple[float, float],
    end: Tuple[float, float],
    *,
    minimum_displacement: float,
    beyond_obstacle_margin: float,
) -> bool:
    """Return whether map poses prove forward progress beyond the entry edge.

    The direction is inferred online from the Action start towards the perceived
    obstacle, never from a Gazebo/world layout.  The check intentionally does not claim
    to prove foot contact or complete body clearance; those are mandatory duties of the
    real ``TraverseObstacle`` server.  It catches the important upper-layer failure in
    which a server reports success while the robot stayed in front of the obstacle.
    """
    values = (*start, *obstacle, *end, minimum_displacement, beyond_obstacle_margin)
    if not all(isfinite(float(value)) for value in values):
        return False
    direction_x = float(obstacle[0]) - float(start[0])
    direction_y = float(obstacle[1]) - float(start[1])
    entry_distance = hypot(direction_x, direction_y)
    if entry_distance < 0.10:
        return False
    unit_x, unit_y = direction_x / entry_distance, direction_y / entry_distance
    displacement = hypot(float(end[0]) - start[0], float(end[1]) - start[1])
    beyond = (
        (float(end[0]) - obstacle[0]) * unit_x
        + (float(end[1]) - obstacle[1]) * unit_y
    )
    return (
        displacement >= max(0.0, float(minimum_displacement))
        and beyond >= max(0.0, float(beyond_obstacle_margin))
    )


def traversal_geometry_evidence(
    semantic_id: str,
    start: Tuple[float, float],
    obstacle: Tuple[float, float],
    end: Tuple[float, float],
    *,
    minimum_displacement: float,
    beyond_obstacle_margin: float,
) -> bool:
    """Apply a completion geometry that matches the competition task topology.

    Seven structures have an entrance surface and therefore require the robot to
    finish beyond the online-observed entry plane.  The right-angle pole course is
    different: its mandatory zones form a turn, so a valid finish may be lateral to
    that plane.  For that one semantic the Action controller proves zone execution;
    the mission layer still independently requires finite map poses and sufficient
    total displacement.  Stability is checked separately for every task.
    """
    if str(semantic_id) == "right_angle_poles":
        values = (*start, *end, minimum_displacement)
        return bool(
            all(isfinite(float(value)) for value in values)
            and hypot(float(end[0]) - start[0], float(end[1]) - start[1])
            >= max(0.0, float(minimum_displacement))
        )
    return traversal_crossing_evidence(
        start,
        obstacle,
        end,
        minimum_displacement=minimum_displacement,
        beyond_obstacle_margin=beyond_obstacle_margin,
    )


def choose_pending_obstacle(
    records: Sequence[ObservedObstacle],
    completed_ids: Sequence[str],
    robot: Tuple[float, float, float],
    now: float,
) -> Optional[ObservedObstacle]:
    """Choose one known unfinished obstacle for active reacquisition.

    Only records whose retry cooldown expired are eligible.  The nearest previously
    safe viewpoint wins; confidence and recency break distance ties.  This produces a
    stable one-at-a-time policy and avoids abandoning a known task merely because a
    farther frontier currently has more unknown cells.
    """
    completed = set(str(item) for item in completed_ids)
    candidates = [
        item
        for item in records
        if is_actionable_semantic_id(item.semantic_id)
        and item.semantic_id not in completed
        and float(item.retry_after) <= float(now)
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda item: (
            hypot(item.view_x - robot[0], item.view_y - robot[1]),
            -float(item.confidence),
            -float(item.last_seen),
            item.semantic_id,
        ),
    )


def obstacle_revisit_delay(
    base_seconds: float,
    failure_count: int,
    maximum_seconds: float,
) -> float:
    """Return bounded exponential backoff for a repeatedly failed revisit.

    ``failure_count=1`` uses the base delay, then doubles until the configured
    maximum.  The helper is pure so scheduling policy can be regression-tested
    without running ROS or Nav2.
    """
    base = max(0.0, float(base_seconds))
    maximum = max(base, float(maximum_seconds))
    failures = max(1, int(failure_count))
    return min(maximum, base * (2.0 ** min(failures - 1, 16)))


def verification_station_matches(
    anchor: Optional[Tuple[float, float]],
    robot_position: Tuple[float, float],
    maximum_distance: float,
) -> bool:
    """Return whether alternate camera views came from the same robot station.

    A long bridge, ramp or pit can report a different *nearest obstacle point* after
    every in-place turn. Resetting the view counter from that moving point caused
    repeated ``1/4`` verification cycles. The robot pose is the stable reference:
    pure rotations at one station remain one bounded sequence, while driving to a
    genuinely new viewpoint starts a fresh sequence.
    """
    if anchor is None:
        return False
    return hypot(
        float(robot_position[0]) - float(anchor[0]),
        float(robot_position[1]) - float(anchor[1]),
    ) <= max(0.0, float(maximum_distance))


def matching_pending_semantic(
    records: Sequence[ObservedObstacle],
    completed_ids: Sequence[str],
    obstacle_position: Tuple[float, float],
    coarse_action_type: int,
    maximum_distance: float,
    current_time: Optional[float] = None,
) -> str:
    """Match near-field coarse geometry to the nearest pending semantic record.

    This performs no perception on its own.  It only reconnects a live map position
    and coarse Action type to an identity that was confirmed from earlier multi-frame
    evidence.  A completed task, incompatible type, or out-of-radius record cannot
    match, which prevents an old name leaking to the next obstacle.  When
    ``current_time`` is supplied, a record whose retry deadline is still in the future
    is also excluded.  This last guard is important: otherwise the spatial fallback
    could silently bypass the backoff applied after a rejected traversal and hand the
    same unsafe entry to the controller again on the next sensor frame.
    """
    completed = set(str(item) for item in completed_ids)
    candidates = []
    for record in records:
        semantic_id = str(record.semantic_id)
        if semantic_id in completed:
            continue
        if current_time is not None and float(record.retry_after) > float(current_time):
            continue
        distance = hypot(
            float(obstacle_position[0]) - float(record.obstacle_x),
            float(obstacle_position[1]) - float(record.obstacle_y),
        )
        if (
            distance <= max(0.0, float(maximum_distance))
            and semantic_id_for_action(semantic_id, coarse_action_type)
            == semantic_id
        ):
            candidates.append((distance, semantic_id))
    return min(candidates)[1] if candidates else ""


def matching_pending_semantic_from_viewpoint(
    records: Sequence[ObservedObstacle],
    completed_ids: Sequence[str],
    robot_pose: Tuple[float, float, float],
    coarse_action_type: int,
    maximum_view_distance: float,
    maximum_heading_difference: float,
    current_time: Optional[float] = None,
) -> str:
    """Recover a pending ID when the robot revisits its confirmed camera station.

    The perceived nearest point on a stair or long bridge can move by more than one
    metre as the camera turns, while the robot has returned to the exact map pose from
    which that semantic was confirmed. Reuse still requires an unfinished record,
    compatible coarse Action type, bounded robot-position error and bounded heading
    difference. This makes the fallback independent of Gazebo coordinates and prevents
    an arbitrary stale label authorising traversal from elsewhere in the arena.
    """
    completed = set(str(item) for item in completed_ids)
    candidates = []
    for record in records:
        semantic_id = str(record.semantic_id)
        if semantic_id in completed:
            continue
        if current_time is not None and float(record.retry_after) > float(current_time):
            continue
        view_distance = hypot(
            float(robot_pose[0]) - float(record.view_x),
            float(robot_pose[1]) - float(record.view_y),
        )
        heading_difference = abs(normalized_angle(
            float(robot_pose[2]) - float(record.view_yaw)
        ))
        if (
            view_distance <= max(0.0, float(maximum_view_distance))
            and heading_difference <= max(0.0, float(maximum_heading_difference))
            and semantic_id_for_action(semantic_id, coarse_action_type)
            == semantic_id
        ):
            candidates.append((view_distance, heading_difference, semantic_id))
    return min(candidates)[2] if candidates else ""


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


def distance_inside_grid_edge(grid: OccupancyGrid, x: float, y: float) -> float:
    """返回地图内一点到 OccupancyGrid 外框的最短距离，地图外返回 0。

    比赛场地边缘在斜视深度云中可能暂时拟合成 10°/14° 坡面。真实内部障碍会随着
    探索逐渐位于地图内部，而场地外框始终贴近 SLAM 栅格外缘。任务层用这个在线地图
    关系延迟边缘目标的 Action 交接，不依赖 14 m × 6 m 尺寸或 Gazebo 坐标。
    """
    resolution = float(grid.info.resolution)
    width, height = int(grid.info.width), int(grid.info.height)
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return 0.0
    origin = grid.info.origin.position
    dx, dy = float(x) - origin.x, float(y) - origin.y
    yaw = _origin_yaw(grid)
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    maximum_x, maximum_y = width * resolution, height * resolution
    if not (0.0 <= local_x <= maximum_x and 0.0 <= local_y <= maximum_y):
        return 0.0
    return min(local_x, local_y, maximum_x - local_x, maximum_y - local_y)


def map_edge_allows_obstacle_handoff(
    semantic_id: str,
    obstacle_type: int,
    edge_distance: float,
    minimum_margin: float,
) -> bool:
    """判断贴近当前 SLAM 栅格边缘的障碍是否仍可进入越障入口流程。

    普通目标必须位于当前地图边框以内，防止把尚未观测的场地外部、斜视边线或地图
    扩展瞬态误当成障碍入口。高墙是唯一必要的例外：垂直墙面会遮挡其后的激光，墙面
    中心因此可能始终就是 ``OccupancyGrid`` 的已知区边缘。如果此时比赛语义已经稳定
    确认为 ``high_wall``，并且实时点云粗类型也确认为 ``WALL``，允许任务层继续对正和
    Action 交接。调用方仍须先检查“场地边界”名称、感知置信度和多帧确认，所以这个
    例外不会把普通未知边界直接放行。

    真机调参提示：不要为了高墙把 ``obstacle_map_edge_margin`` 全局调成负数。若真机
    仍误拦高墙，应先检查墙面高度/宽度标定和 ``front_obstacle_name`` 是否稳定；若其他
    障碍误靠近地图外框，则应增大 ``minimum_margin``，本例外只影响确认后的高墙。
    """
    distance = float(edge_distance)
    margin = max(0.0, float(minimum_margin))
    if isfinite(distance) and distance >= margin:
        return True
    return (
        str(semantic_id) == "high_wall"
        and int(obstacle_type) == int(TraverseObstacle.Goal.OBSTACLE_WALL)
    )


def extract_frontiers(
    grid: OccupancyGrid,
    robot_xy: Tuple[float, float],
    *,
    minimum_cells: int = 8,
    occupied_threshold: int = 50,
    minimum_distance: float = 0.55,
    maximum_distance: float = 7.0,
    goal_standoff: float = 0.0,
    goal_clearance: float = 0.0,
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
            # frontier cell 与未知区直接相邻，目标若原样交给 Nav2，机器人外形和
            # inflation layer 常会在最后几十厘米进入不可达状态。生产模式把目标沿
            # “前沿 -> 机器人”方向退回已知自由区；原始前沿仍决定信息增益和朝向。
            raw_x, raw_y = x, y
            if goal_standoff > 0.0:
                safe_goal = frontier_goal_in_known_free_space(
                    grid,
                    (raw_x, raw_y),
                    robot_xy,
                    standoff=goal_standoff,
                    clearance=goal_clearance,
                    occupied_threshold=occupied_threshold,
                )
                if safe_goal is None:
                    continue
                x, y = safe_goal
            distance = hypot(x - robot_xy[0], y - robot_xy[1])
            if minimum_distance <= distance <= maximum_distance:
                # 信息增益为主、距离轻微加权；这会先扩展较长边界，同时减少脚边抖动。
                score = len(cells) * (1.0 + min(distance, 3.0) * 0.12)
                frontiers.append(Frontier(x, y, len(cells), distance, score))
    return sorted(frontiers, key=lambda item: item.score, reverse=True)


def frontier_goal_in_known_free_space(
    grid: OccupancyGrid,
    frontier_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    *,
    standoff: float = 0.45,
    clearance: float = 0.20,
    occupied_threshold: int = 50,
) -> Optional[Tuple[float, float]]:
    """在真实前沿后方寻找有净空的导航目标。

    该函数只读取在线 OccupancyGrid，不知道赛场尺寸、障碍顺序或 Gazebo 坐标。未知格
    和占用格都计为无净空；从期望退距开始继续向机器人方向搜索，最多退 1.2 m。
    这样能显著减少目标本身落在静态层膨胀区而反复超时，同时仍让 2.5 m 传感器 ROI
    覆盖前沿后的障碍。
    """
    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return None
    dx = float(robot_xy[0]) - float(frontier_xy[0])
    dy = float(robot_xy[1]) - float(frontier_xy[1])
    length = hypot(dx, dy)
    if length <= 1e-6:
        return None
    ux, uy = dx / length, dy / length
    first = max(resolution, float(standoff))
    maximum = min(max(first, 1.20), max(first, length - resolution))
    clearance_cells = max(0, int(round(max(0.0, float(clearance)) / resolution)))
    offset = first
    while offset <= maximum + 1e-9:
        x = float(frontier_xy[0]) + ux * offset
        y = float(frontier_xy[1]) + uy * offset
        cell = world_to_cell(grid, x, y)
        if cell is not None:
            col, row = cell
            free = True
            for oy in range(-clearance_cells, clearance_cells + 1):
                for ox in range(-clearance_cells, clearance_cells + 1):
                    if ox * ox + oy * oy > clearance_cells * clearance_cells:
                        continue
                    checked_col, checked_row = col + ox, row + oy
                    if not (
                        0 <= checked_col < int(grid.info.width)
                        and 0 <= checked_row < int(grid.info.height)
                    ):
                        free = False
                        break
                    value = int(
                        grid.data[checked_row * int(grid.info.width) + checked_col]
                    )
                    if value < 0 or value >= int(occupied_threshold):
                        free = False
                        break
                if not free:
                    break
            if free:
                return x, y
        offset += max(resolution, 0.10)
    return None


def recovery_station_in_known_free_space(
    grid: OccupancyGrid,
    robot_pose: Tuple[float, float, float],
    distance: float,
    *,
    clearance: float = 0.35,
    occupied_threshold: int = 50,
) -> Optional[Tuple[float, float, float]]:
    """Choose a collision-free observation station from an online costmap.

    The previous recovery always moved ``distance`` metres along the newly rotated
    body axis.  That works in open space but repeatedly selected another point on the
    same slope/step in the full-field run.  This helper checks both the swept centre
    line and a circular body-clearance footprint, preferring forward, then diagonal,
    lateral and reverse alternatives.  Unknown cells are rejected; no venue size,
    obstacle name or fixed world coordinate is used.
    """
    resolution = float(grid.info.resolution)
    width, height = int(grid.info.width), int(grid.info.height)
    if (
        resolution <= 0.0
        or width <= 0
        or height <= 0
        or len(grid.data) != width * height
        or not all(isfinite(float(value)) for value in (*robot_pose, distance, clearance))
        or float(distance) <= 0.0
    ):
        return None
    clearance_cells = max(0, int(ceil(max(0.0, float(clearance)) / resolution)))

    def is_clear(x: float, y: float) -> bool:
        cell = world_to_cell(grid, x, y)
        if cell is None:
            return False
        col, row = cell
        for oy in range(-clearance_cells, clearance_cells + 1):
            for ox in range(-clearance_cells, clearance_cells + 1):
                if ox * ox + oy * oy > clearance_cells * clearance_cells:
                    continue
                checked_col, checked_row = col + ox, row + oy
                if not (0 <= checked_col < width and 0 <= checked_row < height):
                    return False
                value = int(grid.data[checked_row * width + checked_col])
                if value < 0 or value >= int(occupied_threshold):
                    return False
        return True

    start_x, start_y, start_yaw = map(float, robot_pose)
    # Preserve the requested 0.8 m when possible. Shorter candidates allow recovery
    # in narrow passages without silently increasing the configured maximum motion.
    lengths = tuple(dict.fromkeys((float(distance), 0.75 * float(distance), 0.50 * float(distance))))
    angle_offsets = (0.0, pi / 4.0, -pi / 4.0, pi / 2.0, -pi / 2.0, pi)
    sample_step = max(0.05, resolution * 0.5)
    for length in lengths:
        for offset in angle_offsets:
            heading = normalized_angle(start_yaw + offset)
            steps = max(1, int(ceil(length / sample_step)))
            if all(
                is_clear(
                    start_x + cos(heading) * length * step / steps,
                    start_y + sin(heading) * length * step / steps,
                )
                for step in range(1, steps + 1)
            ):
                return (
                    start_x + cos(heading) * length,
                    start_y + sin(heading) * length,
                    heading,
                )
    return None


def extract_coverage_goals(
    grid: OccupancyGrid,
    robot_xy: Tuple[float, float],
    visited: Sequence[Tuple[float, float]],
    *,
    spacing: float = 0.60,
    clearance: float = 0.30,
    visit_radius: float = 0.75,
    minimum_distance: float = 0.80,
    maximum_distance: float = 7.0,
    occupied_threshold: int = 50,
) -> List[Frontier]:
    """从已知自由区生成覆盖巡检目标，补足纯前沿探索的盲区。

    2D 雷达可能在场地中央就看完所有墙体，于是地图已没有 unknown frontier，但相机和
    深度点云仍未近距离观察每个比赛障碍。这里按米制网格抽样已知自由栅格，只保留具有
    局部净空、尚未被机器人实际走近的位置，并优先选择离历史轨迹最远的目标。函数只
    使用在线 OccupancyGrid 和走过的轨迹，不知道比赛地图尺寸、障碍坐标或固定顺序。
    """
    resolution = float(grid.info.resolution)
    width, height = int(grid.info.width), int(grid.info.height)
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return []
    if len(grid.data) != width * height:
        return []
    stride = max(1, int(round(max(resolution, float(spacing)) / resolution)))
    clearance_cells = max(0, int(ceil(max(0.0, float(clearance)) / resolution)))
    history = list(visited) or [robot_xy]
    candidates = []
    for row in range(stride // 2, height, stride):
        for col in range(stride // 2, width, stride):
            value = int(grid.data[row * width + col])
            if value < 0 or value >= int(occupied_threshold):
                continue
            free = True
            for oy in range(-clearance_cells, clearance_cells + 1):
                for ox in range(-clearance_cells, clearance_cells + 1):
                    if ox * ox + oy * oy > clearance_cells * clearance_cells:
                        continue
                    checked_col, checked_row = col + ox, row + oy
                    if not (0 <= checked_col < width and 0 <= checked_row < height):
                        free = False
                        break
                    checked = int(grid.data[checked_row * width + checked_col])
                    if checked < 0 or checked >= int(occupied_threshold):
                        free = False
                        break
                if not free:
                    break
            if not free:
                continue
            x, y = _cell_to_world(grid, col, row)
            robot_distance = hypot(x - robot_xy[0], y - robot_xy[1])
            novelty = min(hypot(x - hx, y - hy) for hx, hy in history)
            if (
                novelty < max(0.0, float(visit_radius))
                or robot_distance < max(0.0, float(minimum_distance))
                or robot_distance > max(0.0, float(maximum_distance))
            ):
                continue
            # 新颖度决定是否能补扫未访问区域；距离仅作为轻微并列项，避免每次都只选
            # 机器人脚边的格点。沿途位姿会持续写入 visited，因此不会在同一路径往返。
            score = novelty * 10.0 + min(robot_distance, 3.0) * 0.1
            candidates.append(Frontier(x, y, 1, robot_distance, score))
    return sorted(candidates, key=lambda item: item.score, reverse=True)


def choose_frontier(
    candidates: Sequence[Frontier],
    blocked: Sequence[Tuple[float, float]],
    exclusion_radius: float,
) -> Optional[Frontier]:
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


def obstacle_geometry_fits_candidate(
    obstacle_id: str,
    safety: Optional[NavigationSafety],
) -> bool:
    """判断新鲜的点云几何是否与稳定障碍名称一致。

    这是进入 ``TraverseObstacle`` 前最后一道、故意偏保守的米制几何闸门。字段缺失、
    感知无效或几何超出比赛物体轮廓时一律返回 ``False``，让任务继续留在观察/验证
    阶段；这样可以避免斜坡—木桥、木桥—台阶交界处的瞬态轮廓被误当成越障入口。
    """
    if safety is None or not safety.perception_valid:
        return False
    obstacle_id = str(obstacle_id)
    if not is_actionable_semantic_id(obstacle_id):
        return False

    obstacle_type = int(safety.obstacle_type)
    pitch = abs(degrees(float(safety.slope_pitch)))
    roll = abs(degrees(float(safety.slope_roll)))
    height = float(safety.obstacle_height)
    depth = float(safety.pit_depth)
    roughness = float(safety.roughness)
    width = float(safety.width)
    clearance = float(safety.clearance_height)
    # NavigationSafety 正常由 assessor 先做有限值校验，但 Action 边界仍需独立防御。
    # 尤其 Python 的布尔短路可能让完整 T 台的 height/width 分支跳过 NaN pitch，若不在
    # 这里原子检查，损坏消息就可能绕过最终闸门。任一米制量测无穷或非数都拒绝整帧。
    if not all(isfinite(value) for value in (
        pitch,
        roll,
        height,
        depth,
        roughness,
        width,
        clearance,
    )):
        return False
    if any(value < 0.0 for value in (
        height,
        depth,
        roughness,
        width,
        clearance,
    )):
        return False

    if obstacle_id == "right_angle_poles":
        # 规则绕杆立柱应当窄、高，并明显突出局部地面。单独的视觉框也可能产生相似
        # 语义，因此只有点云同时满足这些尺寸，才允许进入比赛绕杆 Action 流程。
        return (
            obstacle_type == NavigationSafety.OBSTACLE_POLE
            and height >= 0.45
            and width <= 0.35
            and depth < 0.08
        )
    if obstacle_id == "height_bar":
        # 限高杆与坑洞/墙面分开校验，防止斜坡侧视轮廓误进入限高杆交接流程。
        return (
            obstacle_type in {
                NavigationSafety.OBSTACLE_BAR,
                NavigationSafety.OBSTACLE_WALL,
            }
            and 0.18 <= height <= 0.45
            and width <= 1.20
            and clearance >= 0.12
        )
    if obstacle_id == "main_slope":
        return (
            obstacle_type == NavigationSafety.OBSTACLE_CLEAR
            and 7.0 <= pitch <= 12.5
            and roll <= 8.0
            and roughness <= 0.050
        )
    if obstacle_id == "wooden_bridge_a":
        return (
            obstacle_type == NavigationSafety.OBSTACLE_CLEAR
            and 12.0 <= pitch <= 17.5
            and roll <= 8.0
            and roughness <= 0.045
            and width >= 0.80
        )
    if obstacle_id == "wooden_bridge_b":
        return (
            obstacle_type == NavigationSafety.OBSTACLE_STEP
            and 0.19 <= height <= 0.28
            and 0.55 <= width <= 1.40
            and roughness >= 0.078
            and pitch <= 6.0
            and roll <= 12.0
        )
    if obstacle_id == "t_shaped_stairs":
        # 正对 T 台存在三种合法深度轮廓：完整 0.40 m 顶部、只见前几级的局部踏面，
        # 以及参考平面跨越多级踏面后形成的阶梯总体趋势。无闪现样本属于第三种：残余
        # 高度仅 0.081 m，但仍保留 16.23°、0.029 m 残差和 0.998 m 宽度。名称层已经
        # 接受该证据，最终 Action 闸门不能重新引入旧的“高度至少 0.28 m、坡角不超过
        # 15°”矛盾。三种轮廓仍共同受 0.43 m 高度上限、6° 横滚和 0.08 m 残差上限
        # 约束，因此实测 0.44～0.46 m 的主斜坡侧面继续被拒绝。
        stepped_profile = (
            7.0 <= pitch <= 18.0
            and roughness >= 0.020
        )
        partial_tread = (
            height >= 0.28 and width >= 0.45 and roughness >= 0.040
        )
        step_profile = (
            obstacle_type == NavigationSafety.OBSTACLE_STEP
            and height <= 0.43
            and roughness <= 0.080
            and roll <= 6.0
            and (
                (width >= 0.60 and (height >= 0.32 or stepped_profile))
                or partial_tread
            )
        )
        # 近场参考平面可能落在较高踏面，使较低一级暂时成为 PIT。名称层只在这组
        # 严格坡角、坑深、低残余高度、残差和规则宽度同时成立时恢复 T 台语义；最终
        # Action 闸门复用同一轮廓，否则会出现“名称正确但永远不能交接”。横滚门排除
        # 从侧面观察到的坡/台边缘。
        pit_profile = (
            obstacle_type == NavigationSafety.OBSTACLE_PIT
            and 16.0 <= pitch <= 24.0
            and 0.20 <= depth <= 0.36
            and height < 0.12
            and 0.025 <= roughness <= 0.065
            and 0.75 <= width <= 1.25
            and roll <= 6.0
        )
        return step_profile or pit_profile
    if obstacle_id == "gravel_wood_pit":
        if obstacle_type == NavigationSafety.OBSTACLE_PIT:
            return 0.06 <= depth <= 0.30 and 0.05 <= height <= 0.34
        regular_pit = (
            obstacle_type == NavigationSafety.OBSTACLE_STEP
            and 0.10 <= height <= 0.22
            and 0.07 <= depth <= 0.30
            and roughness >= 0.020
            and 0.30 <= width <= 1.20
        )
        close_fill = (
            obstacle_type == NavigationSafety.OBSTACLE_STEP
            and 0.19 <= height <= 0.23
            and 0.02 <= depth <= 0.08
            and 0.045 <= roughness < 0.078
            and 0.40 <= width <= 1.40
        )
        return regular_pit or close_fill
    if obstacle_id == "high_wall":
        # 规则高墙约 0.30 m 高、1.00 m 宽；碎石坑边沿实测只有 0.10～0.25 m 高，
        # 因此完整墙面门限可排除这类裁切误报。若遮挡使点云粗分类表现为 PIT，则只在
        # 名称层使用的那组严格坡角、深度、粗糙度与宽度条件同时成立时放行。
        if obstacle_type == NavigationSafety.OBSTACLE_PIT:
            return (
                12.0 <= pitch <= 22.0
                and depth > 0.36
                and height < 0.10
                and 0.035 <= roughness <= 0.070
                and 0.80 <= width <= 1.20
            )
        return (
            obstacle_type == NavigationSafety.OBSTACLE_WALL
            and 0.27 <= height <= 0.42
            and 0.75 <= width <= 1.25
            and pitch <= 15.0
        )
    return False


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


def close_handoff_is_safe(
    semantic_id: str,
    distance: float,
    lateral_offset: float,
    heading_error: float,
    maximum_distance: float,
    maximum_lateral: float,
    maximum_heading_error: float,
) -> bool:
    """判断机器人已在 Nav2 容差内时能否直接切换到越障控制器。

    Nav2 的位置容差通常大于最后几厘米的入口目标。如果仍把当前位置作为新目标反复
    提交，Action 会立即成功并形成高频循环。这里要求比赛语义已经唯一确认，并同时
    检查距离、横偏和航向；任一条件不满足都只能继续对正或换视角观察。
    """
    return (
        is_actionable_semantic_id(semantic_id)
        and isfinite(float(distance))
        and isfinite(float(lateral_offset))
        and isfinite(float(heading_error))
        and 0.0 <= float(distance) <= max(0.0, float(maximum_distance))
        and abs(float(lateral_offset)) <= max(0.0, float(maximum_lateral))
        and abs(float(heading_error)) <= max(0.0, float(maximum_heading_error))
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

    def __init__(self, **node_kwargs):
        super().__init__("autonomous_mission", **node_kwargs)
        defaults = {
            "autostart": False, "map_timeout": 2.0, "guidance_timeout": 1.0,
            # map/TF 首次就绪后给感知稳定器一个完整窗口，避免第一帧 frontier 抢在
            # 起点正前方已存在的可靠障碍之前被提交。
            "startup_sensor_settle_time": 1.50,
            "frontier_minimum_cells": 8, "frontier_minimum_distance": 0.55,
            "frontier_maximum_distance": 7.0, "frontier_exclusion_radius": 0.65,
            # 前沿本身紧邻未知区；导航目标退回已知自由区并检查一圈净空，避免 DWB
            # 在最后几十厘米卡住。它不改变感知范围或障碍 Action 入口。
            "frontier_goal_standoff": 0.45,
            "frontier_goal_clearance": 0.20,
            # 激光已把整块场地观测为 known 后，继续在自由区做轨迹覆盖，保证近距相机/
            # 点云能逐项复核八个障碍；全部参数都是地图尺度，不含场地坐标。
            "coverage_goal_spacing": 0.60,
            "coverage_goal_clearance": 0.30,
            "coverage_visit_radius": 0.75,
            "coverage_record_spacing": 0.40,
            # 仅容忍 SLAM 栅格边界比机器人滞后几厘米；配合滚动全局代价地图继续选择
            # 已知自由前沿。超过该距离仍进入 WAITING_FOR_MAP，防止定位跳变后盲目规划。
            "map_boundary_tolerance": 0.30,
            # DWB 默认 goal checker 容差约 0.25 m。比它更近的入口目标会被立即判成功，
            # 表现为机器人只挪一下就停；任务层直接进入受控交接，不发送伪 Nav2 目标。
            "minimum_approach_goal_distance": 0.15,
            # 低于该角度时，原地旋转目标也会被 Nav2 立即判定成功。此时任务层应直接
            # 做严格交接判定或换视角复核，禁止发送零位移、零转角的 Action 目标。
            "minimum_alignment_command_angle": 0.14,
            # 一旦多帧语义已确认，先用纯旋转对准障碍法向，再提交带平移的入口目标。
            # 每次最多转 30°，并在转后等待新点云，避免单帧方向跳变造成左右摆头。
            "pre_alignment_trigger_angle": 0.18,
            "pre_alignment_max_step": 0.523599,
            "pre_alignment_settle_time": 0.60,
            "nav_rejection_retry_delay": 1.0,
            "nav_failure_retry_delay": 1.0,
            # TraverseObstacle 拒绝或地形挡住目标后，先原地换 90° 观察方向，再选择
            # 新目标。这是任务级恢复动作，不是腿部越障命令；真机应从 60° 标定。
            "failed_entry_turn_angle": 1.570796,
            "failed_entry_settle_time": 0.80,
            "failed_entry_memory_duration": 45.0,
            "failed_entry_station_tolerance": 0.65,
            "failed_entry_heading_tolerance": 0.70,
            "failed_entry_escape_distance": 0.80,
            "handoff_fallback_max_distance": 1.45,
            "handoff_fallback_max_lateral": 0.50,
            "direct_handoff_max_distance": 1.45,
            "handoff_fallback_spatial_tolerance": 0.90,
            # 障碍前缘漂移过大时，只在机器人真正回到已确认观察位、朝向也接近原方向时
            # 才允许用待办账本恢复 ID；仍必须通过实时粗类型和全部 Action 入口守卫。
            "handoff_fallback_viewpoint_tolerance": 0.60,
            "handoff_fallback_view_heading_tolerance": 0.70,
            # 真正 Action 交接使用与 Nav2 yaw_goal_tolerance 一致的严格角度；不能复用
            # “重复停滞容许值”，否则约 20° 的斜向入口也可能被当成已对准。
            "handoff_alignment_tolerance": 0.12,
            # Nav2 会把坑沿、墙脚等正确地标为不可通行，因此“入口导航失败”并不等于
            # “不能越障”。同一语义目标连续停滞、且仍位于正前方时，任务层应结束普通
            # 底盘规划，转入 TraverseObstacle；否则会永久重发同一个不可达入口点。
            "approach_stall_handoff_count": 1,
            # Nav2 may stop at the inflation boundary before the nominal 1.20 m
            # handoff.  A repeated stall may transfer the remaining approach to
            # TraverseObstacle, whose goal carries the measured entry distance.
            # 当前地形 ROI 到 2.5 m；在空间、横偏、航向和语义守卫同时成立时，允许
            # 越障控制器从 2.10 m 内接管最后一段低速接近。否则 Nav2 的膨胀边界可能
            # 恰好停在 1.9~2.0 m，使任务虽看清障碍却永远够不到名义入口。
            "approach_stall_handoff_max_distance": 2.35,
            "approach_stall_handoff_max_lateral": 0.75,
            "approach_stall_handoff_max_heading_error": 0.22,
            # 入口连续停滞却不满足越障交接门限时，短暂忽略同一 map 位置，先探索其他
            # 方向。否则误分类的场地边缘或远距离轮廓会被无限取消、立即重发。
            "obstacle_failure_cooldown": 12.0,
            "obstacle_failure_radius": 0.90,
            # 斜视场地外沿可能短暂拟合成坡；只允许离在线 SLAM 地图外框有足够余量的
            # 语义目标进入 Action。内部障碍稍后会随着建图扩展自动通过该门限。
            "obstacle_map_edge_margin": 0.05,
            "obstacle_lock_radius": 1.50,
            # The simulator and future hardware adapter may need tens of
            # seconds for a long bridge.  This is a safety ceiling, not the
            # nominal duration, and therefore must exceed every valid action.
            "goal_timeout": 45.0, "traversal_timeout": 45.0,
            # 到达入口但越障 Action 服务未就绪时不能永久停留；短暂等待后保留该障碍
            # 为未完成，换一个目标继续探索。真机服务应在启动自主任务前先就绪。
            "controller_wait_timeout": 5.0,
            # Geometry and image-topic are time-aligned by ROS header.  If safety
            # data is older than this window, we hold in verification instead
            # of triggering action handoff.
            "safety_geometry_stale_seconds": 0.35,
            "minimum_obstacle_confidence": 0.55, "obstacle_confirmation_frames": 3,
            # Only close, repeated semantic observations may arm traversal.
            # Far-field labels remain visible for diagnostics but cannot move
            # the robot through an unverified structure.
            "semantic_observation_distance": 2.50,
            "semantic_confirmation_distance": 1.20,
            # 名称输入已由感知层完成多帧稳定和几何校验；任务层只做第二级短窗口确认。
            # 2/6 能接住木桥偶发但连续的专名，又不接受单帧颜色/深度跳变。
            "semantic_confirmation_votes": 3,
            "semantic_recent_window": 6,
            # If metric geometry is stable but the competition name remains
            # ambiguous, rotate in place through small alternating viewpoints.
            # Translation stays locked by the safety gate during verification.
            "semantic_verification_turn_angle": 0.30,
            # 每个驻留点最多看左右两个附加视角；仍不明确就移动到新观察站，不在原地
            # 用四次旋转耗掉 15～25 秒的比赛预算。
            "semantic_verification_max_attempts": 2,
            "semantic_verification_lock_radius": 1.50,
            # Wait for several complete 5 Hz depth frames after a viewpoint
            # turn.  Without this guard, motion-smeared labels can trigger an
            # Action before the first stable cloud from the new view arrives.
            "semantic_post_turn_settle_time": 1.20,
            "completed_obstacle_radius": 0.65, "post_traversal_cooldown": 3.0,
            # Action success 只是底层声明，任务层还要用在线 TF 做后验验证：机体必须从
            # 入口侧前进到障碍前缘另一侧，并在落地后保持一段时间近似静止。
            "post_traversal_verification_timeout": 5.0,
            "post_traversal_minimum_displacement": 0.45,
            "post_traversal_beyond_margin": 0.12,
            "post_traversal_stable_duration": 0.75,
            "post_traversal_stable_translation": 0.06,
            "post_traversal_stable_rotation": 0.10,
            # 已经从多帧感知确认、但还没有成功越过的障碍进入任务账本。探索空闲时优先
            # 回到当时的安全观察位姿重新捕获它，而不是把目标直接放在实体障碍中心。
            "obstacle_revisit_position_tolerance": 0.30,
            "obstacle_revisit_cooldown": 8.0,
            # 同一观察位按 8/16/32/64 秒退避；到达位姿但未重新捕获也算一次尝试，
            # 期间优先处理其他障碍或前沿，防止一个旧记录耗尽整场比赛时间。
            "obstacle_revisit_max_cooldown": 64.0,
            # 没有前沿不等于比赛完成：先原地分段转向补扫，再重试曾被 Nav2 暂时拒绝
            # 的方向；只有八项任务全部完成或总任务超时才转向终点。
            "empty_frontier_confirmations": 4,
            # 无前沿、无覆盖目标、无可执行待办时最多补扫两圈；仍无新证据就携带当前
            # 已完成/未完成清单返回任务启动点，避免永远原地旋转。
            "maximum_search_turns": 8,
            "search_turn_angle": 1.570796,
            "nav_stall_timeout": 5.0,
            # 普通探索 5 秒不动就应切换目标；返程是唯一终点，必须给 Nav2 行为树足够
            # 时间触发 Spin/BackUp/重新规划，不能每 5 秒由任务层提前取消。
            "return_nav_stall_timeout": 20.0,
            # 运动进展优先看连续本地 odom，避免 SLAM 的 map 修正/发布延迟把真实低速旋转
            # 误判为卡死。odom 断流后退回 map TF，且导航健康节点仍会执行停车。
            "odom_progress_timeout": 0.5,
            # DWB may make genuine 0.02--0.04 m/s corrections near an inflation
            # gradient.  Require measurable odometry progress, but do not mistake
            # that low-speed motion for a complete five-second stall.
            "nav_progress_translation": 0.04,
            "nav_progress_rotation": 0.06,
            "inventory_log_period": 5.0,
            # 整场软件任务预算为 5 分钟。最后 60 秒只允许完成正在执行的越障或返回
            # 起点，不再发起新的探索；若返程受阻仍继续安全重试，而不是到点原地停车。
            "mission_timeout": 300.0,
            "return_time_reserve": 60.0,
            "return_home_tolerance": 0.40,
            "front_name_timeout": 1.2,
            "expected_obstacle_ids": list(COMPETITION_OBSTACLE_IDS),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.params = {name: self.get_parameter(name).value for name in defaults}
        validate_mission_parameters(self.params)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, map_qos)
        # Nav2's standard global costmap also contains the filtered depth/3-D point
        # cloud layer. It is therefore authoritative for selecting a recovery motion;
        # the SLAM map alone may not contain a low step seen below the 2-D lidar plane.
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._costmap_callback,
            map_qos,
        )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(
            TraversalGuidance,
            "/traversal/guidance",
            self._guidance_callback,
            10,
        )
        # 中文话题供人读，任务内部立即转换为上方稳定 ID。它仍来自实时几何/视觉，绝不
        # 读取 Gazebo 实体名；换成真机传感器后通信合同不变。
        self.create_subscription(
            String,
            "/perception/front_obstacle_name",
            self._front_name_callback,
            10,
        )
        self.create_subscription(
            NavigationSafety,
            "/terrain/navigation_safety",
            self._navigation_safety_callback,
            10,
        )
        self.state_pub = self.create_publisher(String, "/autonomy/state", 10)
        self.event_pub = self.create_publisher(String, "/autonomy/event", 10)
        self.progress_pub = self.create_publisher(String, "/autonomy/progress", 10)
        # 两份清单使用 transient-local，监控程序或队友的终端即使晚启动，也能立即看到
        # 最近任务状态。内容是带稳定英文 ID 和中文名称的 JSON，不参与运动控制。
        inventory_qos = QoSProfile(depth=1)
        inventory_qos.reliability = ReliabilityPolicy.RELIABLE
        inventory_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.completed_pub = self.create_publisher(
            String, "/autonomy/completed_obstacles", inventory_qos
        )
        self.pending_pub = self.create_publisher(
            String, "/autonomy/pending_obstacles", inventory_qos
        )
        # 默认终点就是任务启动点，保持三条启动命令即可完整运行。若正式规则指定不同
        # 终点，赛务/上位机可在运行前发布一个 map 坐标 PoseStamped 覆盖它；算法不需要
        # 读取 Gazebo world 或修改任何场地坐标。
        self.create_subscription(
            PoseStamped,
            "/autonomy/finish_pose",
            self._finish_pose_callback,
            inventory_qos,
        )
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
        # 非接近型 Nav2 目标需要先转离正前方障碍时，地形限速可能仍为零。任务节点只发布
        # 带心跳的“允许提取纯 yaw”布尔量；速度门会删除全部线速度并执行健康/雷达急停。
        # 这不是速度指令，Gazebo 和真机均继续使用同一标准 Nav2 -> /cmd_vel 链路。
        self.rotation_recovery_pub = self.create_publisher(
            Bool, "/navigation/rotation_recovery", 10
        )
        self.rotation_recovery_pub.publish(Bool(data=False))
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.traverse_client = ActionClient(self, TraverseObstacle, "/traverse_obstacle")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.enabled = bool(self.params["autostart"])
        self.state = "WAITING_FOR_INPUTS" if self.enabled else "IDLE"
        self.map_msg = None
        self.costmap_msg = None
        self.costmap_received = float("-inf")
        self.map_received = 0.0
        self.latest_odom = None
        self.odom_received = 0.0
        self.guidance = None
        self.guidance_received = 0.0
        self.last_safety = None
        self.safety_received = 0.0
        self.nav_handle = None
        self.nav_send_pending = False
        self.nav_cancel_pending = False
        # 取消原因与目标用途必须分开保存。旧实现把 ``approach`` 覆盖成
        # ``cancel_approach_within_tolerance``，result 回调因而不知道这是一次障碍入口
        # 导航，无法进入受保护的 Action 交接，只会反复提交同一个近距离目标。
        self.nav_cancel_reason = ""
        self.nav_purpose = ""
        self.nav_revisit_id = ""
        self.nav_started = 0.0
        self.nav_target = None
        self.nav_progress_pose = None
        self.nav_progress_time = 0.0
        # approach goal 与其来源障碍绑定；Nav2 成功后用于交叉验证，防止前缘/类别抖动
        # 让严格 READY 漏报，也防止把相邻障碍误交给越障控制器。
        self.nav_obstacle_position = None
        self.nav_obstacle_id = ""
        self.nav_retry_until = 0.0
        # 只累计“同一语义障碍的入口导航停滞”。前沿失败、类别改变、成功越障或人工
        # 停止都会清零，避免把互不相关的两次 Nav2 失败拼成一次越障许可。
        self.approach_stall_id = ""
        self.approach_stall_count = 0
        self.pending_traverse = None
        self.pending_traverse_id = ""
        # READY 时立即冻结障碍物的 map 坐标。Action 执行期间机器人会移动，如果完成后
        # 再用“当前位姿 + 旧相对距离”反算，会把已经越过的障碍错误登记到机器人前方。
        self.pending_traverse_position = None
        self.pending_traverse_robot_start = None
        self.pending_traverse_started = 0.0
        # send_goal_async 到 goal-response 之间还没有 goal handle，必须另设 pending 锁；
        # 否则 4 Hz 定时器会在服务器响应前重复提交同一个障碍。
        self.traverse_send_pending = False
        self.traverse_cancel_pending = False
        self.traverse_handle = None
        self.traverse_started = 0.0
        self.traversal_verification: Optional[TraversalVerification] = None
        self.obstacle_signature = None
        self.obstacle_frames = 0
        self.completed_obstacles = []
        # 每次成功 Action 的入口到出口轨迹；用于抑制长结构出口的重复识别。这里保存
        # 的全是实时 map 位姿，不含规则图或 Gazebo 实体坐标。
        self.completed_traversal_segments = []
        self.completed_semantics = []
        self.last_inventory_signature = None
        self.last_inventory_log_time = 0.0
        # semantic_id -> 最近一次可靠观察。比赛八项 ID 唯一，因此字典天然保证任务清单
        # 不会因同一障碍多帧观测而膨胀；真正完成仍只能由 Action success 写入。
        self.observed_obstacles: Dict[str, ObservedObstacle] = {}
        self.blocked_frontiers = []
        # 只记录本次自主任务实际经过的 map 位姿。当前沿消失后，覆盖目标会选择离这些
        # 轨迹最远的已知自由区，避免“雷达看完地图但相机没看完障碍”时原地转圈。
        self.coverage_visited = []
        # 元素为 (map_x, map_y, monotonic_expiry)。它只抑制本次运行中短暂不可达的感知
        # 入口，不写死比赛坐标；到期后自动允许从新视角重新确认。
        self.blocked_obstacles = []
        self.empty_frontier_count = 0
        self.controller_wait_reported = False
        self.cooldown_until = 0.0
        self.locked_obstacle_position = None
        self.locked_obstacle_id = ""
        # 单帧局部点云常只能看到踏板/边缘，接近过程中语义会逐步从“待确认”收敛到
        # 高墙、台阶或坑。保存同一空间目标最近若干帧的投票，不把第一帧猜测永久锁死。
        self.semantic_votes = deque(maxlen=16)
        # 语义投票从障碍第一次进入前向 ROI 就开始，而不是等 Nav2 已经锁定入口才开始。
        # 近距离墙面可能退化成 STEP；保留更早的 WALL 多帧证据可避免在交接前改名。
        self.semantic_vote_position = None
        # 语义换视角以机器人驻留点计数，而不是以会随相机朝向移动的最近障碍前缘计数。
        # 同时保存本轮看到的所有前缘，耗尽尝试后一起做短暂空间冷却。
        self.semantic_verification_position = None
        self.semantic_verification_obstacle_positions = []
        self.semantic_verification_attempts = 0
        self.semantic_settle_until = 0.0
        # 预对正完成后必须等待相机/点云看到新的机身朝向；否则定时器可能在旧 Guidance
        # 到期前再次提交同一个旋转目标，表现为机械狗连续过转。
        self.pre_alignment_settle_until = 0.0
        # Action 失败不能只依赖空间冷却后原地重试。保存一次性的交替转角，让 Nav2 在
        # 当前安全站位主动换视角；完成后等待新点云，再恢复通用探索。字典只记录每个
        # 在线语义的失败次数，不包含场地坐标或固定障碍顺序。
        self.failed_entry_turn_pending = 0.0
        self.failed_entry_escape_pending = 0.0
        # Most failed obstacle entries need a new *position* after the recovery
        # turn.  Return-to-finish is different: the target is already known and a
        # pure change of heading is enough for Nav2 to search another homotopy.  A
        # separate flag prevents return recovery from blindly walking 0.8 m farther
        # into the structure that blocked the original path.
        self.failed_entry_escape_after_turn = True
        self.failed_entry_failures: Dict[str, int] = {}
        self.failed_entries: List[FailedEntry] = []
        # 模糊结构耗尽换视角次数后，左右交替选择新观察站，避免每次都向
        # 同一侧绕行而最终靠近场地边界。这是调度状态，不包含障碍坐标。
        self.ambiguous_recovery_sign = 1.0
        self.front_obstacle_name = ""
        self.front_name_received = 0.0
        self.home_pose = None
        self.finish_pose = None
        self.returned_home = False
        self.return_attempts = 0
        self.search_turn_index = 0
        self.exploration_exhausted = False
        self.mission_started = time.monotonic()
        self.mission_ready_after = 0.0
        self.create_timer(0.25, self._tick)
        self._publish_state("mission node ready")

    def _publish_state(self, event=""):
        self.state_pub.publish(String(data=self.state))
        self._publish_inventory(force_publish=bool(event))
        if event:
            self.event_pub.publish(String(data=event))
            self.get_logger().info(f"Autonomy {self.state}: {event}")

    def _publish_inventory(self, force_publish=False):
        """Publish machine-readable lists and periodically print the human task ledger.

        The two transient-local topics remain the integration contract.  The terminal line is
        intentionally periodic as well as change-driven, so an operator does not need a second
        ``ros2 topic echo`` window to see completed and pending obstacles.
        """
        completed, pending = mission_inventory(
            self.params["expected_obstacle_ids"], self.completed_semantics
        )
        now = time.monotonic()
        signature = (completed, pending)
        changed = signature != self.last_inventory_signature
        periodic = (
            now - self.last_inventory_log_time
            >= float(self.params["inventory_log_period"])
        )
        if not (force_publish or changed or periodic):
            return
        self.completed_pub.publish(String(data=inventory_message(completed)))
        self.pending_pub.publish(String(data=inventory_message(pending)))
        progress = (
            f"state={self.state}; completed={len(completed)}/"
            f"{len(completed) + len(pending)}; "
            f"score={mission_score(self.completed_semantics, self.returned_home)}; "
            f"elapsed_seconds={max(0.0, now - self.mission_started):.1f}; "
            "budget_remaining_seconds="
            f"{max(0.0, float(self.params['mission_timeout']) - (now - self.mission_started)):.1f}; "
            f"completed_ids={','.join(completed) or 'none'}; "
            f"pending_ids={','.join(pending) or 'none'}"
        )
        self.progress_pub.publish(String(data=progress))
        if changed or periodic:
            elapsed = max(0.0, now - self.mission_started)
            remaining = max(0.0, float(self.params["mission_timeout"]) - elapsed)
            self.get_logger().info(
                inventory_display(completed, pending)
                + f" | 用时={elapsed:.0f}s | 剩余预算={remaining:.0f}s"
            )
            self.last_inventory_signature = signature
            self.last_inventory_log_time = now

    def _publish_immediate_stop(self) -> None:
        """锁住核心速度门；用于独立 launch 退出时确定性停车。"""
        self.rotation_recovery_pub.publish(Bool(data=False))
        self.autonomy_stop_pub.publish(Bool(data=True))

    def _map_callback(self, msg):
        self.map_msg = msg
        self.map_received = time.monotonic()

    def _costmap_callback(self, msg: OccupancyGrid) -> None:
        """Cache the standard Nav2 costmap used only for bounded recovery goals."""
        self.costmap_msg = msg
        self.costmap_received = time.monotonic()

    def _odom_callback(self, msg: Odometry) -> None:
        """Cache local motion only; map coordinates remain authoritative for tasks."""
        self.latest_odom = msg
        self.odom_received = time.monotonic()

    def _motion_pose(self, now: Optional[float] = None):
        """Return a continuous local pose for progress watchdog decisions.

        SLAM may deliberately delay or discretely correct ``map->odom`` during a slow
        turn.  Such corrections are desirable for mapping but unsuitable for asking
        whether wheels/legs moved in the last five seconds. Fresh odometry is used only
        for this watchdog; obstacle positions, goals and crossing proof remain in map.
        """
        current = time.monotonic() if now is None else float(now)
        if (
            self.latest_odom is not None
            and current - self.odom_received
            <= float(self.params["odom_progress_timeout"])
        ):
            pose = self.latest_odom.pose.pose
            q = pose.orientation
            yaw = atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return float(pose.position.x), float(pose.position.y), float(yaw)
        return self._robot_pose()

    def _finish_pose_callback(self, msg: PoseStamped) -> None:
        """Accept an optional map-frame finish pose without coupling to a venue.

        The fallback terminal pose is the live start pose.  A separately deployed
        referee/mission package may override it once official coordinates are known.
        Invalid frames or non-finite values are ignored, because accepting them would
        turn a reporting convenience into an unsafe navigation goal.
        """
        if str(msg.header.frame_id).lstrip("/") != "map":
            self.get_logger().warning(
                "Ignoring /autonomy/finish_pose: frame_id must be map"
            )
            return
        position = msg.pose.position
        orientation = msg.pose.orientation
        values = (
            position.x,
            position.y,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(isfinite(float(value)) for value in values):
            self.get_logger().warning(
                "Ignoring /autonomy/finish_pose: pose contains NaN/Inf"
            )
            return
        norm = sum(float(value) ** 2 for value in values[2:])
        if norm < 1e-6:
            self.get_logger().warning(
                "Ignoring /autonomy/finish_pose: quaternion is invalid"
            )
            return
        # Pose producers should send a unit quaternion, but normalising here prevents a
        # slightly scaled external value from changing the requested finish yaw.
        scale = norm ** -0.5
        qx, qy = float(orientation.x) * scale, float(orientation.y) * scale
        qz, qw = float(orientation.z) * scale, float(orientation.w) * scale
        yaw = atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self.finish_pose = (float(position.x), float(position.y), float(yaw))
        self._publish_state(
            f"finish pose updated to ({position.x:.2f}, {position.y:.2f})"
        )

    def _front_name_callback(self, msg: String) -> None:
        """缓存与最新感知同源的比赛名称；超时后绝不沿用上一处障碍。"""
        self.front_obstacle_name = str(msg.data)
        self.front_name_received = time.monotonic()

    def _navigation_safety_callback(self, msg: NavigationSafety) -> None:
        """缓存最新几何，包括明确的无效帧。

        无效消息说明上一帧米制观测已经不可继续使用；若直接忽略，旧高度/宽度会在
        ``safety_geometry_stale_seconds`` 内继续存活，传感器刚断流时仍可能授权交接。
        因此无效帧也覆盖缓存，让最终几何闸门立即 fail-closed。
        """
        self.last_safety = msg
        self.safety_received = time.monotonic()

    def _latest_navigation_safety(self, now: float) -> Optional[NavigationSafety]:
        """返回仍可用于闸门判断的最新安全观测。"""
        if (
            self.last_safety is None
            or not self.last_safety.perception_valid
            or now - self.safety_received
            > float(self.params["safety_geometry_stale_seconds"])
        ):
            return None
        return self.last_safety

    def _arena_boundary_ahead(self, now: float) -> bool:
        """判断当前新鲜名称是否明确禁止越过场地边界。"""
        return bool(
            now - self.front_name_received
            <= float(self.params["front_name_timeout"])
            and "场地边界" in self.front_obstacle_name
        )

    def _current_obstacle_id(self, now: Optional[float] = None) -> str:
        """返回新鲜的比赛语义 ID，防止旧名称被带入下一次 Action。"""
        stamp = time.monotonic() if now is None else float(now)
        if stamp - self.front_name_received > float(self.params["front_name_timeout"]):
            return ""
        return canonical_obstacle_id(self.front_obstacle_name)

    def _remember_obstacle(
        self,
        semantic_id: str,
        position: Tuple[float, float],
        confidence: float,
        now: float,
    ) -> None:
        """Update the active-search record for one repeatedly confirmed obstacle.

        The observation pose is at the robot, not at the obstacle.  Repeated frames use
        a low-pass update for obstacle position but retain the newest safe viewpoint,
        allowing later exploration to deliberately reacquire an unfinished task while
        Nav2 continues treating the physical structure as occupied space.
        """
        if (
            not is_actionable_semantic_id(semantic_id)
            or semantic_id in self.completed_semantics
            or not isfinite(float(confidence))
        ):
            return
        robot = self._robot_pose()
        if robot is None:
            return
        previous = self.observed_obstacles.get(semantic_id)
        obstacle_x, obstacle_y = float(position[0]), float(position[1])
        retry_after = 0.0
        revisit_failures = 0
        if previous is not None:
            # 相同比赛 ID 理论上只出现一次。用低通抑制深度边缘抖动；若位置发生大跳变，
            # 保留新观察但不平均两处，避免假阳性把回访点落在它们中间。
            if hypot(
                obstacle_x - previous.obstacle_x,
                obstacle_y - previous.obstacle_y,
            ) <= float(self.params["obstacle_lock_radius"]):
                obstacle_x = 0.8 * previous.obstacle_x + 0.2 * obstacle_x
                obstacle_y = 0.8 * previous.obstacle_y + 0.2 * obstacle_y
            retry_after = previous.retry_after
            # 感知再次看到目标或 Nav2 到达旧位姿都不代表重新识别/越障成功，因此保留
            # 调度尝试次数，避免原地连续图像帧绕过指数退避。
            revisit_failures = previous.revisit_failures
        self.observed_obstacles[semantic_id] = ObservedObstacle(
            semantic_id=semantic_id,
            obstacle_x=obstacle_x,
            obstacle_y=obstacle_y,
            view_x=float(robot[0]),
            view_y=float(robot[1]),
            view_yaw=float(robot[2]),
            confidence=max(0.0, min(1.0, float(confidence))),
            last_seen=float(now),
            retry_after=retry_after,
            revisit_failures=revisit_failures,
        )

    def _defer_obstacle_revisit(self, semantic_id: str, now: float) -> None:
        """Bound retries for one known target without blocking other exploration."""
        record = self.observed_obstacles.get(str(semantic_id))
        if record is not None:
            record.retry_after = float(now) + float(
                self.params["obstacle_revisit_cooldown"]
            )

    def _penalize_obstacle_revisit(self, semantic_id: str, now: float) -> None:
        """Back off one repeatedly unreachable viewpoint while exploring elsewhere."""
        record = self.observed_obstacles.get(str(semantic_id))
        if record is None:
            return
        record.revisit_failures += 1
        delay = obstacle_revisit_delay(
            self.params["obstacle_revisit_cooldown"],
            record.revisit_failures,
            self.params["obstacle_revisit_max_cooldown"],
        )
        record.retry_after = float(now) + delay

    def _known_semantic_at_guidance(self, msg) -> str:
        """Recover a previously confirmed name for the same physical entry.

        Close to an obstacle, the depth camera may only see a tread or one post and the
        live classifier can legitimately fall back to a generic STEP/WALL/BAR label.
        The mission ledger already stores the map position of every uniquely confirmed
        competition obstacle.  Reusing that identity is safe only when all three guards
        hold: the record is still pending, its map position is close to the current
        geometry, and the current coarse Action type is compatible with that identity.

        This is deliberately independent of Gazebo model names and rule-map coordinates;
        on hardware the record is created from the same online camera/point-cloud data.
        """
        position = self._obstacle_position(msg)
        if position is None:
            return ""
        matched = matching_pending_semantic(
            tuple(self.observed_obstacles.values()),
            self.completed_semantics,
            position,
            action_obstacle_type(msg),
            float(self.params["handoff_fallback_spatial_tolerance"]),
            current_time=time.monotonic(),
        )
        if matched:
            return matched
        robot = self._robot_pose()
        if robot is None:
            return ""
        return matching_pending_semantic_from_viewpoint(
            tuple(self.observed_obstacles.values()),
            self.completed_semantics,
            robot,
            action_obstacle_type(msg),
            float(self.params["handoff_fallback_viewpoint_tolerance"]),
            float(self.params["handoff_fallback_view_heading_tolerance"]),
            current_time=time.monotonic(),
        )

    def _action_semantic_id(self, msg, fallback: str = "") -> str:
        """Resolve the live name, then safely reuse the pending spatial ledger.

        At long range the camera can see an entire pit/bridge/stair structure, while
        at the handoff boundary it may see only one tread and publish a generic STEP.
        Throwing away the earlier identity caused the mission to rotate four times,
        revisit the same point, and repeat indefinitely.  The fallback below still
        requires a live valid Guidance, a compatible coarse Action type, and a map
        position within ``handoff_fallback_spatial_tolerance``; it never authorises
        traversal from a stale name alone or from a Gazebo entity.
        """
        resolved = self._resolved_obstacle_id(msg, fallback)
        if is_actionable_semantic_id(resolved):
            return resolved
        known = self._known_semantic_at_guidance(msg)
        # A nearby ledger record is only an identity hint, never an independent
        # traversal permit.  The live metric geometry must still fit that exact
        # candidate.  Without this final check, a transient WALL record could win
        # the distance tie while current point-cloud evidence consistently described
        # a T stair, causing the simulation backend (and potentially a real motion
        # controller) to execute the wrong traversal length.
        return (
            known
            if is_actionable_semantic_id(known)
            and self._geometry_supports_obstacle_id(known)
            else ""
        )

    def _geometry_supports_obstacle_id(
        self,
        candidate_id: str,
        now: Optional[float] = None,
    ) -> bool:
        """只让与语义候选一致的新鲜米制几何通过。

        ``front_obstacle_name`` 和 ``TraversalGuidance`` 可用于任务调度，但都不含最终
        Action 所需的完整高度、宽度和深度。NavigationSafety 缺失、过期或明确无效时
        必须拒绝交接，不能把“没有证据”解释成“几何匹配”；后续新鲜有效帧可自然恢复，
        不设置永久故障锁。
        """
        if not candidate_id:
            return False
        safety = self._latest_navigation_safety(
            time.monotonic() if now is None else float(now)
        )
        return bool(
            safety is not None
            and obstacle_geometry_fits_candidate(candidate_id, safety)
        )

    def _voted_obstacle_id(self, fallback: str = "") -> str:
        """返回同一入口最近语义的多数票，票数相同时优先较新的结论。

        这只融合在线感知名称，不读取 world 模型名或坐标。至少两帧一致才覆盖 fallback，
        避免一帧错误类别改变比赛任务；若始终无法细分则保留通用几何 ID/空值。
        """
        return select_semantic_vote(self.semantic_votes, fallback)

    def _resolved_obstacle_id(self, msg, fallback: str = "") -> str:
        """返回经 Action 几何合同校验后的比赛语义 ID。"""
        if not msg.perception_valid:
            return ""
        # 同一 map 入口中至少两帧已经确认的锁定语义优先于最后一帧粗几何。相机进入
        # 近裁剪区、坡顶或墙后时局部类别会自然退化；空间锁仍由 _matches_obstacle_lock
        # 约束，不会把上一障碍带到下一处。
        stable_lock = canonical_obstacle_id(self.locked_obstacle_id)
        # 一旦同一 map 入口在接近阶段以不少于三帧锁定为唯一比赛语义，近裁剪、坡顶
        # 或侧面视角产生的 STEP/WALL 粗分类不得改写它。空间一致性仍由入口锁约束；
        # Action 前还会再次检查实时距离、横偏和航向，因此这不是盲信旧名称。
        recent_window = int(self.params["semantic_recent_window"])
        minimum_votes = int(self.params["semantic_confirmation_votes"])
        recent_values = list(self.semantic_votes)[-recent_window:]
        replacement = replacement_semantic_vote(
            recent_values,
            stable_lock,
            minimum_votes=minimum_votes,
            recent_window=recent_window,
        )
        if is_actionable_semantic_id(stable_lock):
            # A sustained conflicting name means the view has materially changed.
            # Use it only if both the current coarse Action type and the latest
            # NavigationSafety metrics agree; otherwise return ambiguous so the
            # bounded viewpoint-verification state rotates instead of traversing.
            if replacement:
                resolved_replacement = semantic_id_for_action(
                    replacement, action_obstacle_type(msg)
                )
                if (
                    resolved_replacement == replacement
                    and self._geometry_supports_obstacle_id(replacement)
                ):
                    return replacement
                return ""
            if not self._geometry_supports_obstacle_id(stable_lock):
                return ""
            return stable_lock
        planar_lock = dominant_planar_vote(
            self.semantic_votes, minimum_votes=minimum_votes
        )
        if planar_lock:
            # If the current coarse geometry is still compatible, keep the
            # measured ramp identity.  If a close side crop has become STEP,
            # deliberately return ambiguous and invoke the bounded viewpoint
            # verification state instead of authorising a diagonal traversal.
            compatible = semantic_id_for_action(planar_lock, action_obstacle_type(msg))
            if not compatible:
                return ""
            if self._geometry_supports_obstacle_id(compatible):
                return compatible
            return ""
        recent_vote = select_semantic_vote(recent_values, "")
        # 两帧一致的最新结构证据可替换远场锁；单帧仍不能覆盖。这里先做 Action 几何
        # 兼容检查，防止不同障碍的异步名称恰好进入最近窗口。
        if (
            recent_vote
            and recent_vote != stable_lock
            and semantic_vote_is_confirmed(
                recent_values,
                recent_vote,
                minimum_votes=minimum_votes,
                recent_window=recent_window,
            )
        ):
            resolved_recent = semantic_id_for_action(
                recent_vote, action_obstacle_type(msg)
            )
            if resolved_recent and self._geometry_supports_obstacle_id(resolved_recent):
                return resolved_recent
        if stable_lock and semantic_vote_is_confirmed(
            self.semantic_votes,
            stable_lock,
            minimum_votes=minimum_votes,
            recent_window=recent_window,
        ):
            if self._geometry_supports_obstacle_id(stable_lock):
                return stable_lock
            return ""
        voted = self._voted_obstacle_id(
            fallback or self._current_obstacle_id()
        )
        resolved = semantic_id_for_action(voted, action_obstacle_type(msg))
        if not semantic_vote_is_confirmed(
            self.semantic_votes,
            resolved,
            minimum_votes=minimum_votes,
            recent_window=recent_window,
        ):
            return ""
        if not self._geometry_supports_obstacle_id(resolved):
            return ""
        return resolved

    def _reset_obstacle_lock(self) -> None:
        """成对释放入口位置、语义和投票，防止上一障碍污染下一障碍。"""
        self.locked_obstacle_position = None
        self.locked_obstacle_id = ""
        self.semantic_votes.clear()
        self.semantic_vote_position = None
        self.semantic_verification_position = None
        self.semantic_verification_obstacle_positions.clear()
        self.semantic_verification_attempts = 0
        self.semantic_settle_until = 0.0

    def _all_obstacles_complete(self) -> bool:
        expected = set(str(item) for item in self.params["expected_obstacle_ids"])
        return bool(expected) and expected.issubset(set(self.completed_semantics))

    def _complete_at_finish_if_arrived(self, robot_pose) -> bool:
        """Commit the terminal state once localization proves arrival at finish.

        This deliberately accepts arrival independently of the last Nav2 result.
        It is useful on real hardware too: a planner/controller can abort because
        the last pose is occupied or outside its rolling window while the physical
        robot is already inside the allowed finish radius.
        """
        terminal_pose = self.finish_pose or self.home_pose
        if not terminal_pose_reached(
            robot_pose,
            terminal_pose,
            float(self.params["return_home_tolerance"]),
        ):
            return False
        self.returned_home = True
        self.enabled = False
        self.state = "COMPLETED"
        self._publish_state(
            "reached mission finish; mission complete; "
            f"score={mission_score(self.completed_semantics, True)}"
        )
        return True

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
        # SetEntityPose 仿真替身和未来真机越障控制器都会在 Action 期间产生大姿态、近
        # 裁剪和结构侧视。那些帧仍发布给监控/UI，但绝不能成为“下一关”的任务语义。
        # 成功回调会清空当前锁；落地冷却结束后再从静稳的新鲜帧重新累计。否则主坡
        # 出口旁的 T 台立面曾被四帧暂态误锁成高墙，并进一步破坏整场任务顺序。
        now = time.monotonic()
        if (
            self.traverse_handle is not None
            or self.traverse_send_pending
            or self.pending_traverse is not None
            or self.traversal_verification is not None
            or now < self.cooldown_until
        ):
            self.obstacle_signature, self.obstacle_frames = None, 0
            return
        # 名称和点云 Guidance 来自不同节点，DDS 调度可能相差一帧。先用当前 Guidance
        # 的几何类型校验名称，再参与同一入口投票，避免把“上一障碍名称”锁到新目标。
        current_id = semantic_id_for_action(
            self._current_obstacle_id(), action_obstacle_type(msg)
        )
        if current_id and not self._geometry_supports_obstacle_id(
            current_id,
            now=now,
        ):
            return
        # 在正式选择 approach goal 以前就按 map 空间聚合语义。高墙接近时常先获得数帧
        # 唯一的 WALL 证据，贴近后立面因裁剪退化为 STEP；旧实现此时才清空并开始投票，
        # 最终会把高墙误记为 T 台。只有目标中心跨出 lock radius 才开启一组新投票。
        if (
            current_id
            and float(msg.distance)
            <= float(self.params["semantic_observation_distance"])
        ):
            if (
                self.semantic_vote_position is None
                or hypot(
                    position[0] - self.semantic_vote_position[0],
                    position[1] - self.semantic_vote_position[1],
                ) > float(self.params["obstacle_lock_radius"])
            ):
                self.semantic_votes.clear()
                self.semantic_vote_position = position
            else:
                self.semantic_vote_position = (
                    0.85 * self.semantic_vote_position[0] + 0.15 * position[0],
                    0.85 * self.semantic_vote_position[1] + 0.15 * position[1],
                )
            self.semantic_votes.append(current_id)
        if (
            current_id
            and float(msg.distance)
            <= float(self.params["semantic_confirmation_distance"])
            and self.locked_obstacle_position is not None
            and hypot(
                position[0] - self.locked_obstacle_position[0],
                position[1] - self.locked_obstacle_position[1],
            ) <= float(self.params["obstacle_lock_radius"])
        ):
            # 已唯一锁定的比赛语义在同一入口内保持不变。比如主坡靠近后，侧面轮廓
            # 会短暂成为 STEP，若每帧重投票就会被误计为 T 台并使用错误跨越长度；但
            # 持续的新证据也不能被永久锁忽略。至少三个最近投票且米制几何相符时才
            # 替换，随后 Action 交接仍会重复执行同一几何校验。
            replacement = replacement_semantic_vote(
                self.semantic_votes,
                self.locked_obstacle_id,
                minimum_votes=int(self.params["semantic_confirmation_votes"]),
                recent_window=int(self.params["semantic_recent_window"]),
            )
            if (
                replacement
                and self._geometry_supports_obstacle_id(replacement, now=now)
            ):
                self.locked_obstacle_id = replacement
            elif not is_actionable_semantic_id(self.locked_obstacle_id):
                candidate = self._voted_obstacle_id(self.locked_obstacle_id)
                if semantic_vote_is_confirmed(
                    self.semantic_votes,
                    candidate,
                    minimum_votes=int(self.params["semantic_confirmation_votes"]),
                    recent_window=max(
                        int(self.params["semantic_recent_window"]), 8
                    ),
                ):
                    self.locked_obstacle_id = candidate
            self.locked_obstacle_position = (
                0.85 * self.locked_obstacle_position[0] + 0.15 * position[0],
                0.85 * self.locked_obstacle_position[1] + 0.15 * position[1],
            )
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

        # 任务账本只登记通过同一套 Action 语义门的多帧结果。单帧 OpenCV 名称或粗点云
        # 类别仍可在终端显示，但不能变成主动回访目标；否则一个假阳性会让任务反复前往
        # 错误位置。登记不代表完成，只有 TraverseObstacle success 才能移入完成清单。
        if self.obstacle_frames >= int(self.params["obstacle_confirmation_frames"]):
            remembered_id = self._resolved_obstacle_id(msg, current_id)
            if is_actionable_semantic_id(remembered_id):
                self._remember_obstacle(
                    remembered_id,
                    position,
                    float(msg.confidence),
                    now,
                )

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
        candidate_id = semantic_id_for_action(
            self._current_obstacle_id(), obstacle_type
        )
        # 比赛障碍可能紧邻布置。长坡/桥的入口到出口使用线段去重，但必须同时匹配
        # 比赛语义；不能因 T 台刚好位于主坡出口附近，就被另一条已完成线段吞掉。
        if any(
            traversal_segment_matches(
                candidate_id,
                completed_id,
                position,
                start,
                end,
                radius,
            )
            for completed_id, start, end in self.completed_traversal_segments
        ):
            return True
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
        if (
            msg is None
            or time.monotonic() - self.guidance_received
            > float(self.params["guidance_timeout"])
            or not msg.perception_valid
            or not msg.traversal_required
            or not isfinite(float(msg.confidence))
            or msg.confidence < float(self.params["minimum_obstacle_confidence"])
            or self.obstacle_frames
            < int(self.params["obstacle_confirmation_frames"])
            or self._already_completed(msg)
        ):
            return None
        position = self._obstacle_position(msg)
        now = time.monotonic()
        self.blocked_obstacles = [
            item for item in self.blocked_obstacles if item[2] > now
        ]
        if position is None or any(
            hypot(position[0] - x, position[1] - y)
            <= float(self.params["obstacle_failure_radius"])
            for x, y, _expiry in self.blocked_obstacles
        ):
            return None
        semantic_id = self._resolved_obstacle_id(msg)
        # A failed revisit or rejected TraverseObstacle must also cool the live
        # perception path.  Otherwise a camera that keeps seeing the same structure
        # bypasses ``choose_pending_obstacle`` and immediately recreates the failed
        # handoff.  The delay is bounded at 64 s, during which other targets remain
        # eligible; this is scheduling only and never marks the obstacle completed.
        action_id = self._action_semantic_id(msg)
        action_record = self.observed_obstacles.get(action_id)
        if action_record is not None and action_record.retry_after > now:
            return None
        # Explicit arena-boundary evidence is never an obstacle-verification
        # target.  Other stable metric geometry may remain semantically
        # ambiguous; the task layer is allowed to approach/rotate for a better
        # view, but every Action hand-off below still requires a confirmed ID.
        if self._arena_boundary_ahead(now):
            return None
        edge_distance = distance_inside_grid_edge(
            self.map_msg, position[0], position[1]
        )
        if not map_edge_allows_obstacle_handoff(
            action_id,
            action_obstacle_type(msg),
            edge_distance,
            self.params["obstacle_map_edge_margin"],
        ):
            return None
        # 唯一障碍一旦成功就不再重复触发，即使局部点云把障碍中心更新到去重半径之外。
        # 木桥 unknown 例外：规则确有两座木桥，仍由空间去重和 resolve 函数补齐 A/B。
        if semantic_task_is_complete(semantic_id, self.completed_semantics):
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

    def _verify_ambiguous_obstacle(self, guidance, now: float) -> None:
        """Observe an unclassified obstacle from bounded alternate headings.

        This state deliberately commands rotation only.  The safety gate keeps
        translation at zero near a STEP/PIT/WALL/BAR, while Nav2 turns the body
        enough for the depth camera to expose wall thickness, stair levels or
        bridge gaps.  After a bounded number of unsuccessful views the entry is
        temporarily excluded so the mission can continue elsewhere.
        """
        robot = self._robot_pose()
        position = self._obstacle_position(guidance)
        if robot is None or position is None:
            self.state = "WAITING_FOR_INPUTS"
            return
        # 换视角时长坡、桥面和坑沿的“最近前缘”会沿结构移动。这里必须用几乎不动的
        # 机器人驻留点判断是否仍是同一轮验证；若用障碍前缘，即使半径放到 1.5 m 也会
        # 在同一长结构上反复重置为 1/4。
        lock_radius = float(self.params["semantic_verification_lock_radius"])
        robot_xy = (float(robot[0]), float(robot[1]))
        if not verification_station_matches(
            self.semantic_verification_position,
            robot_xy,
            lock_radius,
        ):
            self.semantic_verification_position = robot_xy
            self.semantic_verification_obstacle_positions.clear()
            self.semantic_verification_attempts = 0
        if not any(
            hypot(position[0] - previous[0], position[1] - previous[1])
            <= float(self.params["obstacle_failure_radius"])
            for previous in self.semantic_verification_obstacle_positions
        ):
            self.semantic_verification_obstacle_positions.append(position)
        self.semantic_verification_attempts += 1
        maximum = int(self.params["semantic_verification_max_attempts"])
        if self.semantic_verification_attempts > maximum:
            expiry = now + float(self.params["obstacle_failure_cooldown"])
            # 冷却本轮各个朝向看到的前缘，而不是只冷却最后一个像素投影点。这样长桥/坡
            # 转身后不会立刻从另一个前缘绕过 failure_radius 再进入 1/4。
            for blocked_position in self.semantic_verification_obstacle_positions:
                self.blocked_obstacles.append((
                    float(blocked_position[0]),
                    float(blocked_position[1]),
                    expiry,
                ))
            # 仅屏蔽障碍像素不会改变相机视角；现场回归中机器人因此在主坡
            # 长侧反复看到 0.13 m 边缘。排队一次左右交替的 90° 原地转向；
            # 转向成功后 entry_recovery 回调再排队 0.8 m 普通 Nav2 平移。
            # 平移不获得零限速旁路，仍要通过地形、雷达和导航健康门。
            self.failed_entry_escape_after_turn = True
            self.failed_entry_turn_pending = (
                self.ambiguous_recovery_sign
                * float(self.params["failed_entry_turn_angle"])
            )
            self.ambiguous_recovery_sign *= -1.0
            self._reset_obstacle_lock()
            self.cooldown_until = now + float(self.params["nav_failure_retry_delay"])
            self.state = "RECOVERY"
            self._publish_state(
                "obstacle remained ambiguous after bounded view changes; "
                "changing heading and observation station"
            )
            return
        # +a, -2a, +3a, -4a samples both sides without accumulating yaw in
        # one direction.  The largest change is still below one radian.
        signed_step = (
            self.semantic_verification_attempts
            if self.semantic_verification_attempts % 2
            else -self.semantic_verification_attempts
        )
        delta = signed_step * float(
            self.params["semantic_verification_turn_angle"]
        )
        self.state = "VERIFYING_OBSTACLE"
        self._send_nav_goal(
            self._make_pose(robot[0], robot[1], robot[2] + delta),
            "verify_obstacle",
        )
        self._publish_state(
            "metric obstacle confirmed but name is ambiguous; "
            f"changing view {self.semantic_verification_attempts}/{maximum}"
        )

    def _queue_traversal_handoff(self, target, semantic_id: str, position, now: float) -> bool:
        """冻结已验证入口并切换控制权，避免多个分支各自拼装 Action 状态。

        该函数只准备任务层状态；真正的腿部动作仍由 ``TraverseObstacle`` 服务端负责。
        Gazebo 替身与未来真机控制器因此使用完全相同的入口合同。
        """
        if not is_actionable_semantic_id(semantic_id) or position is None:
            return False
        # A named rule obstacle exists once.  Re-check at the last possible moment so
        # a completed pit/wall cannot be executed again after a near-field semantic
        # change while the previous Nav2 goal was being cancelled.  Unresolved bridge
        # identities remain eligible only until two spatially distinct bridge
        # traversals have resolved the A/B pair.
        if semantic_task_is_complete(semantic_id, self.completed_semantics):
            self.blocked_obstacles.append((
                float(position[0]),
                float(position[1]),
                now + float(self.params["obstacle_revisit_max_cooldown"]),
            ))
            self._reset_obstacle_lock()
            self.cooldown_until = now + float(self.params["nav_failure_retry_delay"])
            self.state = "RECOVERY"
            self._publish_state(
                f"suppressing already completed semantic={semantic_id}; "
                "selecting another target"
            )
            return False
        self.failed_entries = [
            record for record in self.failed_entries if record.expires > float(now)
        ]
        robot = self._robot_pose()
        if robot is not None and failed_entry_matches(
            self.failed_entries,
            semantic_id,
            robot,
            now,
            float(self.params["failed_entry_station_tolerance"]),
            float(self.params["failed_entry_heading_tolerance"]),
            require_new_station=semantic_id in LONG_TRAVERSAL_IDS,
        ):
            # The live classifier and alignment may pull the body back to exactly the
            # side-on heading that the controller just rejected.  Suppress it before
            # sending another Action, then use the next alternating recovery view.
            failures = self.failed_entry_failures.get(semantic_id, 1) + 1
            self.failed_entry_failures[semantic_id] = failures
            turn_sign = 1.0 if failures % 2 else -1.0
            self.failed_entry_escape_after_turn = True
            self.failed_entry_turn_pending = turn_sign * float(
                self.params["failed_entry_turn_angle"]
            )
            self.blocked_obstacles.append((
                float(position[0]),
                float(position[1]),
                now + float(self.params["obstacle_failure_cooldown"]),
            ))
            self._reset_obstacle_lock()
            self.cooldown_until = now + float(self.params["nav_failure_retry_delay"])
            self.state = "RECOVERY"
            self._publish_state(
                f"suppressing repeated failed entry={semantic_id}; "
                "changing station/heading before retry"
            )
            return False
        # 一个长桥/坡的出口在局部点云中可能再次表现成“新踏板”。目标最初以 unknown
        # 进入 approach、临近交接才恢复成已完成专名时，_fresh_target 的早期去重尚未
        # 看到这个 ID。因此 Action 前必须再用“同语义 + 已通过轨迹段”复核一次，避免
        # 把桥 B 出口当成第二座桥并再次把仿真/真机送向场地边缘。
        if any(
            traversal_segment_matches(
                semantic_id,
                completed_id,
                position,
                start,
                end,
                float(self.params["completed_obstacle_radius"]),
            )
            for completed_id, start, end in self.completed_traversal_segments
        ):
            self.blocked_obstacles.append((
                float(position[0]),
                float(position[1]),
                now + float(self.params["obstacle_revisit_max_cooldown"]),
            ))
            self._reset_obstacle_lock()
            self.cooldown_until = now + float(self.params["nav_failure_retry_delay"])
            self.state = "RECOVERY"
            self._publish_state(
                f"suppressing completed obstacle exit={semantic_id}; selecting another target"
            )
            return False
        if not self.traverse_client.server_is_ready():
            self._hold_for_traversal_controller(target, position, now)
            return True
        self.pending_traverse = target
        self.pending_traverse_id = semantic_id
        self.pending_traverse_position = position
        self.pending_traverse_started = now
        self.state = "HANDOFF"
        return True

    def _cancel_nav(self, reason="replace"):
        """只发送一次取消请求，并等待 Nav2 result 回调释放句柄。

        Action 的 cancel future 只说明服务收到请求；真正可以提交新目标的时刻是旧 goal
        result 到达以后。显式锁可避免入口目标与仍在执行的前沿目标互相抢占。
        """
        if self.nav_handle is not None and not self.nav_cancel_pending:
            self.nav_cancel_pending = True
            self.nav_cancel_reason = str(reason)
            self.nav_handle.cancel_goal_async()

    def _send_nav_goal(self, pose, purpose, guidance=None, revisit_id=""):
        """Submit one Nav2 goal and bind all callback context before async send.

        ``revisit_id`` is reporting/scheduling metadata only.  Nav2 still receives a
        standard map-frame PoseStamped, so neither the planner nor a future base driver
        needs to understand competition obstacle names.
        """
        if (
            self.nav_handle is not None
            or self.nav_send_pending
            or not self.nav_client.server_is_ready()
        ):
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_send_pending, self.nav_purpose = True, purpose
        self.nav_revisit_id = str(revisit_id) if purpose == "revisit_obstacle" else ""
        self.nav_cancel_reason = ""
        self.nav_target = (pose.pose.position.x, pose.pose.position.y)
        self.nav_progress_pose = self._motion_pose()
        self.nav_progress_time = time.monotonic()
        if purpose == "approach" and guidance is not None:
            self.nav_obstacle_position = self._obstacle_position(guidance)
            # 冻结“创建接近目标时”已经通过多帧和几何校验的语义。接近碰撞膨胀边界后，
            # 局部点云常在 BAR/WALL/STEP 间变化；result 回调只能在空间和粗类型仍一致时
            # 使用该 ID，不能把它无条件套给相邻结构。
            locked_id = canonical_obstacle_id(self.locked_obstacle_id)
            resolved_id = (
                locked_id
                if is_actionable_semantic_id(locked_id)
                else self._action_semantic_id(
                    guidance,
                    self._current_obstacle_id(),
                )
            )
            # A pending ledger record bridges a short near-field semantic dropout.  It
            # never bypasses the live spatial/type/distance/alignment gates in
            # ``_nav_result``; it only preserves which obstacle those gates refer to.
            self.nav_obstacle_id = resolved_id
        elif purpose != "approach":
            self.nav_obstacle_position = None
            self.nav_obstacle_id = ""
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        self.nav_send_pending = False
        try:
            handle = future.result()
            response_error = ""
        except Exception as exc:  # DDS/server loss may complete the future exceptionally.
            handle = None
            response_error = f": {exc}"
            self.get_logger().error(f"Nav2 goal response failed{response_error}")
        if handle is None or not handle.accepted:
            # Action server 在 Nav2 lifecycle 激活之前已经可被发现，但会拒绝目标。这不是
            # 路径规划失败，不能污染 blocked_frontiers；短暂退避后原目标可重新选择。
            self.nav_target = None
            self.nav_obstacle_position = None
            self.nav_obstacle_id = ""
            self._defer_obstacle_revisit(self.nav_revisit_id, time.monotonic())
            self.nav_revisit_id = ""
            self.nav_purpose = ""
            self.nav_retry_until = time.monotonic() + float(
                self.params["nav_rejection_retry_delay"]
            )
            if self.enabled:
                self.state = "EXPLORING"
            self._publish_state("Nav2 rejected goal" + response_error)
            return
        self.nav_handle, self.nav_started = handle, time.monotonic()
        self.nav_progress_pose = self._motion_pose()
        self.nav_progress_time = self.nav_started
        if not self.enabled:
            self._cancel_nav("stopped_before_accept")
        handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        # Nav2 重启、DDS 断连或 Action server 销毁时，结果 future 可以异常结束。异常仍
        # 必须经过下面统一的句柄/目标清理和有界重试流程，不能让 executor 回调抛出并使
        # 独立自主任务进程退出。
        try:
            result = future.result()
            status = int(result.status) if result is not None else 0
            result_error = ""
            result_failed = False
        except Exception as exc:
            result = None
            status = GoalStatus.STATUS_UNKNOWN
            result_error = f": {exc}"
            result_failed = True
            self.get_logger().error(f"Nav2 result failed{result_error}")
        purpose, target = self.nav_purpose, self.nav_target
        revisit_id = self.nav_revisit_id
        cancel_reason = self.nav_cancel_reason
        obstacle_position = self.nav_obstacle_position
        approach_initial_id = self.nav_obstacle_id
        self.nav_handle, self.nav_target, self.nav_cancel_pending = None, None, False
        self.nav_purpose = ""
        self.nav_progress_pose = None
        self.nav_obstacle_position = None
        self.nav_obstacle_id = ""
        self.nav_revisit_id = ""
        self.nav_cancel_reason = ""
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        if purpose == "verify_obstacle":
            self.semantic_settle_until = time.monotonic() + float(
                self.params["semantic_post_turn_settle_time"]
            )
        elif purpose == "entry_recovery":
            self.semantic_settle_until = time.monotonic() + float(
                self.params["failed_entry_settle_time"]
            )
            if succeeded and self.failed_entry_escape_after_turn:
                # The rotation changed only the viewing direction.  Queue a short
                # normal Nav2 translation along that new heading so the next
                # observation comes from a genuinely new station.  cmd_vel_gate does
                # not grant rotation-recovery bypass to this phase: front terrain,
                # emergency scan and navigation-health checks all remain mandatory.
                self.failed_entry_escape_pending = float(
                    self.params["failed_entry_escape_distance"]
                )
            # The flag belongs to one queued recovery only.  Restore the ordinary
            # obstacle policy even when Nav2 aborts the yaw goal.
            self.failed_entry_escape_after_turn = True
        elif purpose == "prealign_obstacle":
            self.pre_alignment_settle_until = time.monotonic() + float(
                self.params["pre_alignment_settle_time"]
            )
        if purpose == "return_home":
            # Arrival is evaluated even when Nav2 returns ABORTED/CANCELED.  The
            # physical/map pose is authoritative; Action status only decides whether
            # another attempt is needed when the robot remains outside the finish.
            if self._complete_at_finish_if_arrived(self._robot_pose()):
                return
            self.return_attempts += 1
            self.nav_retry_until = time.monotonic() + float(
                self.params["nav_failure_retry_delay"]
            )
            self.state = "RETURNING_TO_FINISH"
            self._publish_state(
                f"finish attempt {self.return_attempts} ended with status={status}; retrying"
            )
            return
        if purpose == "revisit_obstacle":
            # 到达旧观察位后给相机/点云一个完整稳定窗口重新捕获。即使 Nav2 中止，也只
            # 延迟该障碍，不妨碍任务继续选择其他未完成项。Nav2 到达观察位只证明路径
            # 成功，不证明相机重新识别成功；因此成功/失败都算一次有界回访尝试并递增
            # 退避。即使相机仍持续看到该目标，fresh-target 分支也要遵守同一冷却，
            # 否则连续图像帧会绕过回访调度并立即重建同一次失败的交接。
            self._penalize_obstacle_revisit(revisit_id, time.monotonic())
        if purpose in ("frontier", "coverage") and not succeeded and target is not None:
            self.blocked_frontiers.append(target)
        if not succeeded:
            # ABORT/CANCEL 后给 costmap、行为树和地图更新留下恢复窗口。特别是刚越障时，
            # SLAM 地图边界可能比机器人落后一个发布周期，立即重发只会制造失败风暴。
            self.nav_retry_until = time.monotonic() + float(
                self.params["nav_failure_retry_delay"]
            )
        if result_failed and purpose == "approach":
            # 结果传输异常时无法证明旧 Nav2 goal 已经终止。即使该目标先前因 stall
            # 发出过取消请求，也绝不能继续走下面的“停在膨胀层后交接”分支，否则旧
            # Nav2 controller 与 TraverseObstacle 可能同时拥有运动控制权。释放入口锁，
            # 等 Action 客户端/服务器恢复后从新鲜地图和感知重新建立目标。
            self._reset_obstacle_lock()
            self.approach_stall_id = ""
            self.approach_stall_count = 0
        if (
            self.enabled
            and not result_failed
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
            # 接近过程中点云会从远端边缘逐步收敛到真实入口。初次 Nav2 目标对应的
            # obstacle_position 可能因此移动近 1 m；任务锁保存了持续低通后的同一入口，
            # 应以它做交叉验证，而不是永远拿第一帧边缘与最后一帧中心比较。
            entry_reference = self.locked_obstacle_position or obstacle_position
            # Keep spatial identity separate from the nominal close-handoff envelope.
            # A robot stopped at the costmap inflation boundary may still be 1.9~2.0 m
            # away; using the old 1.45 m distance gate as the identity gate would discard
            # the frozen semantic before the wider guarded-stall handoff can evaluate it.
            spatial_match = (
                latest_position is not None
                and hypot(
                    latest_position[0] - entry_reference[0],
                    latest_position[1] - entry_reference[1],
                ) <= float(self.params["handoff_fallback_spatial_tolerance"])
            )
            same_entry = (
                spatial_match
                and latest.distance
                <= float(self.params["handoff_fallback_max_distance"])
                and abs(latest.lateral_offset)
                <= float(self.params["handoff_fallback_max_lateral"])
            )
            guarded_cancel = (
                status == GoalStatus.STATUS_CANCELED
                and cancel_reason in ("stall", "approach_within_tolerance")
            )
            live_semantic_id = (
                self._action_semantic_id(
                    latest,
                    self.locked_obstacle_id or self._current_obstacle_id(),
                )
                if latest is not None
                else ""
            )
            semantic_id = semantic_after_approach_stall(
                approach_initial_id,
                live_semantic_id,
                action_obstacle_type(latest) if latest is not None else 0,
                spatial_match,
            )
            # Even if the live message vanished entirely, retain the initial ID for
            # retry accounting only.  It may trigger cooldown, never Action handoff,
            # because ``semantic_id`` above still requires current spatial evidence.
            stall_tracking_id = (
                semantic_id
                or (
                    approach_initial_id
                    if is_actionable_semantic_id(approach_initial_id)
                    else ""
                )
            )
            if cancel_reason == "stall" and stall_tracking_id:
                if stall_tracking_id == self.approach_stall_id:
                    self.approach_stall_count += 1
                else:
                    self.approach_stall_id = stall_tracking_id
                    self.approach_stall_count = 1
            elif succeeded:
                self.approach_stall_id = ""
                self.approach_stall_count = 0

            # 对坑沿、墙脚和台阶立面，Nav2 的正确行为本来就是停在障碍外，因而无法
            # 达到位于障碍投影内的 approach goal。只有同一类别连续停滞两次，并且最新
            # 点云仍证明目标在正前方、横偏/航向误差受限时，才允许越过空间一致性门限。
            # 这是“导航控制权交给越障控制器”，不是把障碍从代价地图中删掉。
            repeated_stall_handoff = (
                cancel_reason == "stall"
                and semantic_id
                and stall_tracking_id == self.approach_stall_id
                and self.approach_stall_count
                >= int(self.params["approach_stall_handoff_count"])
                and latest is not None
                and latest_position is not None
                and latest.distance
                <= float(self.params["approach_stall_handoff_max_distance"])
                and abs(latest.lateral_offset)
                <= float(self.params["approach_stall_handoff_max_lateral"])
                and abs(latest.heading_error)
                <= float(self.params["approach_stall_handoff_max_heading_error"])
            )
            if semantic_id and ((
                same_entry
                and (nav_status_allows_guarded_handoff(status) or guarded_cancel)
            ) or repeated_stall_handoff):
                now = time.monotonic()
                # 连续停滞旁路使用最新测得的入口位置；旧 Nav2 goal 对应的远端边缘
                # 可能已经随视角变化，拿旧坐标做完成去重会重复处理同一个障碍。
                handoff_position = (
                    latest_position if repeated_stall_handoff else obstacle_position
                )
                queued = self._queue_traversal_handoff(
                    latest, semantic_id, handoff_position, now
                )
                if queued:
                    self._publish_state(
                        "confirmed obstacle boundary; handing off after "
                        f"Nav2 status={status}, approach stalls={self.approach_stall_count}"
                    )
                return
            if (
                cancel_reason == "stall"
                and (
                    not stall_tracking_id
                    or self.approach_stall_count
                    >= int(self.params["approach_stall_handoff_count"])
                )
            ):
                # 同一入口已经停滞，但距离、横偏、航向或语义一致性没有通过越障交接门。
                # “没有唯一语义”本身也必须进入本分支；否则计数无法绑定语义 ID，任务会
                # 每 5 秒取消并立即重发同一 generic STEP/WALL 入口。把最新/原入口短暂
                # 加入空间冷却，稍后从新视角复核，同时允许机器人先处理其他目标。
                blocked = latest_position or obstacle_position
                self.blocked_obstacles.append((
                    float(blocked[0]),
                    float(blocked[1]),
                    time.monotonic() + float(self.params["obstacle_failure_cooldown"]),
                ))
                self.approach_stall_id = ""
                self.approach_stall_count = 0
                recovery_now = time.monotonic()
                # 同一观察点不能在下一 tick 立刻被 pending-obstacle 分支再次选择。
                # 仅加冷却仍会让前沿规划器在障碍边缘反复选择不可达目标：排队一次
                # 交替方向的 90° 转向；转向成功后统一恢复链还会通过普通 Nav2 前移
                # failed_entry_escape_distance，从物理上改变观察站。该平移没有任何
                # 安全旁路，前方仍不安全时会被速度门保持为零并再次触发看门狗。
                self._defer_obstacle_revisit(
                    semantic_id or approach_initial_id,
                    recovery_now,
                )
                self.failed_entry_escape_after_turn = True
                self.failed_entry_turn_pending = (
                    self.ambiguous_recovery_sign
                    * float(self.params["failed_entry_turn_angle"])
                )
                self.ambiguous_recovery_sign *= -1.0
                self.cooldown_until = recovery_now + float(
                    self.params["nav_failure_retry_delay"]
                )
                self._publish_state(
                    "approach stalled and handoff gates remain unsafe; "
                    f"distance={float(latest.distance) if latest is not None else -1.0:.2f}, "
                    f"lateral={float(latest.lateral_offset) if latest is not None else 0.0:.2f}, "
                    f"heading={float(latest.heading_error) if latest is not None else 0.0:.2f}, "
                    f"semantic={semantic_id or 'none'}, "
                    f"spatial={'ok' if spatial_match else 'bad'}, "
                    "limits=("
                    f"d<={float(self.params['approach_stall_handoff_max_distance']):.2f}, "
                    f"|y|<={float(self.params['approach_stall_handoff_max_lateral']):.2f}, "
                    f"|yaw|<={float(self.params['approach_stall_handoff_max_heading_error']):.2f}); "
                    "changing heading and observation station before replanning"
                )
            # 到达旧入口后若最新证据已经属于另一个障碍，释放目标锁再探索，不能把新障碍
            # 的相对目标误套到旧入口，也不能永久锁死。
            self._reset_obstacle_lock()
        if self.enabled:
            self.state = "EXPLORING"
        self._publish_state(
            f"Nav2 {purpose} finished with status={status}{result_error}"
        )

    def _nav_is_stalled(self, robot, now: float) -> bool:
        """检测“Action 仍运行但机体不再前进/转向”，避免等满 45 秒才恢复。

        优先比较连续本地 odom，不把目标反馈或命令发布当作运动；odom 断流才退回传入的
        map 位姿。达到任一平移/旋转阈值就刷新窗口，因此低速转向不会因 SLAM 延迟被误判。
        """
        motion_pose = self._motion_pose(now) or robot
        if self.nav_handle is None or motion_pose is None:
            return False
        if self.nav_progress_pose is None:
            self.nav_progress_pose = motion_pose
            self.nav_progress_time = now
            return False
        translation = hypot(
            motion_pose[0] - self.nav_progress_pose[0],
            motion_pose[1] - self.nav_progress_pose[1],
        )
        rotation = abs(normalized_angle(
            motion_pose[2] - self.nav_progress_pose[2]
        ))
        if (
            translation >= float(self.params["nav_progress_translation"])
            or rotation >= float(self.params["nav_progress_rotation"])
        ):
            self.nav_progress_pose = motion_pose
            self.nav_progress_time = now
            return False
        stall_timeout = float(
            self.params[
                "return_nav_stall_timeout"
                if self.nav_purpose == "return_home"
                else "nav_stall_timeout"
            ]
        )
        return now - self.nav_progress_time >= stall_timeout

    def _start_traverse(self, guidance):
        if (
            self.traverse_handle is not None
            or self.traverse_send_pending
            or not self.traverse_client.server_is_ready()
        ):
            return
        # 入口接近期间分类仍可能变化。必须在构造 Action 之前做最后一次“名称 + 几何”
        # 一致性校验；没有稳定比赛 ID 时返回探索换视角，绝不发送匿名直行命令。
        if not self.pending_traverse_id:
            self.pending_traverse_id = self._action_semantic_id(
                guidance,
                self.locked_obstacle_id or self._current_obstacle_id(),
            )
        if not is_actionable_semantic_id(self.pending_traverse_id):
            self.pending_traverse = None
            self.pending_traverse_position = None
            self.pending_traverse_robot_start = None
            self._reset_obstacle_lock()
            self.cooldown_until = time.monotonic() + 1.0
            self.state = "EXPLORING" if self.enabled else "STOPPED"
            self._publish_state(
                "handoff rejected: obstacle identity is not stable; resuming observation"
            )
            return
        goal = TraverseObstacle.Goal()
        # 冻结 Action 开始时的真实机体位置；成功回调再取出口位置，形成长障碍去重线段。
        # 不能用障碍前缘位置代替，因为它会随点云视角漂移。
        if self.pending_traverse_robot_start is None:
            robot = self._robot_pose()
            if robot is not None:
                self.pending_traverse_robot_start = (robot[0], robot[1])
        goal.obstacle_type = action_type_for_semantic(
            self.pending_traverse_id,
            action_obstacle_type(guidance),
        )
        goal.obstacle_id = str(self.pending_traverse_id)
        for field in ("confidence", "distance", "lateral_offset", "heading_error"):
            setattr(goal, field, getattr(guidance, field))
        self.state, self.traverse_started = "TRAVERSING", time.monotonic()
        self.traverse_send_pending = True
        self._publish_state(
            f"handoff obstacle={self.pending_traverse_id}, "
            f"action_type={goal.obstacle_type}"
        )
        future = self.traverse_client.send_goal_async(goal)
        future.add_done_callback(self._traverse_goal_response)

    def _hold_for_traversal_controller(self, target, position, now):
        """入口已到达但执行器未接入时保持原地，不让 Nav2把赛道障碍当作绕行物。

        Gazebo、真机 SDK 或未来运动控制器都只能通过同一个 Action 合同接入。任务管理器
        不猜测腿部动作，也不会因服务缺失继续选择障碍背后的前沿目标。
        """
        self.pending_traverse = target
        self.pending_traverse_id = self._action_semantic_id(
            target,
            self.locked_obstacle_id or self._current_obstacle_id(now),
        )
        if not is_actionable_semantic_id(self.pending_traverse_id):
            self.pending_traverse = None
            self.pending_traverse_position = None
            self.pending_traverse_robot_start = None
            self._reset_obstacle_lock()
            self.state = "EXPLORING" if self.enabled else "STOPPED"
            self._publish_state(
                "controller handoff deferred: obstacle identity is not stable"
            )
            return
        self.pending_traverse_position = position
        self.pending_traverse_started = now
        self.state = "WAITING_FOR_TRAVERSAL_CONTROLLER"
        self._cancel_nav("waiting_for_traversal_controller")
        if not self.controller_wait_reported:
            self.controller_wait_reported = True
            self._publish_state(
                "entry reached; waiting for /traverse_obstacle controller"
            )

    def _abandon_controller_wait(self) -> None:
        """Leave a missing controller after a bounded wait and continue elsewhere.

        The obstacle remains in the pending ledger and its map neighborhood receives the same
        cooldown used for a failed traversal.  This prevents the next 4 Hz tick from selecting
        the identical entry again while still allowing a later revisit after the controller or
        perception recovers.
        """
        semantic_id = self.pending_traverse_id
        position = self.pending_traverse_position
        self.pending_traverse = None
        self.pending_traverse_position = None
        self.pending_traverse_robot_start = None
        self.pending_traverse_id = ""
        self.controller_wait_reported = False
        self._reject_traversal_completion(
            semantic_id,
            position,
            "TraverseObstacle controller unavailable for "
            f"{float(self.params['controller_wait_timeout']):.1f} seconds",
        )

    def _traverse_goal_response(self, future):
        self.traverse_send_pending = False
        try:
            handle = future.result()
            response_error = ""
        except Exception as exc:  # Treat transport failure exactly like a rejected handoff.
            handle = None
            response_error = f": {exc}"
            self.get_logger().error(
                f"TraverseObstacle goal response failed{response_error}"
            )
        if handle is None or not handle.accepted:
            semantic_id = self.pending_traverse_id
            position = self.pending_traverse_position
            self.pending_traverse = None
            self.pending_traverse_position = None
            self.pending_traverse_robot_start = None
            self.pending_traverse_id = ""
            self.controller_wait_reported = False
            self._reject_traversal_completion(
                semantic_id,
                position,
                "TraverseObstacle unavailable/rejected" + response_error,
            )
            return
        self.traverse_handle = handle
        if not self.enabled:
            self.traverse_cancel_pending = True
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self._traverse_result)

    def _traverse_result(self, future):
        """Accept controller success only as a request for task-level verification.

        ROS Action status, controller result, map displacement, crossing direction and
        post-motion stability are separate pieces of evidence.  Completion is recorded
        only after all available upper-layer checks pass; the real controller remains
        responsible for feet/contact, body attitude and actuator-fault verification.
        """
        # DDS/服务器异常会让 result future 抛出异常，而不是返回一个失败 Result。任务节点
        # 不能因此崩溃，也绝不能沿用上一次成功；统一降级为未证实并把本障碍留在待办。
        try:
            wrapped = future.result()
            result_exception = ""
        except Exception as exc:  # rclpy future exposes several middleware errors.
            wrapped = None
            result_exception = f"Action result exception: {exc}"
            self.get_logger().error(result_exception)
        action_succeeded = bool(
            wrapped
            and int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.success
        )
        controller_message = (
            str(wrapped.result.message)
            if wrapped and wrapped.result
            else result_exception
        )
        completed_position = self.pending_traverse_position
        completed_robot_start = self.pending_traverse_robot_start
        completed_type = (
            int(self.pending_traverse.obstacle_type)
            if self.pending_traverse is not None
            else TraversalGuidance.OBSTACLE_UNKNOWN
        )
        completed_id = self.pending_traverse_id
        self.traverse_handle, self.pending_traverse = None, None
        self.traverse_cancel_pending = False
        self.pending_traverse_position = None
        self.pending_traverse_robot_start = None
        self.pending_traverse_id = ""
        # 无论 Action 成功、失败还是被取消，下一障碍都必须重新累计入口停滞，不能继承
        # 当前障碍获得的交接许可。
        self.approach_stall_id = ""
        self.approach_stall_count = 0
        if not self.enabled:
            self.state = "STOPPED"
            return
        if (
            action_succeeded
            and is_actionable_semantic_id(completed_id)
            and completed_position is not None
            and completed_robot_start is not None
        ):
            now = time.monotonic()
            self.traversal_verification = TraversalVerification(
                semantic_id=completed_id,
                obstacle_type=completed_type,
                obstacle_position=(
                    float(completed_position[0]), float(completed_position[1])
                ),
                robot_start=(
                    float(completed_robot_start[0]),
                    float(completed_robot_start[1]),
                ),
                controller_message=controller_message,
                started=now,
            )
            self._reset_obstacle_lock()
            self.state = "VERIFYING_TRAVERSAL_RESULT"
            self._publish_state(
                f"controller reported success for {completed_id}; "
                "verifying crossing displacement and landing stability"
            )
            return
        self._reject_traversal_completion(
            completed_id,
            completed_position,
            "controller result/ROS Action status did not prove success"
            + (f" ({controller_message})" if controller_message else ""),
        )

    def _reject_traversal_completion(self, semantic_id, position, reason: str) -> None:
        """Keep an unverified obstacle pending and schedule a different observation.

        Merely cooling down the obstacle point is insufficient for a long bridge or
        stair: its nearest point moves along the structure as the sensor turns, so a
        second point can immediately evade the radius.  We therefore combine three
        independent responses: semantic retry backoff, a temporary spatial exclusion,
        and one bounded in-place viewpoint change.  The latter is executed by Nav2 and
        remains subject to the normal health/scan watchdogs.
        """
        now = time.monotonic()
        self.traversal_verification = None
        semantic_id = str(semantic_id)
        self._penalize_obstacle_revisit(semantic_id, now)
        if is_actionable_semantic_id(semantic_id):
            failures = self.failed_entry_failures.get(semantic_id, 0) + 1
            self.failed_entry_failures[semantic_id] = failures
            # Alternate one 90-degree turn left/right. Multiplying the new angle on a
            # repeated failure would create a 180-degree turn and waste the budget.
            turn_sign = 1.0 if failures % 2 else -1.0
            self.failed_entry_escape_after_turn = True
            self.failed_entry_turn_pending = turn_sign * float(
                self.params["failed_entry_turn_angle"]
            )
            robot = self._robot_pose()
            if robot is not None:
                self.failed_entries.append(FailedEntry(
                    semantic_id=semantic_id,
                    robot_x=float(robot[0]),
                    robot_y=float(robot[1]),
                    robot_yaw=float(robot[2]),
                    expires=now + float(self.params["failed_entry_memory_duration"]),
                ))
        if position is not None:
            self.blocked_obstacles.append((
                float(position[0]),
                float(position[1]),
                now + float(self.params["obstacle_failure_cooldown"]),
            ))
        self._reset_obstacle_lock()
        self.cooldown_until = now + float(self.params["nav_failure_retry_delay"])
        self.state = "RECOVERY"
        self._publish_state(
            f"traversal not counted: {reason}; obstacle remains pending with backoff"
        )

    def _confirm_traversal_completion(
        self,
        verification: TraversalVerification,
        robot_exit: Tuple[float, float, float],
    ) -> None:
        """Commit one obstacle after controller and independent task checks pass."""
        self.completed_obstacles.append((
            verification.obstacle_type,
            verification.obstacle_position[0],
            verification.obstacle_position[1],
        ))
        self.completed_semantics = list(resolve_completed_semantics(
            self.completed_semantics, verification.semantic_id
        ))
        self.failed_entry_failures.pop(verification.semantic_id, None)
        self.failed_entries = [
            record for record in self.failed_entries
            if record.semantic_id != verification.semantic_id
        ]
        self.completed_traversal_segments.append((
            verification.semantic_id,
            verification.robot_start,
            (float(robot_exit[0]), float(robot_exit[1])),
        ))
        self.traversal_verification = None
        self.cooldown_until = time.monotonic() + float(
            self.params["post_traversal_cooldown"]
        )
        self.state = "EXPLORING"
        completed_inventory, pending_inventory = mission_inventory(
            self.params["expected_obstacle_ids"], self.completed_semantics
        )
        self._publish_state(
            f"obstacle verified complete: {verification.semantic_id}; "
            f"tasks={len(completed_inventory)}/"
            f"{len(completed_inventory) + len(pending_inventory)}; "
            "continuing with next pending obstacle"
        )

    def _verify_traversal_completion(
        self, robot: Tuple[float, float, float], now: float
    ) -> None:
        """Accumulate bounded stillness and geometric crossing evidence."""
        verification = self.traversal_verification
        if verification is None:
            return
        if verification.last_pose is None:
            verification.last_pose = robot
            verification.stable_since = now
        else:
            translation = hypot(
                robot[0] - verification.last_pose[0],
                robot[1] - verification.last_pose[1],
            )
            rotation = abs(normalized_angle(robot[2] - verification.last_pose[2]))
            if (
                translation > float(self.params["post_traversal_stable_translation"])
                or rotation > float(self.params["post_traversal_stable_rotation"])
            ):
                # Start a fresh stability window anchored at the new pose.  Do not
                # update the anchor for tiny per-tick motion, otherwise slow drift
                # could incorrectly appear stable forever.
                verification.last_pose = robot
                verification.stable_since = now
        crossed = traversal_geometry_evidence(
            verification.semantic_id,
            verification.robot_start,
            verification.obstacle_position,
            (robot[0], robot[1]),
            minimum_displacement=float(
                self.params["post_traversal_minimum_displacement"]
            ),
            beyond_obstacle_margin=float(
                self.params["post_traversal_beyond_margin"]
            ),
        )
        stable = now - verification.stable_since >= float(
            self.params["post_traversal_stable_duration"]
        )
        if crossed and stable:
            self._confirm_traversal_completion(verification, robot)
            return
        if now - verification.started >= float(
            self.params["post_traversal_verification_timeout"]
        ):
            reason = (
                "robot did not cross the observed entry plane"
                if not crossed
                else "robot did not become stable after landing"
            )
            self._reject_traversal_completion(
                verification.semantic_id,
                verification.obstacle_position,
                reason,
            )

    def _tick(self):
        self.state_pub.publish(String(data=self.state))
        # 20 Hz 速度门只接受新鲜许可；任务崩溃或 Ctrl-C 后最多一个心跳
        # 窗口便恢复默认拒绝。许可仅让速度门提取有界 angular.z，所有线速度
        # 都被强制归零。approach 必须包含在内：障碍将地形限速置零时，机体仍需要
        # 消除最后几度航向误差；否则会每 5 秒取消同一入口。handoff/traversal 不属于
        # Nav2 运动，entry_escape 是平移，因此不获得该许可。
        self.rotation_recovery_pub.publish(Bool(data=bool(
            navigation_purpose_allows_yaw_only_recovery(self.nav_purpose)
            and (self.nav_handle is not None or self.nav_send_pending)
        )))
        self._publish_inventory()
        if not self.enabled:
            return
        now = time.monotonic()
        if (
            self.nav_handle is not None
            and now - self.nav_started > float(self.params["goal_timeout"])
        ):
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
        # Action 返回成功后禁止立即恢复探索。先用独立的 map->base_link 轨迹证明机体已
        # 越过感知到的入口平面，并等待落地静止窗口；否则错误的控制器 success 会把未越过
        # 的障碍从待办清单删除。验证期不发 Nav2 目标，也不采纳新的障碍语义。
        if self.traversal_verification is not None:
            self.state = "VERIFYING_TRAVERSAL_RESULT"
            self._verify_traversal_completion(robot, now)
            return
        if (
            not self.coverage_visited
            or hypot(
                robot[0] - self.coverage_visited[-1][0],
                robot[1] - self.coverage_visited[-1][1],
            ) >= float(self.params["coverage_record_spacing"])
        ):
            self.coverage_visited.append((robot[0], robot[1]))
        if self.home_pose is None:
            # 起点必须来自任务启动时的实时 map->base_link，而不是规则图或 Gazebo pose。
            # 若没有外部 finish_pose，这个实时位姿就是默认终点；不依赖任何场地坐标。
            self.home_pose = robot
            self.mission_started = now
            self.mission_ready_after = now + float(
                self.params["startup_sensor_settle_time"]
            )
            self._publish_state(
                f"start pose captured at ({robot[0]:.2f}, {robot[1]:.2f})"
            )
        if now < self.mission_ready_after:
            self.state = "WAITING_FOR_INPUTS"
            return
        # 2D 雷达无法从“黄色赛台边缘”获知场地语义，深度云与颜色融合会把它明确标成
        # 场地边界。此时终止当前前沿目标并临时拉黑，选择另一方向；绝不能把边缘负高度
        # 当作砂砾坑交给 TraverseObstacle。该逻辑只依赖在线感知名称，不读取 world 坐标。
        if (
            self._arena_boundary_ahead(now)
            and self.nav_handle is not None
            and not self.nav_cancel_pending
            and self.nav_purpose in (
                "frontier", "coverage", "revisit_obstacle", "approach"
            )
            # 相机仍看到旧边界时，侧后方的新目标正是脱离动作，必须允许 Nav2 先旋转；
            # 只有目标方向与当前边界方向一致时才取消。原地 search_turn 也始终放行。
            and target_is_in_heading_cone(robot, self.nav_target)
        ):
            if self.nav_purpose in ("frontier", "coverage") and self.nav_target is not None:
                self.blocked_frontiers.append(self.nav_target)
            if self.nav_purpose == "revisit_obstacle":
                self._defer_obstacle_revisit(self.nav_revisit_id, now)
            self._cancel_nav("arena_boundary")
            self.nav_retry_until = now + float(self.params["nav_failure_retry_delay"])
            self._reset_obstacle_lock()
            self.state = "RECOVERY"
            self._publish_state(
                "arena boundary detected; cancelling target and choosing another direction"
            )
            return
        if (
            self.nav_handle is not None
            and not self.nav_cancel_pending
            and self._nav_is_stalled(robot, now)
        ):
            stalled_purpose = self.nav_purpose
            stalled_target = self.nav_target
            guidance = self.guidance
            guidance_is_blocking = (
                guidance is not None
                and now - self.guidance_received
                <= float(self.params["guidance_timeout"])
                and bool(guidance.perception_valid)
                and bool(guidance.traversal_required)
            )
            if stalled_purpose in ("frontier", "coverage") and stalled_target is not None:
                self.blocked_frontiers.append(stalled_target)
                # A frontier behind a currently visible obstacle is unreachable by
                # ordinary Nav2 because cmd_vel_gate correctly removes translation.
                # Merely blacklisting successive cells leaves the robot at the same
                # camera station and produced a repeatable 5 s cancellation loop in
                # the full-field run. Queue the same bounded change-of-station chain
                # used by an unsafe obstacle approach whenever fresh Guidance proves
                # that terrain, rather than a generic planner failure, blocks motion.
            # Return-to-finish can fail in the same way: DWB may repeatedly request
            # pure forward motion through an obstacle, so the speed gate correctly
            # outputs zero and no yaw exists to extract.  Schedule a bounded turn and
            # normal Nav2 escape before retrying home.  This does not mark or cross the
            # obstacle and remains fully subject to scan/terrain/health guards.
            if (
                stalled_purpose in ("frontier", "coverage", "return_home")
                and guidance_is_blocking
            ):
                # Exploration needs a genuinely new camera station.  Returning to
                # a known pose instead retries immediately after the yaw change;
                # translating along an arbitrary recovery heading can move farther
                # from home or into the same obstacle.
                self.failed_entry_escape_after_turn = stalled_purpose != "return_home"
                self.failed_entry_turn_pending = (
                    self.ambiguous_recovery_sign
                    * float(self.params["failed_entry_turn_angle"])
                )
                self.ambiguous_recovery_sign *= -1.0
                self._reset_obstacle_lock()
            if stalled_purpose == "revisit_obstacle":
                self._defer_obstacle_revisit(self.nav_revisit_id, now)
            self._cancel_nav("stall")
            self.nav_retry_until = now + float(self.params["nav_failure_retry_delay"])
            self.state = "RECOVERY"
            recovery_detail = (
                "; obstacle blocks goal, changing observation station"
                if stalled_purpose in ("frontier", "coverage", "return_home")
                and guidance_is_blocking
                else ""
            )
            self._publish_state(
                f"Nav2 {stalled_purpose} made no pose progress; "
                f"cancelling and replanning{recovery_detail}"
            )
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
        if now < self.semantic_settle_until:
            self.state = "VERIFYING_OBSTACLE"
            return
        if now < self.pre_alignment_settle_until:
            self.state = "ALIGNING_OBSTACLE"
            return
        # A rejected-entry turn must finish before live READY frames can cancel it and
        # recreate the same Action.  The result callback starts the post-turn sensor
        # settling window, so the next decision always uses observations from the new
        # heading rather than buffered frames from the failed one.
        if (
            self.nav_handle is not None
            and self.nav_purpose in ("entry_recovery", "entry_escape")
        ):
            return
        if self.failed_entry_escape_pending > 0.0:
            if self.nav_handle is not None or self.nav_send_pending:
                return
            escape_distance = float(self.failed_entry_escape_pending)
            self.failed_entry_escape_pending = 0.0
            self.state = "RECOVERY"
            # Prefer the global costmap because it contains both laser and filtered
            # terrain points. Fall back to the current SLAM map only if costmap data
            # has not arrived; both use the same standard OccupancyGrid contract.
            recovery_grid = (
                self.costmap_msg
                if self.costmap_msg is not None and now - self.costmap_received <= 2.0
                else self.map_msg
            )
            station = recovery_station_in_known_free_space(
                recovery_grid,
                robot,
                escape_distance,
                clearance=float(self.params["coverage_goal_clearance"]),
            ) if recovery_grid is not None else None
            if station is None:
                self.nav_retry_until = now + float(self.params["nav_failure_retry_delay"])
                self._publish_state(
                    "no costmap-clear recovery station; skipping blind translation"
                )
                return
            self._send_nav_goal(
                self._make_pose(*station),
                "entry_escape",
            )
            self._publish_state(
                "moving to a costmap-clear observation station "
                f"({hypot(station[0] - robot[0], station[1] - robot[1]):.2f} m)"
            )
            return
        if abs(self.failed_entry_turn_pending) > 0.0:
            # Perform exactly one bounded viewpoint change after a rejected Action.
            # Translation remains locked by cmd_vel_gate's rotation-recovery mode, so
            # this cannot turn an unsafe entry into an unreviewed forward command.
            if self.nav_handle is not None or self.nav_send_pending:
                return
            recovery_delta = float(self.failed_entry_turn_pending)
            self.failed_entry_turn_pending = 0.0
            self.state = "RECOVERY"
            self._send_nav_goal(
                self._make_pose(robot[0], robot[1], robot[2] + recovery_delta),
                "entry_recovery",
            )
            self._publish_state(
                "unsafe or ambiguous entry: changing viewpoint by "
                f"{degrees(recovery_delta):.0f} deg before selecting another target"
            )
            return
        if self.pending_traverse is not None:
            if (
                self.state == "WAITING_FOR_TRAVERSAL_CONTROLLER"
                and timeout_reached(
                    self.pending_traverse_started,
                    now,
                    float(self.params["controller_wait_timeout"]),
                )
            ):
                self._abandon_controller_wait()
                return
            if (
                self.state != "WAITING_FOR_TRAVERSAL_CONTROLLER"
                and timeout_reached(
                    self.pending_traverse_started,
                    now,
                    float(self.params["traversal_timeout"]),
                )
            ):
                self.pending_traverse = None
                self.pending_traverse_position = None
                self.pending_traverse_robot_start = None
                self.pending_traverse_id = ""
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

        # 完成八项后立即去终点；达到总任务时限也会携带已完成成绩结束，避免无前沿时
        # 永远原地等待。默认终点是本次任务实时捕获的起点；若 /autonomy/finish_pose
        # 提供正式终点则优先使用它。两种情况都不读取仿真 world。
        mission_elapsed = now - self.mission_started
        # ``mission_timeout`` 是“启动到回到终点”的总预算，不是可以全部用于探索的时间。
        # 提前保留返程窗口，防止直到 300 秒才开始回头。正在执行的 TraverseObstacle
        # 不会被硬切断；它完成独立落地验证后，下一 tick 立即进入返程。
        work_deadline = max(
            0.0,
            float(self.params["mission_timeout"])
            - float(self.params["return_time_reserve"]),
        )
        mission_timed_out = mission_elapsed >= work_deadline
        if (
            self._all_obstacles_complete()
            or mission_timed_out
            or self.exploration_exhausted
        ):
            if self.nav_handle is not None:
                if self.nav_purpose != "return_home":
                    self._cancel_nav("return_home")
                return
            if self.nav_send_pending:
                return
            terminal_pose = self.finish_pose or self.home_pose
            if terminal_pose is None:
                self.state = "WAITING_FOR_INPUTS"
                return
            # A simulator, recovery controller or future gait controller may have
            # already placed the robot inside the finish circle.  Finish immediately
            # instead of submitting a zero-distance goal that Nav2 can legitimately
            # reject when the start cell touches an inflated obstacle.
            if self._complete_at_finish_if_arrived(robot):
                return
            self.state = "RETURNING_TO_FINISH"
            self._send_nav_goal(
                self._make_pose(*terminal_pose),
                "return_home",
            )
            if self._all_obstacles_complete():
                reason = "all eight obstacles complete"
            elif self.exploration_exhausted:
                reason = "bounded exploration complete"
            else:
                reason = (
                    f"return reserve reached after {mission_elapsed:.0f} seconds"
                )
            destination = "configured finish" if self.finish_pose is not None else "start/finish"
            self._publish_state(f"{reason}; navigating to {destination}")
            return
        target = None if now < self.cooldown_until else self._fresh_target()
        if target is not None:
            self.search_turn_index = 0
            self.exploration_exhausted = False
            if not self._matches_obstacle_lock(target):
                # 多个障碍同时进入宽视场时，保持已确认入口，不在两个 Nav2 目标之间来回
                # cancel。旧目标结束后会释放锁，再处理新目标。
                if self.nav_handle is not None or self.nav_send_pending:
                    return
                self._reset_obstacle_lock()
            if target.phase == TraversalGuidance.PHASE_READY and target.ready_for_handoff:
                # 纯旋转目标必须真正执行完成并等待新点云。旧逻辑在下一帧 READY 到达时
                # 立即取消刚提交的 verify/pre-align goal，结果每次只转动约 0.25 s，四次
                # 尝试全部耗尽后仍保持原视角。对正过程中不得提前进入 Action 交接。
                if (
                    self.nav_handle is not None
                    and self.nav_purpose
                    in ("verify_obstacle", "prealign_obstacle")
                ):
                    return
                obstacle_position = (
                    self.locked_obstacle_position or self._obstacle_position(target)
                )
                live_handoff_id = self._resolved_obstacle_id(
                    target,
                    self.locked_obstacle_id
                    or self._current_obstacle_id(now),
                )
                handoff_id = (
                    live_handoff_id
                    if is_actionable_semantic_id(live_handoff_id)
                    else self._known_semantic_at_guidance(target)
                )
                if not is_actionable_semantic_id(handoff_id):
                    # Geometry says the robot is at an obstacle boundary,
                    # but its competition identity is not yet trustworthy.
                    # Keep the robot in place and collect alternate views;
                    # never fall back to a frontier command that immediately
                    # drives into the same safety stop again.
                    if self.nav_handle is not None:
                        self._cancel_nav("verify_obstacle")
                        self.state = "VERIFYING_OBSTACLE"
                        self._publish_state(
                            "READY geometry reached; cancelling Nav2 before "
                            "semantic verification"
                        )
                    elif not self.nav_send_pending:
                        self._verify_ambiguous_obstacle(target, now)
                    return
                queued = self._queue_traversal_handoff(
                    target, handoff_id, obstacle_position, now
                )
                if not queued:
                    return
                self._cancel_nav("handoff")
                identity_source = (
                    "live semantic"
                    if is_actionable_semantic_id(live_handoff_id)
                    else "pending map-position ledger"
                )
                self._publish_state(
                    "READY confirmed from " + identity_source
                    + "; cancelling Nav2 before handoff"
                )
                return
            if target.phase in (TraversalGuidance.PHASE_APPROACH, TraversalGuidance.PHASE_ALIGN):
                # The real TraverseObstacle controller contract includes the
                # remaining entry distance and performs the final slow approach.
                # Once a multi-frame semantic, metric geometry, lateral alignment
                # and strict heading are already valid, waiting for Nav2 to reach a
                # point inside the obstacle inflation layer adds no safety.  In the
                # field regression that wait consumed 20--35 s and repeatedly
                # toggled STOP/WALK at the same pose.  Handoff early, but only within
                # this deliberately smaller direct distance; the wider 2.35 m gate
                # remains reserved for a measured five-second approach stall.
                direct_id = self._action_semantic_id(
                    target,
                    self.locked_obstacle_id or self._current_obstacle_id(now),
                )
                direct_position = (
                    self.locked_obstacle_position or self._obstacle_position(target)
                )
                if close_handoff_is_safe(
                    direct_id,
                    target.distance,
                    target.lateral_offset,
                    target.heading_error,
                    float(self.params["direct_handoff_max_distance"]),
                    float(self.params["handoff_fallback_max_lateral"]),
                    float(self.params["handoff_alignment_tolerance"]),
                ):
                    queued = self._queue_traversal_handoff(
                        target, direct_id, direct_position, now
                    )
                    if queued:
                        self._cancel_nav("handoff")
                        self._publish_state(
                            "metric entry and body alignment confirmed; "
                            f"handing off {direct_id} before inflation-layer stall"
                        )
                    return
                approach = self._relative_approach_pose(target)
                if self.nav_handle is not None and self.nav_purpose in (
                    "frontier", "coverage", "revisit_obstacle"
                ):
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
                        initial_id = self._current_obstacle_id(now)
                        # _guidance_callback 已从首次看见该空间目标起累计投票；这里不得
                        # 清空，否则会再次丢掉远距离下更完整、更可靠的结构证据。
                        voted_id = select_full_semantic_vote(
                            self.semantic_votes, initial_id
                        )
                        # 只有连续多帧投票才能成为不可变入口语义；一两帧远场猜测仍
                        # 保持空锁，留给接近后的结构证据纠正。
                        self.locked_obstacle_id = (
                            voted_id
                            if list(self.semantic_votes).count(voted_id)
                            >= int(self.params["semantic_confirmation_votes"])
                            else ""
                        )
                    # 识别到唯一比赛障碍后，先主动把 base_link 对准坡轴/前缘法向，
                    # 再允许带平移的入口目标。旧流程把转向和平移合在一次 Nav2 goal 中，
                    # DWB 可能沿弧线斜着靠近，导致可用宽度变小且近场分类继续恶化。
                    # 这里只发同位置的 yaw 目标；速度门会丢弃任何意外线速度。每次转角
                    # 有上限，完成后还会等待新点云，因此方向噪声不会造成连续摆头。
                    pre_alignment = bounded_alignment_delta(
                        target.heading_error,
                        float(self.params["pre_alignment_trigger_angle"]),
                        float(self.params["pre_alignment_max_step"]),
                    )
                    pre_alignment_id = self._action_semantic_id(
                        target,
                        self.locked_obstacle_id
                        or self._current_obstacle_id(now),
                    )
                    if (
                        is_actionable_semantic_id(pre_alignment_id)
                        and abs(pre_alignment) > 0.0
                    ):
                        self.state = "ALIGNING_OBSTACLE"
                        self._send_nav_goal(
                            self._make_pose(
                                robot[0], robot[1], robot[2] + pre_alignment
                            ),
                            "prealign_obstacle",
                        )
                        self._publish_state(
                            f"obstacle={pre_alignment_id} confirmed; "
                            f"pre-aligning body by {degrees(pre_alignment):.1f} deg"
                        )
                        return
                    if robot_to_goal < float(self.params["minimum_approach_goal_distance"]):
                        semantic_id = self._action_semantic_id(
                            target,
                            self.locked_obstacle_id
                            or self._current_obstacle_id(now),
                        )
                        obstacle_position = (
                            self.locked_obstacle_position
                            or self._obstacle_position(target)
                        )
                        # 未确认到八项比赛语义时，不存在可定义的“障碍法向”。继续使用
                        # 通用 approach_yaw 只会让 Nav2 在其 yaw_goal_tolerance 内立即
                        # 成功并循环。直接进入有次数上限的左右换视角，观察失败后跳过。
                        if not is_actionable_semantic_id(semantic_id):
                            self._verify_ambiguous_obstacle(target, now)
                            return
                        if close_handoff_is_safe(
                            semantic_id,
                            target.distance,
                            target.lateral_offset,
                            target.heading_error,
                            float(self.params["handoff_fallback_max_distance"]),
                            float(self.params["handoff_fallback_max_lateral"]),
                            float(self.params["handoff_alignment_tolerance"]),
                        ):
                            queued = self._queue_traversal_handoff(
                                target, semantic_id, obstacle_position, now
                            )
                            if queued:
                                self._publish_state(
                                    "close entry and body alignment confirmed; "
                                    f"handing off {semantic_id} without a zero-distance Nav2 goal"
                                )
                            return
                        # 平移目标很近但姿态尚未对正时，只发送有意义的旋转。感知给出的
                        # approach_yaw 偶尔已被滤波为零，因此用 heading_error 作后备；两者
                        # 都小于 Nav2 航向容差则不再提交当前位置目标形成成功循环。
                        alignment_delta = normalized_angle(
                            target.approach_yaw
                            if abs(target.approach_yaw)
                            >= float(self.params["minimum_alignment_command_angle"])
                            else target.heading_error
                        )
                        if abs(alignment_delta) < float(
                            self.params["minimum_alignment_command_angle"]
                        ):
                            # 语义已经确认但小角度仍未满足严格 handoff，通常是瞬时横偏或
                            # 点云抖动。保留入口锁并等下一帧，不占用 Nav2 Action 通道。
                            self.state = "ALIGNING_OBSTACLE"
                            return
                        rotation_only = self._make_pose(
                            robot[0], robot[1], robot[2] + alignment_delta
                        )
                        self.state = "ALIGNING_OBSTACLE"
                        self._send_nav_goal(rotation_only, "approach", target)
                        self._publish_state(
                            f"aligning body to {self.locked_obstacle_id or 'detected obstacle'}"
                        )
                        return
                    self.state = "ALIGNING_OBSTACLE"
                    self._send_nav_goal(approach, "approach", target)
                return
        if self.nav_handle is not None or self.nav_send_pending:
            return

        # 只要仍存在可用未知前沿，就优先扩张地图。远距离语义记录的观察点可能已被
        # 后续 SLAM/代价地图证明不可达；旧顺序会在两个这类点之间每 5 秒往返重试，
        # 虽然不会永久卡死，却会耗尽 300 秒预算。前沿耗尽后才主动回访账本；实时位于
        # 正前方的障碍始终由上面的 fresh_target 分支最高优先处理，不受本调度改变。
        candidates = extract_frontiers(
            self.map_msg,
            (robot[0], robot[1]),
            minimum_cells=int(self.params["frontier_minimum_cells"]),
            minimum_distance=float(self.params["frontier_minimum_distance"]),
            maximum_distance=float(self.params["frontier_maximum_distance"]),
            goal_standoff=float(self.params["frontier_goal_standoff"]),
            goal_clearance=float(self.params["frontier_goal_clearance"]),
        )
        frontier = choose_frontier(
            candidates,
            self.blocked_frontiers,
            float(self.params["frontier_exclusion_radius"]),
        )

        # 回访目标是当时确认障碍的安全观察位姿，并把机身朝向障碍中心；到达后仍必须
        # 重新通过实时感知、对正和 Action 门，绝不会凭历史记录盲目越障。
        eligible_pending_records = tuple(
            record
            for record in self.observed_obstacles.values()
            if not failed_entry_matches(
                self.failed_entries,
                record.semantic_id,
                (record.view_x, record.view_y, record.view_yaw),
                now,
                float(self.params["failed_entry_station_tolerance"]),
                float(self.params["failed_entry_heading_tolerance"]),
                require_new_station=record.semantic_id in LONG_TRAVERSAL_IDS,
            )
        )
        pending_record = choose_pending_obstacle(
            eligible_pending_records,
            self.completed_semantics,
            robot,
            now,
        )
        if frontier is None and pending_record is not None:
            self.search_turn_index = 0
            self.exploration_exhausted = False
            distance_to_view = hypot(
                pending_record.view_x - robot[0],
                pending_record.view_y - robot[1],
            )
            if distance_to_view <= float(
                self.params["obstacle_revisit_position_tolerance"]
            ):
                # 已在观察点附近时只转向，不生成小于 Nav2 位置容差的伪平移目标。
                goal_x, goal_y = robot[0], robot[1]
            else:
                goal_x, goal_y = pending_record.view_x, pending_record.view_y
            goal_yaw = atan2(
                pending_record.obstacle_y - goal_y,
                pending_record.obstacle_x - goal_x,
            )
            self._defer_obstacle_revisit(pending_record.semantic_id, now)
            self.state = "SEEKING_PENDING_OBSTACLE"
            self._send_nav_goal(
                self._make_pose(goal_x, goal_y, goal_yaw),
                "revisit_obstacle",
                revisit_id=pending_record.semantic_id,
            )
            self._publish_state(
                f"actively revisiting pending obstacle={pending_record.semantic_id}; "
                f"view_distance={distance_to_view:.2f} m"
            )
            return
        if frontier is None:
            # 激光的长视距可能早于相机/深度 ROI 把整场变成 known；没有 frontier 时不能
            # 直接认定探索结束。先走访尚未靠近的已知自由区，让近距感知覆盖每个障碍。
            coverage = choose_frontier(
                extract_coverage_goals(
                    self.map_msg,
                    (robot[0], robot[1]),
                    self.coverage_visited,
                    spacing=float(self.params["coverage_goal_spacing"]),
                    clearance=float(self.params["coverage_goal_clearance"]),
                    visit_radius=float(self.params["coverage_visit_radius"]),
                    minimum_distance=float(self.params["frontier_minimum_distance"]),
                    maximum_distance=float(self.params["frontier_maximum_distance"]),
                ),
                self.blocked_frontiers,
                float(self.params["frontier_exclusion_radius"]),
            )
            if coverage is not None:
                self.empty_frontier_count = 0
                self.search_turn_index = 0
                self.exploration_exhausted = False
                yaw = atan2(coverage.y - robot[1], coverage.x - robot[0])
                self.state = "COVERAGE_EXPLORING"
                self._send_nav_goal(
                    self._make_pose(coverage.x, coverage.y, yaw), "coverage"
                )
                self._publish_state(
                    f"coverage survey: distance={coverage.distance:.2f} m"
                )
                return
            self.empty_frontier_count += 1
            if self.empty_frontier_count >= int(self.params["empty_frontier_confirmations"]):
                self.empty_frontier_count = 0
                if self.search_turn_index >= int(
                    self.params["maximum_search_turns"]
                ):
                    self.exploration_exhausted = True
                    self.state = "EXPLORATION_EXHAUSTED"
                    self._publish_state(
                        "bounded missing-obstacle scan exhausted; returning with current ledger"
                    )
                    return
                self.search_turn_index += 1
                # 分段旋转而不是一次 360°：每个 90° 目标都由 Nav2/碰撞监控接管，并在
                # 转动过程中持续更新 SLAM 和障碍识别。转满一圈后清除临时失败前沿，允许
                # 代价地图恢复后再次尝试，解决“走到一半自己停下”。
                if self.search_turn_index % 4 == 0:
                    self.blocked_frontiers.clear()
                yaw = robot[2] + float(self.params["search_turn_angle"])
                self.state = "SEARCHING_MISSING_OBSTACLES"
                self._send_nav_goal(
                    self._make_pose(robot[0], robot[1], yaw),
                    "search_turn",
                )
                missing = set(self.params["expected_obstacle_ids"]) - set(
                    self.completed_semantics
                )
                self._publish_state(
                    "no frontier: scanning for missing tasks "
                    + ",".join(sorted(missing))
                )
            return
        self.empty_frontier_count = 0
        self.search_turn_index = 0
        self.exploration_exhausted = False
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

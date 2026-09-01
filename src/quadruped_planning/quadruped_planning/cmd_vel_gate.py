"""Nav2 与底盘之间的最终失效安全速度门。

只有 Nav2 速度命令、地形安全评估和导航健康三条独立心跳都有效且新鲜时才允许非零
输出。最后再用二维雷达做一个极近距离急停检查，然后直接发布标准 ``/cmd_vel``。

该节点不规划路线、不创建新的运动意图，也不会把可越障目标改成绕行目标；雷达检查只在
物体已经进入紧急制动距离时兜底。真正的障碍分类、入口对正与越障交接仍分别属于
OpenCV/点云、Nav2 和 ``TraverseObstacle``。它也不替代硬件急停、驱动器看门狗或姿态
保护，只防止 ROS 节点失联或近距离碰撞时沿用最后一条速度。
"""

from math import isfinite, pi

import rclpy
from geometry_msgs.msg import Twist
from quadruped_interfaces.msg import AutonomyLease, TraversalGuidance

from quadruped_planning.parameter_validation import (
    SPEED_GATE_PARAMETER_NAMES,
    validate_speed_gate_parameters,
)
from quadruped_planning.time_utils import (
    ros_age_is_fresh,
    ros_clock_moved_backward,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


DIFFERENTIAL_UNUSED_AXIS_TOLERANCE = 1e-6
DEFAULT_SCAN_MIN_VALID_RATIO = 0.80
DEFAULT_SCAN_MAX_INVALID_GAP_ANGLE = 0.20
MAX_LEASE_SESSION_LENGTH = 128


def autonomy_lease_is_well_formed(message: AutonomyLease) -> bool:
    """Validate the identity/ordering fields before they affect ownership.

    ``session_id`` is deliberately opaque, but it must be a single canonical token so
    whitespace-only or accidentally concatenated IDs cannot become distinct owners in
    logs and state.  ``uint64`` prevents negative wire values; the explicit positive
    check reserves zero as "uninitialised".  Invalid messages are ignored and therefore
    cannot refresh an ACTIVE owner: a broken publisher will still expire fail-closed.
    """
    try:
        session_id = str(message.session_id)
        sequence = int(message.sequence)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
    active = bool(message.active)
    motion_allowed = bool(message.motion_allowed)
    return bool(
        session_id
        and session_id == session_id.strip()
        and len(session_id) <= MAX_LEASE_SESSION_LENGTH
        and not any(character.isspace() for character in session_id)
        and sequence > 0
        # A released owner cannot simultaneously grant autonomous motion.  Reject the
        # complete contradictory sample before its sequence reaches the high-water mark.
        and (active or not motion_allowed)
    )


def twist_components_are_finite(command: Twist) -> bool:
    """仅当 Twist 六个自由度均为有限数时返回真。

    ROS 消息类型不会禁止 NaN/Inf。即使四足底盘当前只消费 ``linear.x`` 和
    ``angular.z``，把其他损坏字段继续转发也会让未来的全向底盘、日志或仲裁器行为
    不确定，因此最终速度门必须把整条命令作为一个原子合同检查。异常命令直接归零，
    等待下一条新鲜且完整的 Twist，不能逐字段修补后继续运动。
    """
    return all(
        isfinite(float(value))
        for value in (
            command.linear.x,
            command.linear.y,
            command.linear.z,
            command.angular.x,
            command.angular.y,
            command.angular.z,
        )
    )


def twist_matches_differential_drive(
    command: Twist,
    unused_axis_tolerance: float = DIFFERENTIAL_UNUSED_AXIS_TOLERANCE,
) -> bool:
    """Accept only the planar differential-drive contract used by this stack.

    Nav2 is configured with ``linear.y == 0`` and the hardware adapter consumes only
    forward velocity plus yaw.  Passing a lateral, vertical, roll or pitch component
    through a front/rear laser sector would check the wrong direction.  Tiny serializer
    noise is tolerated but sanitized to zero by :func:`gated_twist`; any material value
    rejects the complete command.
    """
    if (
        not twist_components_are_finite(command)
        or not isfinite(float(unused_axis_tolerance))
        or float(unused_axis_tolerance) < 0.0
    ):
        return False
    tolerance = float(unused_axis_tolerance)
    return all(
        abs(float(value)) <= tolerance
        for value in (
            command.linear.y,
            command.linear.z,
            command.angular.x,
            command.angular.y,
        )
    )


def gated_twist(
    source: Twist,
    limit: float,
    command_fresh: bool,
    decision_fresh: bool,
    navigation_healthy: bool = True,
    health_fresh: bool = True,
    external_stop: bool = False,
    autonomy_authorized: bool = True,
) -> Twist:
    """仅在命令、地形评估和导航健康状态均有效时缩放 Twist。"""
    output = Twist()
    # 三条独立安全条件任一失效都输出默认构造的零 Twist。
    if (
        not command_fresh
        or not decision_fresh
        or not navigation_healthy
        or not health_fresh
        or external_stop
        or not autonomy_authorized
        or not twist_matches_differential_drive(source)
        or not isfinite(limit)
        or limit <= 0.0
    ):
        return output
    safe_limit = min(1.0, limit)
    output.linear.x = source.linear.x * safe_limit
    output.angular.z = source.angular.z * safe_limit
    return output


def alignment_twist(source: Twist, angular_limit: float) -> Twist:
    """生成只允许原地对正的命令。

    越障入口已经进入硬停车距离时，点云安全层会把普通导航限速置零。这时仍必须允许
    Nav2 的有限角速度把 ``base_link`` 对准障碍法向，否则引导状态会永久停在 ALIGN。
    该辅助函数故意丢弃全部线速度和 roll/pitch 角速度，不能被用于向障碍继续前进。
    """
    output = Twist()
    if (
        not twist_matches_differential_drive(source)
        or not isfinite(angular_limit)
        or angular_limit <= 0.0
    ):
        return output
    output.angular.z = max(
        -abs(float(angular_limit)),
        min(abs(float(angular_limit)), float(source.angular.z)),
    )
    return output


def is_pure_rotation_request(source: Twist, linear_tolerance: float = 0.12) -> bool:
    """识别 Nav2 的转向主导请求，供停车状态安全脱困。

    当前方障碍不具备稳定比赛语义时，引导层不会进入 ``ALIGN``；若速度门又把所有 yaw
    一并归零，机器人就永远保持同一视角，无法转身或返回。DWB 常把期望原地转向写成
    小线速度 + yaw（实测约 0.10 m/s + 0.20 rad/s）；这里允许该输入进入
    ``alignment_twist``，但输出会强制丢弃全部平移，只保留有界 yaw。后续仍经过导航
    健康、超时、外部停车和 360° 雷达急停，绝不允许借此继续靠近障碍。
    """
    if not twist_matches_differential_drive(source) or not isfinite(linear_tolerance):
        return False
    tolerance = max(0.0, float(linear_tolerance))
    return bool(
        abs(float(source.linear.x)) <= tolerance
        and abs(float(source.linear.y)) <= tolerance
        and abs(float(source.linear.z)) <= tolerance
        and abs(float(source.angular.z)) > 1e-4
    )


def has_finite_yaw_request(source: Twist, minimum_magnitude: float = 1e-3) -> bool:
    """仅接受数值有限且幅值有效的 Nav2 转向请求。

    任务层明确发送“允许转向恢复”的心跳时，DWB 原始命令仍可能夹带较大的前进分量。
    调用方可以读取其中的 yaw 意图，但最终必须交给 :func:`alignment_twist`；后者会
    清空所有平移分量，仅输出受限的原地转向，避免借恢复状态继续向障碍推进。
    """
    if (
        not twist_matches_differential_drive(source)
        or not isfinite(minimum_magnitude)
    ):
        return False
    yaw = float(source.angular.z)
    return abs(yaw) >= max(0.0, float(minimum_magnitude))


def scan_allows_command(
    scan: LaserScan | None,
    command: Twist,
    stop_distance: float,
    sector_half_angle: float,
    minimum_valid_ratio: float = DEFAULT_SCAN_MIN_VALID_RATIO,
    maximum_invalid_gap_angle: float = DEFAULT_SCAN_MAX_INVALID_GAP_ANGLE,
) -> bool:
    """Fail closed unless the complete swept sector has trustworthy ranges.

    A pure straight command checks its front or rear sector.  Any material yaw request,
    including a forward arc, checks the whole body sweep and therefore requires almost
    360-degree scan coverage.  A finite range is valid only inside the advertised
    ``[range_min, range_max]`` interval; standard positive ``Inf`` is also valid
    LaserScan evidence for "no return within range".  NaN, negative Inf and finite
    values above range_max remain invalid, so one far ray cannot hide a broken sector.
    Coverage, valid ratio and the longest angular hole are all checked before clearance.
    """
    if not twist_matches_differential_drive(command):
        return False
    linear = float(command.linear.x)
    angular = float(command.angular.z)
    if abs(linear) < 1e-6 and abs(angular) < 1e-6:
        return True
    if scan is None or not scan.ranges:
        return False
    metadata = (
        scan.angle_min,
        scan.angle_max,
        scan.angle_increment,
        scan.range_min,
        scan.range_max,
        stop_distance,
        sector_half_angle,
        minimum_valid_ratio,
        maximum_invalid_gap_angle,
    )
    if not all(isfinite(float(value)) for value in metadata):
        return False
    increment = float(scan.angle_increment)
    range_min = float(scan.range_min)
    range_max = float(scan.range_max)
    threshold = float(stop_distance)
    half_angle = float(sector_half_angle)
    minimum_ratio = float(minimum_valid_ratio)
    maximum_gap = float(maximum_invalid_gap_angle)
    if (
        increment <= 0.0
        or range_min < 0.0
        or range_max <= range_min
        or threshold <= 0.0
        or not 0.0 < half_angle <= pi
        or not 0.0 < minimum_ratio <= 1.0
        or not 0.0 < maximum_gap <= 0.35
        # Even nominally valid adjacent rays leave an unobserved angular interval.
        # A scan coarser than the permitted hole cannot prove continuous clearance.
        or increment > maximum_gap
    ):
        return False

    sample_count = len(scan.ranges)
    measured_last_angle = float(scan.angle_min) + (sample_count - 1) * increment
    declared_span = float(scan.angle_max) - float(scan.angle_min)
    metadata_tolerance = max(1e-6, 0.51 * increment)
    if (
        declared_span < 0.0
        or abs(measured_last_angle - float(scan.angle_max)) > metadata_tolerance
        or declared_span > 2.0 * pi + metadata_tolerance
    ):
        return False

    # Any yaw sweeps the body sideways.  Until footprint-aware arc clearance exists,
    # conservatively use the full scan instead of checking only the translation sector.
    check_all = abs(angular) >= 1e-4
    target_angle = 0.0 if linear >= 0.0 else pi
    selected = []
    for index, measured_range in enumerate(scan.ranges):
        distance = float(measured_range)
        angle = float(scan.angle_min) + index * float(scan.angle_increment)
        angle_error = (angle - target_angle + pi) % (2.0 * pi) - pi
        if check_all or abs(angle_error) <= half_angle:
            valid = bool(
                (isfinite(distance) and range_min <= distance <= range_max)
                or (distance == float("inf"))
            )
            selected.append((angle_error, valid, distance))

    if check_all:
        # A 270-degree scanner cannot prove that rotating the body is clear.  The blind
        # arc itself counts as an invalid angular gap and must fit the same conservative
        # bound as a run of invalid rays.
        uncovered_angle = max(0.0, 2.0 * pi - declared_span)
        if uncovered_angle > maximum_gap:
            return False
        # A scan that includes both -pi and +pi duplicates one direction; remove one
        # endpoint before ratio/circular-run accounting.
        if declared_span >= 2.0 * pi - metadata_tolerance and len(selected) > 1:
            selected = selected[:-1]
        ordered = selected
    else:
        # Sort wrapped rear-sector samples into continuous [-half_angle, +half_angle]
        # order, then require both sector edges to be represented by scan metadata.
        ordered = sorted(selected, key=lambda sample: sample[0])
        if (
            not ordered
            or ordered[0][0] > -half_angle + increment
            or ordered[-1][0] < half_angle - increment
        ):
            return False

    if not ordered:
        return False
    validity = [sample[1] for sample in ordered]
    valid_count = sum(validity)
    if valid_count / len(validity) < minimum_ratio:
        return False

    def longest_invalid_run(values, circular=False):
        run = longest = 0
        sequence = values + values if circular else values
        for valid in sequence:
            run = 0 if valid else run + 1
            longest = max(longest, run)
            if circular and longest >= len(values):
                return len(values)
        return min(longest, len(values))

    invalid_gap = longest_invalid_run(validity, circular=check_all) * increment
    if check_all:
        uncovered_angle = max(0.0, 2.0 * pi - declared_span)
        leading_invalid = 0
        for valid in validity:
            if valid:
                break
            leading_invalid += 1
        trailing_invalid = 0
        for valid in reversed(validity):
            if valid:
                break
            trailing_invalid += 1
        # End and start rays border the physical blind arc.  Their invalid runs and the
        # unobserved angle form one continuous hole, not three independent smaller gaps.
        boundary_gap = (
            leading_invalid + trailing_invalid
        ) * increment + uncovered_angle
        invalid_gap = max(invalid_gap, uncovered_angle, boundary_gap)
    if invalid_gap > maximum_gap:
        return False
    return not any(
        valid and distance <= threshold for _angle, valid, distance in ordered
    )


class NavigationSpeedGate(Node):
    """以 20 Hz 重算最终速度，任一安全输入超时即归零。"""

    def __init__(self, **node_kwargs):
        """建立三路安全心跳，并以固定频率发布经过门控的 Twist。

        这里保存消息接收时间而不是源 Header，因为 Twist/Float32/Bool 没有 Header。每次
        定时回调都重新检查超时，所以发布者退出后不会永久沿用最后一条非零命令。
        """
        super().__init__("navigation_speed_gate", **node_kwargs)
        self.declare_parameter("input_topic", "/cmd_vel_smoothed")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("assessment_timeout", 0.7)
        self.declare_parameter("navigation_health_timeout", 0.5)
        self.declare_parameter("require_navigation_health", True)
        self.declare_parameter("default_speed_limit", 0.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("emergency_stop_distance", 0.22)
        self.declare_parameter("emergency_sector_half_angle", 0.60)
        self.declare_parameter(
            "emergency_scan_min_valid_ratio", DEFAULT_SCAN_MIN_VALID_RATIO
        )
        self.declare_parameter(
            "emergency_scan_max_invalid_gap_angle",
            DEFAULT_SCAN_MAX_INVALID_GAP_ANGLE,
        )
        self.declare_parameter("require_emergency_scan", True)
        self.declare_parameter("alignment_guidance_timeout", 0.8)
        self.declare_parameter("alignment_max_angular_speed", 0.30)
        self.declare_parameter("stopped_rotation_linear_tolerance", 0.12)
        # /navigation/rotation_recovery 由独立自主任务以 4 Hz 刷新；若进程崩溃，
        # 0.8 秒后许可自动失效，不能留下永久旋转旁路。
        self.declare_parameter("rotation_recovery_timeout", 0.8)
        # 独立自主任务只发布许可心跳，不发送速度。任务进程崩溃、卡死或 Ctrl-C 后，
        # 最终速度门在该窗口内自动失效；手柄/键盘使用独立候选话题，不经过此自主门。
        self.declare_parameter("autonomy_lease_timeout", 0.8)

        validate_speed_gate_parameters(
            {name: self.get_parameter(name).value for name in SPEED_GATE_PARAMETER_NAMES}
        )

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        configured_command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        configured_assessment_timeout = float(
            self.get_parameter("assessment_timeout").value
        )
        self.command_timeout = (
            configured_command_timeout
            if isfinite(configured_command_timeout) and configured_command_timeout > 0.0
            else 0.5
        )
        self.assessment_timeout = (
            configured_assessment_timeout
            if isfinite(configured_assessment_timeout)
            and configured_assessment_timeout > 0.0
            else 0.7
        )
        configured_health_timeout = float(
            self.get_parameter("navigation_health_timeout").value
        )
        self.health_timeout = (
            configured_health_timeout
            if isfinite(configured_health_timeout)
            and configured_health_timeout > 0.0
            else 0.5
        )
        self.require_navigation_health = bool(
            self.get_parameter("require_navigation_health").value
        )
        self.scan_timeout = max(
            0.05, float(self.get_parameter("scan_timeout").value)
        )
        self.emergency_stop_distance = max(
            0.05, float(self.get_parameter("emergency_stop_distance").value)
        )
        self.emergency_sector_half_angle = min(
            pi,
            max(
                0.05,
                float(self.get_parameter("emergency_sector_half_angle").value),
            ),
        )
        self.emergency_scan_min_valid_ratio = float(
            self.get_parameter("emergency_scan_min_valid_ratio").value
        )
        self.emergency_scan_max_invalid_gap_angle = float(
            self.get_parameter("emergency_scan_max_invalid_gap_angle").value
        )
        self.require_emergency_scan = bool(
            self.get_parameter("require_emergency_scan").value
        )
        self.autonomy_lease_timeout = float(
            self.get_parameter("autonomy_lease_timeout").value
        )
        configured_limit = float(self.get_parameter("default_speed_limit").value)
        self.speed_limit = (
            max(0.0, min(1.0, configured_limit))
            if isfinite(configured_limit)
            else 0.0
        )
        self.latest_cmd = Twist()
        # ``/navigation/autonomy_stop`` remains a transient-local compatibility/diagnostic
        # veto.  Its anonymous false payload is never sufficient to unlock this gate; only
        # an ordered AutonomyLease permit or clean release may clear the latch.
        self.external_stop = False
        # 启动阶段没有任何输入可被视为新鲜。使用 None 而不是构造时刻，避免在首个
        # command/assessment 到来前出现一个仅由默认值决定的伪心跳窗口。
        self.last_cmd_time = None
        self.last_assessment_time = None
        self.navigation_healthy = not self.require_navigation_health
        self.last_health_time = None
        self.latest_scan = None
        self.last_scan_time = None
        self.alignment_requested = False
        self.last_guidance_time = None
        self.rotation_recovery_requested = False
        self.last_rotation_recovery_time = None
        # UNOWNED: 自主任务从未取得或已明确释放，普通 RViz/Nav2 Goal 可以工作。
        # ACTIVE: 自主任务拥有 Nav2，必须持续刷新 true。
        # EXPIRED: owner 消失，锁存停车；只有重启本速度门/核心栈才回到 UNOWNED。
        # 每个进程使用独立 session，严格递增 sequence。这样旧进程迟到的 false 不会
        # 释放新 owner，重复/回退的 heartbeat 也不能延长许可窗口。
        self.autonomy_lease_state = "UNOWNED"
        self.autonomy_lease_session = ""
        self.autonomy_lease_sequence = 0
        self.autonomy_motion_allowed = False
        self.autonomy_lease_high_watermarks = {}
        self.last_autonomy_lease_time = None
        self.last_clock_time = self.get_clock().now()

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        self.create_subscription(
            Float32, "/terrain/speed_limit", self.limit_callback, 10
        )
        self.create_subscription(
            Bool, "/navigation/healthy", self.health_callback, 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            # Gazebo 与常见雷达驱动通常使用传感器 QoS；显式 best-effort 可同时兼容
            # reliable 发布者，避免“话题存在但急停门收不到扫描”的静默故障。
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            "/navigation/autonomy_stop",
            self.autonomy_stop_callback,
            stop_qos,
        )
        # Lease 必须是 volatile 心跳：不能让前一次已退出任务的 transient-local true
        # 在下一次 SLAM 启动后继续授权。只有新进程持续刷新 true 才能放行 Nav2 分支。
        self.create_subscription(
            AutonomyLease,
            "/navigation/autonomy_lease",
            self.autonomy_lease_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            TraversalGuidance,
            "/traversal/guidance",
            self.guidance_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/navigation/rotation_recovery",
            self.rotation_recovery_callback,
            10,
        )
        self.timer = self.create_timer(0.05, self.publish_safe_command)
        self.get_logger().info(f"Velocity gate: {input_topic} -> {output_topic}")

    def cmd_callback(self, msg: Twist) -> None:
        """缓存 Nav2 最新速度及本机接收时刻。"""
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def limit_callback(self, msg: Float32) -> None:
        """接收 0～1 速度上限；NaN/Inf 按零处理。"""
        value = float(msg.data)
        self.speed_limit = max(0.0, min(1.0, value)) if isfinite(value) else 0.0
        self.last_assessment_time = self.get_clock().now()

    def health_callback(self, msg: Bool) -> None:
        """缓存导航健康心跳；false 或断流都会关闭非零速度输出。"""
        self.navigation_healthy = bool(msg.data)
        self.last_health_time = self.get_clock().now()

    def autonomy_stop_callback(self, msg: Bool) -> None:
        """Apply the legacy anonymous topic only as an additional stop veto.

        A delayed ``false`` from an old mission process cannot identify which session it
        belongs to.  It is therefore ignored: an ordered same-session
        :class:`AutonomyLease` motion permit is the only autonomous unlock boundary.
        ``true`` remains useful to dashboards and older launch helpers and always clears
        cached Nav2 intent immediately.  Keyboard/gamepad candidates bypass this node.
        """
        if bool(msg.data):
            self._clear_motion_intent()
            self.external_stop = True

    def autonomy_lease_callback(self, msg: AutonomyLease) -> None:
        """Apply one session-scoped autonomous ownership heartbeat.

        No owner means ordinary RViz/Nav2 goals remain available without a heartbeat.
        ACTIVE accepts only a strictly newer message from the exact current session;
        another process therefore cannot release or silently replace it.  High-water
        marks are retained after a clean release so delayed heartbeats cannot reacquire
        UNOWNED.  Once an owner expires, no message can prove its remote Action stopped,
        so EXPIRED remains latched until the speed gate/core stack is restarted.
        """
        if not autonomy_lease_is_well_formed(msg):
            return
        session_id = str(msg.session_id)
        sequence = int(msg.sequence)
        active = bool(msg.active)
        motion_allowed = bool(msg.motion_allowed)
        previous = self.autonomy_lease_state
        now = self.get_clock().now()

        # Expiry is evaluated before interpreting even a correctly formed release.  A
        # late packet cannot retroactively turn unknown remote ownership into a clean one.
        if previous == "ACTIVE" and not ros_age_is_fresh(
            now, self.last_autonomy_lease_time, self.autonomy_lease_timeout
        ):
            self.autonomy_lease_state = "EXPIRED"
            self.autonomy_motion_allowed = False
            self._clear_motion_intent()
            self.last_autonomy_lease_time = None
            previous = "EXPIRED"
        if previous == "EXPIRED":
            return

        previous_sequence = int(
            self.autonomy_lease_high_watermarks.get(session_id, 0)
        )
        if sequence <= previous_sequence:
            # Duplicate or reordered packets neither refresh nor release an owner.
            return
        self.autonomy_lease_high_watermarks[session_id] = sequence

        if previous == "UNOWNED":
            if not active:
                # Remember the release sequence to reject older delayed heartbeats, but
                # do not manufacture ownership for a session that was never active here.
                return
            self._clear_motion_intent()
            self.autonomy_lease_state = "ACTIVE"
            self.autonomy_lease_session = session_id
            self.autonomy_lease_sequence = sequence
            # First contact proves identity/ownership only.  Requiring a second ordered
            # sample prevents a delayed one-shot ``motion_allowed=true`` from an old
            # discovery epoch from both acquiring and moving in a single packet.
            self.autonomy_motion_allowed = False
            self.last_autonomy_lease_time = now
            return

        # ACTIVE is single-owner: a different session is remembered only for ordering
        # and otherwise ignored.  In particular, old_session:false cannot release the
        # current process during a fast restart or overlapping DDS discovery window.
        if session_id != self.autonomy_lease_session:
            return
        self.autonomy_lease_sequence = sequence
        if not active:
            self._clear_motion_intent()
            self.autonomy_lease_state = "UNOWNED"
            self.autonomy_lease_session = ""
            self.autonomy_motion_allowed = False
            self.last_autonomy_lease_time = None
            # This is an authenticated clean release.  Ordinary RViz/Nav2 may use the
            # now-unowned branch after producing a fresh Twist.
            self.external_stop = False
            return
        if motion_allowed != self.autonomy_motion_allowed:
            # Both stop and permit edges drop the previous controller sample.  A newly
            # permitted goal must produce a Twist after its accepted/result-monitored
            # Action boundary; a revoked goal cannot coast on its last cached sample.
            self._clear_motion_intent()
        self.autonomy_motion_allowed = motion_allowed
        if motion_allowed:
            # Clear a diagnostic stop only through this authenticated ordered permit.
            # Anonymous ``autonomy_stop=false`` packets never reach this transition.
            self.external_stop = False
        self.last_autonomy_lease_time = now

    def scan_callback(self, msg: LaserScan) -> None:
        """保存最新扫描和接收时刻；具体扇区由当前运动方向决定。"""
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

    def guidance_callback(self, msg: TraversalGuidance) -> None:
        """仅在点云有效、确需越障且处于 ALIGN 阶段时请求原地对正权限。"""
        self.alignment_requested = bool(
            msg.perception_valid
            and msg.traversal_required
            and msg.phase == TraversalGuidance.PHASE_ALIGN
            and not msg.ready_for_handoff
        )
        self.last_guidance_time = self.get_clock().now()

    def rotation_recovery_callback(self, msg: Bool) -> None:
        """缓存任务层允许“仅转向恢复”的心跳。

        探索、返航、换视角或接近障碍时，任务层可以短时置位该信号。它不是通用速度
        旁路：``publish_safe_command`` 始终通过 :func:`alignment_twist` 生成输出，
        因而全部平移分量都会被清零。
        """
        self.rotation_recovery_requested = bool(msg.data)
        self.last_rotation_recovery_time = self.get_clock().now()

    def _clear_motion_intent(self) -> None:
        """Drop every cached Nav2 intent that could produce motion after re-authorization."""
        self.latest_cmd = Twist()
        self.last_cmd_time = None
        self.alignment_requested = False
        self.last_guidance_time = None
        self.rotation_recovery_requested = False
        self.last_rotation_recovery_time = None

    def _reset_after_clock_rewind(self, now) -> None:
        """Invalidate all state whose freshness belonged to the previous ROS epoch."""
        self._clear_motion_intent()
        self.last_assessment_time = None
        self.navigation_healthy = not self.require_navigation_health
        self.last_health_time = None
        self.latest_scan = None
        self.last_scan_time = None
        if self.autonomy_lease_state != "UNOWNED":
            self.autonomy_lease_state = "EXPIRED"
            self.autonomy_motion_allowed = False
        else:
            self.autonomy_lease_state = "UNOWNED"
            self.autonomy_lease_session = ""
            self.autonomy_lease_sequence = 0
            self.autonomy_motion_allowed = False
        self.last_autonomy_lease_time = None
        self.last_clock_time = now
        self.get_logger().warning(
            "ROS clock moved backward; cleared velocity-gate heartbeats and cached Twist"
        )

    def publish_safe_command(self) -> None:
        """依据本机 ROS 时钟计算心跳年龄并始终发布一条明确命令。"""
        now = self.get_clock().now()
        if ros_clock_moved_backward(now, self.last_clock_time):
            self._reset_after_clock_rewind(now)
            self.pub.publish(Twist())
            return
        self.last_clock_time = now
        command_fresh = ros_age_is_fresh(now, self.last_cmd_time, self.command_timeout)
        assessment_fresh = ros_age_is_fresh(
            now, self.last_assessment_time, self.assessment_timeout
        )
        health_fresh = ros_age_is_fresh(now, self.last_health_time, self.health_timeout)
        scan_fresh = ros_age_is_fresh(now, self.last_scan_time, self.scan_timeout)
        alignment_fresh = ros_age_is_fresh(
            now,
            self.last_guidance_time,
            max(0.1, float(self.get_parameter("alignment_guidance_timeout").value)),
        )
        rotation_recovery_fresh = ros_age_is_fresh(
            now,
            self.last_rotation_recovery_time,
            max(0.1, float(self.get_parameter("rotation_recovery_timeout").value)),
        )
        lease_fresh = ros_age_is_fresh(
            now, self.last_autonomy_lease_time, self.autonomy_lease_timeout
        )
        if self.autonomy_lease_state == "ACTIVE" and not lease_fresh:
            self.autonomy_lease_state = "EXPIRED"
            self.autonomy_motion_allowed = False
            self._clear_motion_intent()
        autonomy_authorized = bool(
            self.autonomy_lease_state == "UNOWNED"
            or (
                self.autonomy_lease_state == "ACTIVE"
                and self.autonomy_motion_allowed
            )
        )
        # 每 50 ms 重新计算，而不是沿用上一条非零速度，防止失联后继续走。
        output = gated_twist(
            self.latest_cmd,
            self.speed_limit,
            command_fresh,
            assessment_fresh,
            self.navigation_healthy,
            not self.require_navigation_health
            or health_fresh,
            self.external_stop,
            autonomy_authorized,
        )
        # 硬停车只负责禁止继续接近障碍；明确 ALIGN 或 Nav2 的纯原地转向仍保留一个
        # 有界 yaw。后者用于从“未知但很近的轮廓”转身换视角，否则分类不稳定时会形成
        # STOP→无法旋转→永远无法重新分类的闭环。健康、超时、外部停车仍具有否决权。
        safe_rotation_requested = is_pure_rotation_request(
            self.latest_cmd,
            float(self.get_parameter("stopped_rotation_linear_tolerance").value),
        )
        rotation_recovery_requested = bool(
            self.rotation_recovery_requested
            and rotation_recovery_fresh
            and has_finite_yaw_request(self.latest_cmd)
        )
        if (
            self.speed_limit <= 0.0
            and (
                (self.alignment_requested and alignment_fresh)
                or safe_rotation_requested
                or rotation_recovery_requested
            )
            and command_fresh
            and assessment_fresh
            and self.navigation_healthy
            and (not self.require_navigation_health or health_fresh)
            and not self.external_stop
            and autonomy_authorized
        ):
            output = alignment_twist(
                self.latest_cmd,
                float(self.get_parameter("alignment_max_angular_speed").value),
            )
        # 导航健康监控负责完整的 scan/odom/TF 校验；这里再保留一个局部、可解释的最终
        # 防撞条件。扫描断流或命令方向 22 cm 内有物体时，只把本周期输出置零。
        if self.require_emergency_scan and (
            not scan_fresh
            or not scan_allows_command(
                self.latest_scan,
                output,
                self.emergency_stop_distance,
                self.emergency_sector_half_angle,
                self.emergency_scan_min_valid_ratio,
                self.emergency_scan_max_invalid_gap_angle,
            )
        ):
            output = Twist()
        self.pub.publish(output)


def main(args=None):
    """启动 Nav2 速度安全门。"""
    rclpy.init(args=args)
    node = NavigationSpeedGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

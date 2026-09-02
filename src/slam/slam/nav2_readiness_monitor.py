"""在最小传感器与定位 TF 就绪后才启动 Nav2 生命周期节点。"""

import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from slam.navigation_health_monitor import (
    OdometryJumpFilter,
    odometry_is_valid,
    odometry_yaw,
    scan_contract_is_valid,
    scan_is_valid,
    source_stamp_is_current,
    transform_stamp_is_current,
)
from slam.parameter_validation import (
    READINESS_PARAMETER_NAMES,
    validate_nav2_readiness_parameters,
)


def slam_transition_for_state(state_id: int):
    """Return the safe next SLAM lifecycle transition, or ``None``.

    ``slam_toolbox`` can occasionally remain unconfigured/inactive when a complete
    Gazebo + SLAM stack is stopped and restarted quickly.  Nav2 must not be started
    without ``map -> odom``, but simply waiting for that TF creates a permanent
    startup deadlock.  This small pure function deliberately permits only the two
    forward startup transitions; it never cleans up, shuts down or otherwise takes
    ownership of an already active external localization source.
    """
    if int(state_id) == State.PRIMARY_STATE_UNCONFIGURED:
        return Transition.TRANSITION_CONFIGURE
    if int(state_id) == State.PRIMARY_STATE_INACTIVE:
        return Transition.TRANSITION_ACTIVATE
    return None


class Nav2ReadinessMonitor(Node):
    """把 Nav2 激活条件集中到一个节点，避免各服务器在输入缺失时反复报错。"""

    def __init__(self, **node_kwargs):
        """订阅雷达/里程计心跳并建立生命周期管理服务客户端。"""
        super().__init__("nav2_readiness_monitor", **node_kwargs)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("sensor_timeout", 1.0)
        self.declare_parameter("future_stamp_tolerance", 0.10)
        self.declare_parameter("minimum_scan_valid_ratio", 0.60)
        self.declare_parameter("minimum_scan_samples", 90)
        self.declare_parameter("minimum_scan_field_of_view", 3.14)
        self.declare_parameter("max_xy_covariance", 1.0)
        self.declare_parameter("max_yaw_covariance", 1.0)
        self.declare_parameter("max_odom_jump", 0.75)
        self.declare_parameter("max_odom_yaw_jump", 0.75)
        self.declare_parameter("odom_jump_recovery_samples", 3)
        self.declare_parameter("expected_odom_frame", "odom")
        self.declare_parameter(
            "lifecycle_service",
            "/lifecycle_manager_navigation/manage_nodes",
        )
        self.declare_parameter("recover_slam_toolbox", True)
        self.declare_parameter("slam_lifecycle_node", "/slam_toolbox")
        self.declare_parameter("slam_recovery_period", 2.0)
        self.declare_parameter("slam_recovery_startup_grace", 4.0)
        # ROS service futures have no built-in response deadline.  A lifecycle manager
        # which disappears after discovery must therefore be bounded here, otherwise
        # one never-completing future can hold Nav2 inactive until the process restarts.
        # A first Nav2 bringup configures costmaps, plugins and five lifecycle bonds.  On this
        # machine that valid transaction takes about 2.2 s, so a 2 s deadline races the successful
        # response and then repeatedly sends STARTUP to nodes which are already active.
        self.declare_parameter("service_request_timeout", 5.0)
        # Readiness owns lifecycle activation, so invalid topics/frames must stop here rather
        # than leave Nav2 waiting forever with a misleading "missing input" message.
        validate_nav2_readiness_parameters(
            {name: self.get_parameter(name).value for name in READINESS_PARAMETER_NAMES}
        )
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        service_name = str(self.get_parameter("lifecycle_service").value)
        slam_lifecycle_node = str(
            self.get_parameter("slam_lifecycle_node").value
        ).rstrip("/")
        self.recover_slam_toolbox = bool(
            self.get_parameter("recover_slam_toolbox").value
        )
        self.slam_recovery_period = max(
            0.5, float(self.get_parameter("slam_recovery_period").value)
        )
        self.slam_recovery_startup_grace = max(
            0.0, float(self.get_parameter("slam_recovery_startup_grace").value)
        )
        self.service_request_timeout = max(
            0.1, float(self.get_parameter("service_request_timeout").value)
        )
        self.sensor_timeout = max(
            0.1, float(self.get_parameter("sensor_timeout").value)
        )
        self.future_stamp_tolerance = max(
            0.0, float(self.get_parameter("future_stamp_tolerance").value)
        )
        self.minimum_scan_valid_ratio = min(
            1.0,
            max(0.0, float(self.get_parameter("minimum_scan_valid_ratio").value)),
        )
        self.minimum_scan_samples = max(
            2, int(self.get_parameter("minimum_scan_samples").value)
        )
        self.minimum_scan_fov = max(
            0.0, float(self.get_parameter("minimum_scan_field_of_view").value)
        )
        self.max_xy_covariance = max(
            0.0, float(self.get_parameter("max_xy_covariance").value)
        )
        self.max_yaw_covariance = max(
            0.0, float(self.get_parameter("max_yaw_covariance").value)
        )
        self.expected_odom_frame = str(
            self.get_parameter("expected_odom_frame").value
        )

        self.scan_received = False
        self.odom_received = False
        self.scan_valid = False
        self.odom_valid = False
        self.odom_jump_filter = OdometryJumpFilter(
            float(self.get_parameter("max_odom_jump").value),
            int(self.get_parameter("odom_jump_recovery_samples").value),
            float(self.get_parameter("max_odom_yaw_jump").value),
        )
        self.odom_jump = False
        self.last_scan_time = None
        self.last_odom_time = None
        self.last_scan_source_stamp = None
        self.last_odom_source_stamp = None
        self.startup_requested = False
        self.startup_complete = False
        self.startup_request_generation = 0
        self.startup_request_deadline = 0.0
        self.startup_request_future = None
        self.slam_recovery_pending = False
        self.slam_recovery_generation = 0
        self.slam_recovery_deadline = 0.0
        self.slam_recovery_future = None
        self.last_slam_recovery_time = None
        # 启动宽限必须使用墙钟。仿真 /clock 可能在节点刚创建时已经运行数分钟，若用
        # ROS 时间会把启动年龄误算成数分钟并立刻与 launch 自带生命周期事件竞争。
        self.node_started_monotonic = time.monotonic()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.lifecycle_client = self.create_client(
            ManageLifecycleNodes, service_name
        )
        self.slam_get_state_client = self.create_client(
            GetState, f"{slam_lifecycle_node}/get_state"
        )
        self.slam_change_state_client = self.create_client(
            ChangeState, f"{slam_lifecycle_node}/change_state"
        )
        self.create_timer(0.5, self._check_readiness)
        self.get_logger().info(
            "Nav2 is held inactive until scan, odometry and localization TF "
            "are ready"
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        """记录激光心跳，并用健康监控的同一合同校验该帧是否足以建图。

        “收到 DDS 消息”与“可以激活 Nav2”不是同一件事。空 frame、过期 Header、零角
        增量或绝大多数无效回波都会保持 ``scan_valid=False``，避免生命周期节点在坏
        驱动上启动后持续刷错。
        """
        now = self.get_clock().now()
        self.scan_received = True
        self.last_scan_time = now
        self.last_scan_source_stamp = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )
        self.scan_valid = (
            source_stamp_is_current(
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                now.nanoseconds * 1e-9,
                self.sensor_timeout,
                self.future_stamp_tolerance,
            )
            and scan_contract_is_valid(
                msg, self.minimum_scan_samples, self.minimum_scan_fov
            )
            and scan_is_valid(
                msg.ranges,
                self.minimum_scan_valid_ratio,
                msg.range_min,
                msg.range_max,
            )
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """记录里程计心跳，同时校验时间、frame、四元数和协方差。"""
        now = self.get_clock().now()
        self.odom_received = True
        self.last_odom_time = now
        self.last_odom_source_stamp = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )
        self.odom_valid = source_stamp_is_current(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            now.nanoseconds * 1e-9,
            self.sensor_timeout,
            self.future_stamp_tolerance,
        ) and odometry_is_valid(
            msg,
            self.max_xy_covariance,
            self.expected_odom_frame,
            self.base_frame,
            self.max_yaw_covariance,
        )
        self.odom_jump = self.odom_jump_filter.update(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            self.odom_valid,
            odometry_yaw(msg),
        )

    def _sensor_is_fresh(self, receipt_stamp, source_stamp=None) -> bool:
        """按 ROS 时钟同时检查 DDS 心跳和消息 Header 的当前年龄。

        Header 在回调时有效不代表半秒后的生命周期检查仍有效。每个周期重算源年龄可
        拒绝晚到或重复的缓存帧；接收时刻则独立拒绝完全断流的驱动。
        """
        if receipt_stamp is None:
            return False
        now = self.get_clock().now()
        age = (now - receipt_stamp).nanoseconds / 1e9
        if not 0.0 <= age <= self.sensor_timeout:
            return False
        if source_stamp is None:
            return True
        return source_stamp_is_current(
            source_stamp[0],
            source_stamp[1],
            now.nanoseconds * 1e-9,
            self.sensor_timeout,
            self.future_stamp_tolerance,
        )

    @staticmethod
    def _cancel_pending_future(future) -> None:
        """Best-effort cancel a client future without depending on RMW behavior."""
        if future is None:
            return
        try:
            if future.done():
                return
            future.cancel()
        except Exception:
            # Some client implementations cannot recall a request already sent to the
            # server.  Generation checks below still isolate its eventual late reply.
            pass

    def _invalidate_startup_request(self, future=None) -> None:
        """Invalidate one startup generation and release its retry guard."""
        stale = self.startup_request_future if future is None else future
        self.startup_request_generation += 1
        self.startup_request_future = None
        self.startup_request_deadline = 0.0
        self.startup_requested = False
        self._cancel_pending_future(stale)

    def _invalidate_slam_recovery_request(self, future=None) -> None:
        """Invalidate one SLAM recovery generation and release its retry guard."""
        stale = self.slam_recovery_future if future is None else future
        self.slam_recovery_generation += 1
        self._finish_slam_recovery_request()
        self._cancel_pending_future(stale)

    def _expire_service_requests(self, now_monotonic=None) -> None:
        """Release lifecycle guards whose asynchronous response deadline elapsed.

        Deadlines use wall time because a paused bag or Gazebo clock must not prevent a
        DDS/service liveness timeout.  Incrementing the generation *before* cancellation
        makes callbacks from an old request harmless even when the middleware delivers a
        response after a retry has already begun.
        """
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        if (
            self.startup_requested
            and self.startup_request_deadline > 0.0
            and now >= self.startup_request_deadline
        ):
            self._invalidate_startup_request()
            self.get_logger().error(
                "Nav2 startup service timed out; readiness will retry safely"
            )
        if (
            self.slam_recovery_pending
            and self.slam_recovery_deadline > 0.0
            and now >= self.slam_recovery_deadline
        ):
            self._invalidate_slam_recovery_request()
            self.get_logger().warning(
                "slam_toolbox lifecycle service timed out; recovery will retry"
            )

    def _request_nav2_startup(self) -> bool:
        """Send one bounded STARTUP request; return whether it was registered."""
        self.startup_requested = True
        self.startup_request_generation += 1
        generation = self.startup_request_generation
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        try:
            future = self.lifecycle_client.call_async(request)
        except Exception as exc:
            self.startup_requested = False
            self.startup_request_deadline = 0.0
            self.startup_request_future = None
            self.get_logger().error(f"Unable to send Nav2 startup request: {exc}")
            return False
        self.startup_request_future = future
        self.startup_request_deadline = (
            time.monotonic() + self.service_request_timeout
        )
        try:
            future.add_done_callback(
                lambda completed, request_generation=generation: self._startup_response(
                    completed, request_generation
                )
            )
        except Exception as exc:
            # A request may already be in flight even though callback registration
            # failed.  Invalidate its generation before cancellation/retry.
            self._invalidate_startup_request(future)
            self.get_logger().error(
                f"Unable to monitor Nav2 startup request: {exc}"
            )
            return False
        return True

    def _check_readiness(self) -> None:
        """仅在 scan、odom 和 map→base_link TF 同时就绪时启动 Nav2。"""
        self._expire_service_requests()
        if self.startup_requested:
            return
        # 不只检查“曾经收到”，还检查传感器正在持续更新。
        scan_ready = self.scan_valid and self._sensor_is_fresh(
            self.last_scan_time, self.last_scan_source_stamp
        )
        odom_ready = (
            self.odom_valid
            and not self.odom_jump
            and self._sensor_is_fresh(
                self.last_odom_time, self.last_odom_source_stamp
            )
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            now_seconds = self.get_clock().now().nanoseconds * 1e-9
            tf_ready = transform_stamp_is_current(
                transform.header.stamp.sec,
                transform.header.stamp.nanosec,
                now_seconds,
                self.sensor_timeout,
                self.future_stamp_tolerance,
            )
        except TransformException:
            tf_ready = False
        # Only inspect SLAM after both physical inputs are healthy.  If an external
        # localization stack is used and no slam_toolbox service exists this branch
        # is a harmless no-op; the normal TF readiness contract remains unchanged.
        if scan_ready and odom_ready and not tf_ready:
            self._recover_slam_if_needed()
        if not (scan_ready and odom_ready and tf_ready):
            missing = []
            if not scan_ready:
                missing.append("fresh scan")
            if not odom_ready:
                missing.append("fresh odom")
            if not tf_ready:
                missing.append(f"{self.global_frame}->{self.base_frame} TF")
            self.get_logger().info(
                "Waiting before Nav2 activation: " + ", ".join(missing),
                throttle_duration_sec=5.0,
            )
            return
        if not self.lifecycle_client.service_is_ready():
            self.get_logger().info(
                "Waiting for Nav2 lifecycle manager service",
                throttle_duration_sec=5.0,
            )
            return
        # 由 lifecycle manager 按固定顺序配置和激活全部 Nav2 节点。
        if self._request_nav2_startup():
            self.get_logger().info("Inputs ready; requesting Nav2 activation")

    def _recover_slam_if_needed(self) -> None:
        """Advance a stalled local ``slam_toolbox`` to active, one step at a time."""
        if not self.recover_slam_toolbox or self.slam_recovery_pending:
            return
        now = self.get_clock().now()
        # 正常启动本来就需要短暂 configure/activate；过早查询会与 launch 自带的
        # lifecycle 事件竞争并在 slam_toolbox 端留下无意义的 service timeout 警告。
        startup_age = time.monotonic() - self.node_started_monotonic
        if startup_age < self.slam_recovery_startup_grace:
            return
        if self.last_slam_recovery_time is not None:
            age = (now - self.last_slam_recovery_time).nanoseconds / 1e9
            if age < self.slam_recovery_period:
                return
        if not self.slam_get_state_client.service_is_ready():
            return
        self.slam_recovery_pending = True
        self.last_slam_recovery_time = now
        self.slam_recovery_generation += 1
        generation = self.slam_recovery_generation
        try:
            future = self.slam_get_state_client.call_async(GetState.Request())
        except Exception as exc:
            self._finish_slam_recovery_request()
            self.get_logger().warning(
                f"Unable to send slam_toolbox state request: {exc}"
            )
            return
        self.slam_recovery_future = future
        self.slam_recovery_deadline = (
            time.monotonic() + self.service_request_timeout
        )
        try:
            future.add_done_callback(
                lambda completed, request_generation=generation: self._slam_state_response(
                    completed, request_generation
                )
            )
        except Exception as exc:
            self._invalidate_slam_recovery_request(future)
            self.get_logger().warning(
                f"Unable to monitor slam_toolbox state request: {exc}"
            )

    def _slam_state_response(self, future, generation=None) -> None:
        """Read lifecycle state and request only the next safe startup transition."""
        if generation is not None and generation != self.slam_recovery_generation:
            return
        if (
            self.slam_recovery_deadline > 0.0
            and time.monotonic() >= self.slam_recovery_deadline
        ):
            self._invalidate_slam_recovery_request(future)
            self.get_logger().warning(
                "Ignoring late slam_toolbox state response after its deadline"
            )
            return
        try:
            response = future.result()
            transition_id = slam_transition_for_state(response.current_state.id)
        except Exception as exc:
            self._finish_slam_recovery_request()
            self.get_logger().warning(f"Unable to inspect slam_toolbox state: {exc}")
            return
        if transition_id is None:
            self._finish_slam_recovery_request()
            return
        if not self.slam_change_state_client.service_is_ready():
            self._finish_slam_recovery_request()
            return
        request = ChangeState.Request()
        request.transition.id = int(transition_id)
        try:
            future = self.slam_change_state_client.call_async(request)
        except Exception as exc:
            self._finish_slam_recovery_request()
            self.get_logger().warning(
                f"Unable to send slam_toolbox transition request: {exc}"
            )
            return
        self.slam_recovery_future = future
        self.slam_recovery_deadline = (
            time.monotonic() + self.service_request_timeout
        )
        active_generation = self.slam_recovery_generation
        try:
            future.add_done_callback(
                lambda completed, request_generation=active_generation: (
                    self._slam_transition_response(completed, request_generation)
                )
            )
        except Exception as exc:
            self._invalidate_slam_recovery_request(future)
            self.get_logger().warning(
                f"Unable to monitor slam_toolbox transition request: {exc}"
            )
            return
        transition_name = (
            "configure"
            if transition_id == Transition.TRANSITION_CONFIGURE
            else "activate"
        )
        self.get_logger().warning(
            f"map TF is absent; requesting slam_toolbox {transition_name} recovery"
        )

    def _finish_slam_recovery_request(self) -> None:
        """Clear the currently active SLAM request without changing its generation."""
        self.slam_recovery_future = None
        self.slam_recovery_deadline = 0.0
        self.slam_recovery_pending = False

    def _slam_transition_response(self, future, generation=None) -> None:
        """Release the recovery guard so the next timer can verify actual state."""
        if generation is not None and generation != self.slam_recovery_generation:
            return
        if (
            self.slam_recovery_deadline > 0.0
            and time.monotonic() >= self.slam_recovery_deadline
        ):
            self._invalidate_slam_recovery_request(future)
            self.get_logger().warning(
                "Ignoring late slam_toolbox transition response after its deadline"
            )
            return
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("slam_toolbox lifecycle recovery was rejected")
        except Exception as exc:
            self.get_logger().error(f"slam_toolbox lifecycle recovery failed: {exc}")
        finally:
            self._finish_slam_recovery_request()

    def _startup_response(self, future, generation=None) -> None:
        """Handle one lifecycle STARTUP result without retrying an active stack.

        A completed ``success=True`` response is authoritative evidence that the lifecycle manager
        activated every managed node.  Accept it even when it arrives a few milliseconds beyond
        the local wall deadline, provided its generation is still current.  The timer invalidates
        and increments the generation before issuing any retry, so a truly stale response can
        never overwrite a newer transaction.  This distinction avoids the former failure mode:
        a valid 2.2 s initial activation lost a 2.0 s race and STARTUP was then spammed forever at
        already-active nodes.
        """
        if generation is not None and generation != self.startup_request_generation:
            return
        deadline_expired = bool(
            self.startup_request_deadline > 0.0
            and time.monotonic() >= self.startup_request_deadline
        )
        self.startup_request_future = None
        self.startup_request_deadline = 0.0
        try:
            response = future.result()
        except Exception as exc:
            self.startup_requested = False
            self.get_logger().error(f"Nav2 startup request failed: {exc}")
            return
        if response.success:
            self.startup_complete = True
            self.get_logger().info("Nav2 activated successfully")
            return
        if deadline_expired:
            # No newer generation exists, but a late negative result still proves nothing became
            # active.  Release the guard and retry after the normal readiness period.
            self.startup_requested = False
            self.get_logger().error(
                "Late Nav2 startup response reported failure; readiness will retry"
            )
            return
        if not response.success:
            self.startup_requested = False
            self.get_logger().error("Nav2 lifecycle manager rejected startup")
            return


def main(args=None):
    """运行传感器与定位就绪监控节点。"""
    rclpy.init(args=args)
    node = Nav2ReadinessMonitor()
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

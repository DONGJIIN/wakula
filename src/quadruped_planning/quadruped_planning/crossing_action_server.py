"""在上层 ROS 2 Action 与可替换的真机步态控制器之间建立严格合同。

Action 负责目标互斥、取消、双重超时、反馈和最终结果；底层 SDK 适配器负责执行腿部
动作，并通过同一 goal UUID 回报状态。服务器不会因为收到一个字符串“完成”就成功，
而是要求合法状态、单调进度以及可配置的触地确认。
"""

import math
import threading
import time
from typing import Tuple

import rclpy
from quadruped_interfaces.action import TraverseObstacle
from quadruped_interfaces.msg import CrossingCommand, CrossingStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


MODE_NAMES = {
    TraverseObstacle.Goal.STEP: "STEP",
    TraverseObstacle.Goal.CLIMB: "CLIMB",
    TraverseObstacle.Goal.LOW_PROFILE: "LOW_PROFILE",
}
VALID_STATUS_STATES = {
    CrossingStatus.RUNNING,
    CrossingStatus.SUCCEEDED,
    CrossingStatus.FAILED,
    CrossingStatus.CANCELED,
}
VALID_FEEDBACK_PHASES = {
    CrossingStatus.ACCEPTED,
    CrossingStatus.PREPARING,
    CrossingStatus.EXECUTING,
    CrossingStatus.VERIFYING_CONTACT,
    CrossingStatus.RECOVERING,
}


def validate_goal_values(
    mode: int,
    obstacle_height: float,
    obstacle_distance: float,
    speed_scale: float,
    timeout: float,
    maximum_timeout: float,
) -> Tuple[bool, str]:
    """在进入执行器前校验 Goal 数值，使错误请求不会触达真机控制器。"""
    values = (obstacle_height, obstacle_distance, speed_scale, timeout)
    if mode not in MODE_NAMES:
        return False, "unsupported crossing mode"
    if not all(math.isfinite(value) for value in values):
        return False, "goal contains a non-finite value"
    if obstacle_height < 0.0 or obstacle_distance < 0.0:
        return False, "height and distance must be non-negative"
    if not 0.0 < speed_scale <= 1.0:
        return False, "speed_scale must be in (0, 1]"
    if not 0.1 <= timeout <= maximum_timeout:
        return False, f"timeout must be in [0.1, {maximum_timeout:.1f}] seconds"
    return True, ""


def validate_controller_status(
    state: int, phase: int, progress: float, previous_progress: float
) -> Tuple[bool, str]:
    """拒绝非法、非有限或进度倒退的控制器状态。

    FAILED/CANCELED 允许进度低于上一帧，因为某些控制器会在终止时清零；终止状态
    仍必须被上层及时接收，不能因“进度倒退”被忽略后误判为心跳超时。
    """
    if state not in VALID_STATUS_STATES:
        return False, "unknown controller state"
    if phase not in VALID_FEEDBACK_PHASES:
        return False, "unknown feedback phase"
    if not math.isfinite(progress) or not 0.0 <= progress <= 1.0:
        return False, "progress must be finite and in [0, 1]"
    if state in (CrossingStatus.RUNNING, CrossingStatus.SUCCEEDED) and (
        progress + 1e-6 < previous_progress
    ):
        return False, "progress regressed"
    return True, ""


def controller_success_is_valid(
    progress: float,
    contact_verified: bool,
    minimum_progress: float,
    require_contact: bool,
) -> bool:
    """Require explicit completion evidence before accepting controller success."""
    return progress >= minimum_progress and (
        contact_verified or not require_contact
    )


class CrossingActionServer(Node):
    """管理单个活动 Goal 的完整生命周期，腿部执行由硬件适配器负责。"""

    def __init__(self):
        super().__init__("crossing_action_server")
        self.declare_parameter("action_name", "/crossing/traverse_obstacle")
        self.declare_parameter("command_topic", "/crossing/execution_command")
        self.declare_parameter("status_topic", "/crossing/execution_status")
        self.declare_parameter("legacy_request_topic", "/crossing/action_request")
        self.declare_parameter("maximum_goal_timeout", 60.0)
        self.declare_parameter("controller_status_timeout", 1.5)
        self.declare_parameter("minimum_success_progress", 0.95)
        self.declare_parameter("require_contact_verification", True)
        self.maximum_goal_timeout = max(
            0.1, float(self.get_parameter("maximum_goal_timeout").value)
        )
        self.status_timeout = max(
            0.1, float(self.get_parameter("controller_status_timeout").value)
        )
        self.minimum_success_progress = float(
            max(
                0.0,
                min(1.0, self.get_parameter("minimum_success_progress").value),
            )
        )
        self.require_contact_verification = bool(
            self.get_parameter("require_contact_verification").value
        )

        self.command_pub = self.create_publisher(
            CrossingCommand, str(self.get_parameter("command_topic").value), 10
        )
        self.legacy_pub = self.create_publisher(
            String, str(self.get_parameter("legacy_request_topic").value), 10
        )
        callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            CrossingStatus,
            str(self.get_parameter("status_topic").value),
            self.status_callback,
            10,
            callback_group=callback_group,
        )
        self._lock = threading.Lock()
        self._goal_reserved = False
        self._active_goal_id = None
        self._active_goal_handle = None
        self._latest_status = None
        self._latest_status_time = None
        self._last_progress = 0.0
        self._action_server = ActionServer(
            self,
            TraverseObstacle,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group,
        )
        self.get_logger().info("Obstacle traversal Action gateway ready")

    def goal_callback(self, goal_request):
        """校验并原子预留控制器；并发 Goal 会在执行开始前被拒绝。"""
        valid, reason = validate_goal_values(
            int(goal_request.mode),
            float(goal_request.obstacle_height),
            float(goal_request.obstacle_distance),
            float(goal_request.speed_scale),
            float(goal_request.timeout),
            self.maximum_goal_timeout,
        )
        if not valid:
            self.get_logger().warning(f"Rejected crossing goal: {reason}")
            return GoalResponse.REJECT
        with self._lock:
            if self._goal_reserved:
                self.get_logger().warning("Rejected crossing goal: controller is busy")
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        """Cancellation is always accepted for the active safety-critical goal."""
        return CancelResponse.ACCEPT

    def status_callback(self, msg: CrossingStatus) -> None:
        """只接收当前 UUID 的合法状态；无关/畸形消息不能刷新安全心跳。"""
        goal_id = bytes(msg.goal_id.uuid)
        with self._lock:
            if goal_id != self._active_goal_id:
                return
            valid, reason = validate_controller_status(
                int(msg.state),
                int(msg.phase),
                float(msg.progress),
                self._last_progress,
            )
            if not valid:
                self.get_logger().warning(
                    f"Ignored invalid crossing status: {reason}",
                    throttle_duration_sec=1.0,
                )
                return
            self._latest_status = msg
            self._latest_status_time = time.monotonic()
            self._last_progress = float(msg.progress)

    def execute_callback(self, goal_handle):
        """转发 START，持续反馈，并在取消、超时或终态时返回强类型结果。"""
        start = time.monotonic()
        goal_id = bytes(goal_handle.goal_id.uuid)
        with self._lock:
            self._active_goal_id = goal_id
            self._active_goal_handle = goal_handle
            self._latest_status = None
            self._latest_status_time = None
            self._last_progress = 0.0
        try:
            self._publish_command(
                CrossingCommand.START, goal_handle.goal_id, goal_handle.request
            )
            self.legacy_pub.publish(
                String(
                    data=MODE_NAMES.get(int(goal_handle.request.mode), "UNKNOWN")
                )
            )
            while rclpy.ok():
                elapsed = time.monotonic() - start
                if goal_handle.is_cancel_requested:
                    self._publish_command(
                        CrossingCommand.CANCEL,
                        goal_handle.goal_id,
                        goal_handle.request,
                    )
                    goal_handle.canceled()
                    return self._result(
                        False,
                        TraverseObstacle.Result.CANCELED,
                        "goal canceled; cancel command forwarded",
                        elapsed,
                    )
                if elapsed > float(goal_handle.request.timeout):
                    self._publish_command(
                        CrossingCommand.CANCEL,
                        goal_handle.goal_id,
                        goal_handle.request,
                    )
                    goal_handle.abort()
                    return self._result(
                        False,
                        TraverseObstacle.Result.CONTROLLER_TIMEOUT,
                        "goal timeout; cancel command forwarded",
                        elapsed,
                    )

                with self._lock:
                    status = self._latest_status
                    status_time = self._latest_status_time
                # Goal 总超时约束整次动作；状态超时约束控制器通信。两者缺一不可：
                # 持续发 RUNNING 不能无限执行，完全不回状态也不能继续驱动机器人。
                if status_time is None:
                    stale = elapsed > self.status_timeout
                else:
                    stale = time.monotonic() - status_time > self.status_timeout
                if stale:
                    self._publish_command(
                        CrossingCommand.CANCEL,
                        goal_handle.goal_id,
                        goal_handle.request,
                    )
                    goal_handle.abort()
                    return self._result(
                        False,
                        TraverseObstacle.Result.CONTROLLER_TIMEOUT,
                        "gait controller status timed out",
                        elapsed,
                    )

                if status is not None:
                    goal_handle.publish_feedback(self._feedback(status, elapsed))
                    if status.state == CrossingStatus.SUCCEEDED:
                        success_valid = controller_success_is_valid(
                            float(status.progress),
                            bool(status.contact_verified),
                            self.minimum_success_progress,
                            self.require_contact_verification,
                        )
                        if not success_valid:
                            goal_handle.abort()
                            return self._result(
                                False,
                                TraverseObstacle.Result.EXECUTION_FAILED,
                                "controller success lacked progress/contact proof",
                                elapsed,
                            )
                        goal_handle.succeed()
                        return self._result(
                            True,
                            TraverseObstacle.Result.OK,
                            status.message or "obstacle traversal completed",
                            elapsed,
                        )
                    if status.state == CrossingStatus.FAILED:
                        goal_handle.abort()
                        return self._result(
                            False,
                            TraverseObstacle.Result.EXECUTION_FAILED,
                            status.message or "gait controller reported failure",
                            elapsed,
                        )
                    if status.state == CrossingStatus.CANCELED:
                        goal_handle.canceled()
                        return self._result(
                            False,
                            TraverseObstacle.Result.CANCELED,
                            status.message or "gait controller canceled execution",
                            elapsed,
                        )
                else:
                    feedback = TraverseObstacle.Feedback()
                    feedback.phase = TraverseObstacle.Feedback.ACCEPTED
                    feedback.progress = 0.0
                    feedback.elapsed = float(elapsed)
                    feedback.message = "waiting for gait controller"
                    goal_handle.publish_feedback(feedback)
                time.sleep(0.05)
        finally:
            with self._lock:
                if self._active_goal_id == goal_id:
                    self._active_goal_id = None
                    self._active_goal_handle = None
                    self._latest_status = None
                    self._latest_status_time = None
                    self._last_progress = 0.0
                    self._goal_reserved = False

    def _publish_command(self, command, goal_id, request) -> None:
        """完整复制 Goal 参数和 UUID，确保控制器响应可与请求一一对应。"""
        msg = CrossingCommand()
        msg.command = command
        msg.goal_id = goal_id
        msg.mode = request.mode
        msg.obstacle_height = request.obstacle_height
        msg.obstacle_distance = request.obstacle_distance
        msg.speed_scale = request.speed_scale
        msg.timeout = request.timeout
        self.command_pub.publish(msg)

    @staticmethod
    def _feedback(status: CrossingStatus, elapsed: float):
        """将已校验状态转换为 Action Feedback；夹紧仅作为最后一道防御。"""
        feedback = TraverseObstacle.Feedback()
        feedback.phase = status.phase
        feedback.progress = float(max(0.0, min(1.0, status.progress)))
        feedback.elapsed = float(elapsed)
        feedback.contact_verified = bool(status.contact_verified)
        feedback.message = status.message
        return feedback

    @staticmethod
    def _result(success: bool, code: int, message: str, duration: float):
        """集中构造 Action Result，保证所有退出路径字段一致。"""
        result = TraverseObstacle.Result()
        result.success = success
        result.error_code = code
        result.message = message
        result.duration = float(duration)
        return result

    def destroy_node(self):
        """节点退出前向仍活动的真机控制器发送 CANCEL。"""
        with self._lock:
            active = self._active_goal_handle
        if active is not None:
            self._publish_command(
                CrossingCommand.CANCEL, active.goal_id, active.request
            )
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CrossingActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
        except KeyboardInterrupt:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

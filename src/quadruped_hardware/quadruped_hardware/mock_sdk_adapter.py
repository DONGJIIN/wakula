"""无需真机的 CrossingCommand/CrossingStatus SDK 后端替身。

它不模拟动力学或关节运动，只严格实现未来厂商 SDK 必须遵守的 UUID、心跳、阶段、进度、
触地、取消和终态合同。可通过 ``/testing/mock_controller_fault`` 注入 fail、silence 或
invalid_progress，验证 Action 服务端和安全互锁是否按预期失败。
"""

import math
import time

import rclpy
from quadruped_interfaces.msg import CrossingCommand, CrossingStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


VALID_FAULTS = {"none", "fail", "silence", "invalid_progress"}


def mock_traversal_state(elapsed: float, duration: float, fault: str = "none"):
    """返回 ``(state, phase, progress, contact)``，silence 返回 ``None``。"""
    if not math.isfinite(elapsed) or not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("elapsed and duration must be finite; duration must be > 0")
    if fault == "silence":
        return None
    progress = max(0.0, min(1.0, elapsed / duration))
    if fault == "invalid_progress":
        progress = 1.2
    if fault == "fail" and elapsed >= duration * 0.5:
        return CrossingStatus.FAILED, CrossingStatus.RECOVERING, progress, False
    if elapsed >= duration:
        return (
            CrossingStatus.SUCCEEDED,
            CrossingStatus.VERIFYING_CONTACT,
            progress,
            True,
        )
    if progress < 0.10:
        phase = CrossingStatus.ACCEPTED
    elif progress < 0.25:
        phase = CrossingStatus.PREPARING
    elif progress < 0.90:
        phase = CrossingStatus.EXECUTING
    else:
        phase = CrossingStatus.VERIFYING_CONTACT
    return CrossingStatus.RUNNING, phase, progress, progress >= 0.90


class MockSdkAdapter(Node):
    """模拟一个单任务厂商 SDK，并以 20 Hz 发布强类型执行状态。"""

    def __init__(self):
        super().__init__("mock_sdk_adapter")
        self.declare_parameter("step_duration", 2.0)
        self.declare_parameter("climb_duration", 4.0)
        self.declare_parameter("low_profile_duration", 3.0)
        self.declare_parameter("initial_fault", "none")
        self.durations = {
            CrossingCommand.STEP: max(
                0.1, float(self.get_parameter("step_duration").value)
            ),
            CrossingCommand.CLIMB: max(
                0.1, float(self.get_parameter("climb_duration").value)
            ),
            CrossingCommand.LOW_PROFILE: max(
                0.1, float(self.get_parameter("low_profile_duration").value)
            ),
        }
        fault = str(self.get_parameter("initial_fault").value).strip().lower()
        self.fault = fault if fault in VALID_FAULTS else "none"
        self.active_command = None
        self.started_at = None
        self.status_pub = self.create_publisher(
            CrossingStatus, "/crossing/execution_status", 10
        )
        self.create_subscription(
            CrossingCommand,
            "/crossing/execution_command",
            self.command_callback,
            10,
        )
        self.create_subscription(
            String,
            "/testing/mock_controller_fault",
            self.fault_callback,
            10,
        )
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().warning(
            "Mock SDK adapter active: no physical leg command will be sent"
        )

    def fault_callback(self, msg: String) -> None:
        value = msg.data.strip().lower()
        if value not in VALID_FAULTS:
            self.get_logger().warning(f"Ignored unknown mock fault: {value}")
            return
        self.fault = value
        self.get_logger().warning(f"Mock controller fault -> {value}")

    def command_callback(self, msg: CrossingCommand) -> None:
        goal_id = bytes(msg.goal_id.uuid)
        if msg.command == CrossingCommand.CANCEL:
            if self.active_command is not None and goal_id == bytes(
                self.active_command.goal_id.uuid
            ):
                self._publish_terminal(CrossingStatus.CANCELED, "mock canceled")
            return
        if msg.command != CrossingCommand.START or msg.mode not in self.durations:
            return
        self.active_command = msg
        self.started_at = time.monotonic()

    def timer_callback(self) -> None:
        if self.active_command is None or self.started_at is None:
            return
        elapsed = time.monotonic() - self.started_at
        duration = self.durations[int(self.active_command.mode)]
        state = mock_traversal_state(elapsed, duration, self.fault)
        if state is None:
            return
        status_code, phase, progress, contact = state
        msg = CrossingStatus()
        msg.goal_id = self.active_command.goal_id
        msg.state = status_code
        msg.phase = phase
        msg.progress = float(progress)
        msg.contact_verified = bool(contact)
        msg.message = f"mock backend ({self.fault})"
        self.status_pub.publish(msg)
        if status_code != CrossingStatus.RUNNING:
            self.active_command = None
            self.started_at = None

    def _publish_terminal(self, state: int, message: str) -> None:
        msg = CrossingStatus()
        msg.goal_id = self.active_command.goal_id
        msg.state = state
        msg.phase = CrossingStatus.RECOVERING
        msg.progress = 0.0
        msg.contact_verified = False
        msg.message = message
        self.status_pub.publish(msg)
        self.active_command = None
        self.started_at = None


def main(args=None):
    rclpy.init(args=args)
    node = MockSdkAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

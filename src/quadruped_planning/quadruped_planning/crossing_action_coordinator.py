"""将稳定地形决策自动转换为可取消的越障 Action Goal。

感知/融合节点只负责判断“需要 STEP 或 CLIMB”，Action 服务端只负责管理一个已经提交
的 Goal。本节点补齐两者之间的编排：在障碍进入触发距离时发送 Goal、越障期间暂停
Nav2 速度、转发安全取消，并在重新看到连续平地前禁止对同一障碍重复触发。
"""

import math
import signal
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from quadruped_interfaces.action import TraverseObstacle
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String


TERRAIN_FRONTAL_HEIGHT = 6
TERRAIN_LOOKAHEAD = 7
TERRAIN_OBSTACLE_TYPE = 11
GEOMETRY_BAR = 5
MODE_TO_ACTION = {
    "STEP": TraverseObstacle.Goal.STEP,
    "CLIMB": TraverseObstacle.Goal.CLIMB,
}


def select_goal_mode(mode: str, obstacle_type: int, action_intent: str) -> int:
    """把四态 FSM 和比赛动作意图转换为底层三类 Action。"""
    intent = action_intent.upper()
    if obstacle_type == GEOMETRY_BAR or "LOW_PROFILE" in intent:
        return TraverseObstacle.Goal.LOW_PROFILE
    if mode == "CLIMB" or "CLIMB" in intent or "WALL" in intent:
        return TraverseObstacle.Goal.CLIMB
    return TraverseObstacle.Goal.STEP


@dataclass
class CrossingTriggerLatch:
    """对一个物理障碍最多触发有限次，并以连续 WALK 证据重新布防。"""

    clear_frames: int = 3
    retry_limit: int = 1
    armed: bool = True
    retries: int = 0
    clear_count: int = 0

    def observe(self, mode: str, within_trigger_distance: bool, active: bool) -> bool:
        """返回本帧是否应发送 Goal；本函数不依赖 ROS，便于单元测试。"""
        if mode == "WALK":
            self.clear_count += 1
            if self.clear_count >= max(1, self.clear_frames):
                self.armed = True
                self.retries = 0
            return False
        self.clear_count = 0
        if active or mode not in MODE_TO_ACTION or not within_trigger_distance:
            return False
        if not self.armed:
            return False
        self.armed = False
        return True

    def finish(self, success: bool) -> None:
        """失败时按上限允许重试；成功后必须离开障碍并重新看到平地。"""
        if not success and self.retries < max(0, self.retry_limit):
            self.retries += 1
            self.armed = True


def valid_terrain_observation(data, trigger_distance: float):
    """解析地形高度/距离；短消息、NaN、负值和 ROI 外距离均拒绝。"""
    if len(data) <= TERRAIN_LOOKAHEAD:
        return None
    height = float(data[TERRAIN_FRONTAL_HEIGHT])
    distance = float(data[TERRAIN_LOOKAHEAD])
    if (
        not math.isfinite(height)
        or not math.isfinite(distance)
        or height < 0.0
        or distance < 0.0
        or distance > trigger_distance
    ):
        return None
    return height, distance


class CrossingActionCoordinator(Node):
    """根据模式和最近地形特征发起 Action，并发布执行互锁状态。"""

    def __init__(self):
        super().__init__("crossing_action_coordinator")
        self.declare_parameter("action_name", "/crossing/traverse_obstacle")
        self.declare_parameter("trigger_distance", 0.80)
        self.declare_parameter("observation_timeout", 0.70)
        self.declare_parameter("safety_timeout", 0.50)
        self.declare_parameter("goal_timeout", 20.0)
        self.declare_parameter("step_speed_scale", 0.35)
        self.declare_parameter("climb_speed_scale", 0.20)
        self.declare_parameter("clear_confirmation_frames", 3)
        self.declare_parameter("retry_limit", 1)
        self.declare_parameter("require_navigation_health", True)

        self.trigger_distance = max(
            0.05, float(self.get_parameter("trigger_distance").value)
        )
        self.observation_timeout = max(
            0.05, float(self.get_parameter("observation_timeout").value)
        )
        self.safety_timeout = max(
            0.05, float(self.get_parameter("safety_timeout").value)
        )
        self.goal_timeout = min(
            60.0,
            max(0.1, float(self.get_parameter("goal_timeout").value)),
        )
        self.speed_scales = {
            "STEP": self._unit_parameter("step_speed_scale"),
            "CLIMB": self._unit_parameter("climb_speed_scale"),
        }
        self.latch = CrossingTriggerLatch(
            clear_frames=int(
                self.get_parameter("clear_confirmation_frames").value
            ),
            retry_limit=int(self.get_parameter("retry_limit").value),
        )

        self.mode = "STOP"
        self.action_intent = ""
        self.latest_terrain = None
        self.latest_terrain_time = None
        self.goal_active = False
        self.goal_pending = False
        self.goal_handle = None
        self.cancel_requested = False
        self.require_navigation_health = bool(
            self.get_parameter("require_navigation_health").value
        )
        self.navigation_healthy = not self.require_navigation_health
        # 没收到安全监督心跳前不允许发起真实动作。
        self.safety_stop = True
        self.last_safety_time = None

        active_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active_pub = self.create_publisher(
            Bool, "/crossing/execution_active", active_qos
        )
        self.result_pub = self.create_publisher(
            String, "/crossing/last_result", 10
        )
        self.create_subscription(String, "/crossing/mode", self.mode_callback, 10)
        self.create_subscription(
            String, "/crossing/action", self.action_callback, 10
        )
        self.create_subscription(
            Float32MultiArray,
            "/terrain/features",
            self.terrain_callback,
            10,
        )
        self.create_subscription(Bool, "/safety/stop", self.safety_callback, 10)
        self.create_subscription(
            Bool, "/navigation/healthy", self.navigation_health_callback, 10
        )
        self.action_client = ActionClient(
            self,
            TraverseObstacle,
            str(self.get_parameter("action_name").value),
        )
        self.active_pub.publish(Bool(data=False))
        self.create_timer(0.05, self.safety_watchdog_callback)
        self.get_logger().info("Automatic crossing Action coordinator ready")

    def _unit_parameter(self, name: str) -> float:
        """读取严格大于零且不超过一的速度缩放值。"""
        value = float(self.get_parameter(name).value)
        return min(1.0, max(0.01, value if math.isfinite(value) else 0.01))

    def mode_callback(self, msg: String) -> None:
        """更新稳定模式；STOP 或安全停车会取消当前越障。"""
        self.mode = msg.data.strip().upper()
        if self.mode == "STOP" and self.goal_active:
            self._cancel_active("terrain escalated to STOP")
        self._maybe_send_goal()

    def terrain_callback(self, msg: Float32MultiArray) -> None:
        """原子保存地形数组和接收时刻，随后尝试触发。"""
        self.latest_terrain = tuple(msg.data)
        self.latest_terrain_time = self.get_clock().now()
        self._maybe_send_goal()

    def action_callback(self, msg: String) -> None:
        """保存比赛/通用状态机动作意图；模式仍负责互锁和严重度。"""
        self.action_intent = msg.data.strip().upper()

    def safety_callback(self, msg: Bool) -> None:
        """安全停车拥有最高权限，并会主动取消底层动作。"""
        self.safety_stop = bool(msg.data)
        self.last_safety_time = self.get_clock().now()
        if self.safety_stop and self.goal_active:
            self._cancel_active("safety supervisor requested stop")
        elif not self.safety_stop:
            self._maybe_send_goal()

    def safety_watchdog_callback(self) -> None:
        """安全监督器本身断流也等价于停车，并取消正在执行的动作。"""
        age = (
            float("inf")
            if self.last_safety_time is None
            else (
                self.get_clock().now() - self.last_safety_time
            ).nanoseconds / 1e9
        )
        if age > self.safety_timeout:
            self.safety_stop = True
            if self.goal_active:
                self._cancel_active("safety supervisor heartbeat timed out")

    def navigation_health_callback(self, msg: Bool) -> None:
        """定位、TF 或传感器失效时取消动作并交回重规划流程。"""
        self.navigation_healthy = bool(msg.data)
        if not self.navigation_healthy and self.goal_active:
            self._cancel_active("navigation inputs became unhealthy")
        elif self.navigation_healthy:
            self._maybe_send_goal()

    def _maybe_send_goal(self) -> None:
        if (
            self.latest_terrain is None
            or self.latest_terrain_time is None
            or self.safety_stop
            or not self.navigation_healthy
        ):
            return
        age = (
            self.get_clock().now() - self.latest_terrain_time
        ).nanoseconds / 1e9
        observation = valid_terrain_observation(
            self.latest_terrain, self.trigger_distance
        )
        within_distance = observation is not None and age <= self.observation_timeout
        should_trigger = self.latch.observe(
            self.mode,
            within_distance,
            self.goal_active or self.goal_pending,
        )
        if not should_trigger or observation is None:
            return
        if not self.action_client.server_is_ready():
            # 未找到服务端不消耗一次触发机会，下帧仍可重试发现。
            self.latch.armed = True
            self.get_logger().warning(
                "Crossing Action server is not ready",
                throttle_duration_sec=2.0,
            )
            return

        height, distance = observation
        goal = TraverseObstacle.Goal()
        # 横杆沿用四态 FSM 的 CLIMB 占用/互锁流程，但给执行器发送 LOW_PROFILE，
        # 避免为了一个障碍类型再增加一套并行状态机。
        obstacle_type = (
            int(round(self.latest_terrain[TERRAIN_OBSTACLE_TYPE]))
            if len(self.latest_terrain) > TERRAIN_OBSTACLE_TYPE
            else 0
        )
        goal.mode = select_goal_mode(
            self.mode, obstacle_type, self.action_intent
        )
        goal.obstacle_height = height
        goal.obstacle_distance = distance
        goal.speed_scale = self.speed_scales[self.mode]
        goal.timeout = self.goal_timeout
        self.goal_pending = True
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        self.goal_pending = False
        try:
            self.goal_handle = future.result()
        except Exception as exc:  # rclpy future transports executor exceptions.
            self._finish(False, f"goal submission failed: {exc}")
            return
        if not self.goal_handle.accepted:
            self._finish(False, "crossing goal rejected")
            return
        self.goal_active = True
        self.cancel_requested = False
        self.active_pub.publish(Bool(data=True))
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
        if self.safety_stop or self.mode == "STOP":
            self._cancel_active("stop became active while goal was pending")

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Crossing progress={feedback.progress:.2f} phase={feedback.phase}"
        )

    def _result_callback(self, future) -> None:
        try:
            wrapped = future.result()
            success = (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and bool(wrapped.result.success)
            )
            message = wrapped.result.message or f"status={wrapped.status}"
        except Exception as exc:
            success, message = False, f"result transport failed: {exc}"
        self._finish(success, message)

    def _finish(self, success: bool, message: str) -> None:
        self.goal_active = False
        self.goal_pending = False
        self.goal_handle = None
        self.cancel_requested = False
        self.active_pub.publish(Bool(data=False))
        self.result_pub.publish(
            String(data=f"{'SUCCESS' if success else 'FAILED'}: {message}")
        )
        self.latch.finish(success)

    def _cancel_active(self, reason: str) -> None:
        if self.goal_handle is not None and not self.cancel_requested:
            self.cancel_requested = True
            self.get_logger().warning(f"Cancel crossing: {reason}")
            self.goal_handle.cancel_goal_async()


def main(args=None):
    """运行自动越障 Action 协调节点。"""
    rclpy.init(args=args)
    node = CrossingActionCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # ROS 2 Jazzy may tear down a subscription between wait-set wakeup and
        # message conversion when launch broadcasts SIGINT.  Suppress only
        # that shutdown race; a RuntimeError during normal operation remains
        # visible and must not be mistaken for a clean exit.
        if rclpy.ok():
            raise
    finally:
        # launch 与 timeout 可能连续转发 SIGINT，清理阶段也必须可重入退出。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

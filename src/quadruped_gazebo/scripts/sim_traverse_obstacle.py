#!/usr/bin/env python3
"""Gazebo 通用测试狗的 TraverseObstacle Action 适配器。

此节点只属于仿真包。通用狗没有腿部动力学，因此它用受限的短时平面前进模拟“底层控制器
完成了跨越”，用于验证任务编排、取消和逐障碍继续探索。真机绝不能启动它；运动团队应以
同名 Action server 替换，核心导航节点无需修改。
"""

import time

from geometry_msgs.msg import Twist
from quadruped_interfaces.action import TraverseObstacle
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


class SimTraverseObstacle(Node):
    def __init__(self):
        super().__init__("sim_traverse_obstacle")
        self.declare_parameter("command_topic", "/cmd_vel_teleop")
        self.declare_parameter("forward_speed", 0.20)
        # 通用载体没有接触/落脚状态，只能按规则障碍长度给出确定性的流程模拟时间。
        # 坑洞最长，限高杆只需通过其投影区；这些值绝不能复制到真机控制器。
        self.declare_parameter("step_duration", 4.0)
        self.declare_parameter("pit_duration", 11.5)
        self.declare_parameter("wall_duration", 4.0)
        self.declare_parameter("bar_duration", 5.0)
        self.declare_parameter("slope_duration", 9.0)
        self.declare_parameter("stabilize_duration", 0.6)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        self.busy = False
        self.server = ActionServer(
            self,
            TraverseObstacle,
            "/traverse_obstacle",
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().warning(
            "SIMULATION ONLY TraverseObstacle adapter active; no leg dynamics"
        )

    def goal_callback(self, goal):
        valid = (
            not self.busy
            # CLEAR=1 和 POLE=6 不应交接；坡面由任务层显式映射为 SLOPE=7。
            and goal.obstacle_type in (2, 3, 4, 5, 7)
            and 0.0 <= goal.confidence <= 1.0
        )
        return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

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

    def execute(self, handle):
        self.busy = True
        result = TraverseObstacle.Result()
        try:
            duration_parameter = {
                2: "step_duration",
                3: "pit_duration",
                4: "wall_duration",
                5: "bar_duration",
                7: "slope_duration",
            }[int(handle.request.obstacle_type)]
            duration = max(0.5, float(self.get_parameter(duration_parameter).value))
            speed = max(0.03, min(0.25, float(self.get_parameter("forward_speed").value)))
            started = time.monotonic()
            while rclpy.ok() and time.monotonic() - started < duration:
                if handle.is_cancel_requested:
                    self._stop()
                    handle.canceled()
                    result.success = False
                    result.message = "simulation traversal cancelled"
                    return result
                elapsed = time.monotonic() - started
                command = Twist()
                command.linear.x = speed
                self.publisher.publish(command)
                feedback = TraverseObstacle.Feedback()
                feedback.state = TraverseObstacle.Feedback.STATE_TRAVERSING
                feedback.progress = min(0.9, elapsed / duration * 0.9)
                feedback.message = "simulation planar traversal"
                handle.publish_feedback(feedback)
                time.sleep(0.05)
            self._stop()
            if not rclpy.ok():
                result.success = False
                result.message = "simulation shutting down"
                return result
            stabilize = max(0.0, float(self.get_parameter("stabilize_duration").value))
            deadline = time.monotonic() + stabilize
            while rclpy.ok() and time.monotonic() < deadline:
                self._stop()
                time.sleep(0.05)
            handle.succeed()
            result.success = True
            result.message = "simulation traversal completed"
            return result
        finally:
            self._stop()
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = SimTraverseObstacle()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

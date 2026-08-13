"""终端键盘式自主导航开关。

本工具只调用 ``/autonomy/toggle``，不发布速度，也不读取 Gazebo。它可以与
``slam.launch.py`` 同时运行在仿真或真机上：空格（或 t）切换自主任务，q 退出工具。
退出工具不会关闭 SLAM；为避免误以为退出等于停车，退出前会明确打印当前行为。
"""

from __future__ import annotations

import select
import sys
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


class AutonomyKeyboard(Node):
    """把单个终端按键转换为算法层的原子 toggle 服务调用。"""

    def __init__(self):
        super().__init__("autonomy_keyboard")
        self.client = self.create_client(Trigger, "/autonomy/toggle")
        self.latest_state = "UNKNOWN"
        self.create_subscription(String, "/autonomy/state", self._state_callback, 10)

    def _state_callback(self, msg: String) -> None:
        """缓存任务状态，让按键结果能够反映真实任务执行而不只是服务响应。"""
        self.latest_state = msg.data

    def toggle(self) -> None:
        """等待服务并切换；只有服务端成功响应才向操作员报告成功。"""
        if not self.client.wait_for_service(timeout_sec=2.0):
            print("未找到自主导航节点，请先启动：ros2 launch slam slam.launch.py")
            return
        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            print("切换超时，自主导航状态未确认。")
        elif result.success:
            print(f"切换成功：{result.message}")
            # Trigger 成功只说明 autonomous_mission 接受了开关请求。继续短暂监听状态，
            # 区分真正进入 EXPLORING/ALIGNING_OBSTACLE 与仍缺少地图、TF 或 Nav2 输入。
            deadline = time.monotonic() + 3.0
            initial_state = self.latest_state
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.latest_state in {
                    "EXPLORING",
                    "APPROACHING_OBSTACLE",
                    "ALIGNING_OBSTACLE",
                    "TRAVERSING",
                    "STOPPED",
                    "COMPLETED",
                }:
                    break
                # 关闭请求通常立即变为 STOPPED；开启请求允许 WAITING_FOR_INPUTS 继续
                # 等待短暂的地图/导航发现，但不能因此把“接受请求”误报成“已经在走”。
                if self.latest_state != initial_state:
                    initial_state = self.latest_state
            if self.latest_state == "WAITING_FOR_INPUTS":
                print(
                    "当前状态：WAITING_FOR_INPUTS（尚未开始移动；请检查 /map、"
                    "/navigation/healthy、/scan 和 /odom）"
                )
            else:
                print(f"当前状态：{self.latest_state}")
        else:
            print(f"切换被拒绝：{result.message}")


def main(args=None):
    """进入原始终端模式读取单键，并在 finally 中恢复终端设置。"""
    rclpy.init(args=args)
    node = AutonomyKeyboard()
    if not sys.stdin.isatty():
        node.get_logger().error("键盘工具必须在可交互终端中运行")
        node.destroy_node()
        rclpy.shutdown()
        return

    old_settings = termios.tcgetattr(sys.stdin)
    print("自主导航键盘开关：空格/t = 开启或停止，q = 退出工具")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            # 短超时允许 Ctrl-C/RCL shutdown 被及时处理，不让 read() 永久阻塞。
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                rclpy.spin_once(node, timeout_sec=0.0)
                continue
            key = sys.stdin.read(1)
            if key in (" ", "t", "T"):
                node.toggle()
            elif key in ("q", "Q"):
                print("退出键盘工具；自主导航保持当前状态。需要停车请先按空格。")
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

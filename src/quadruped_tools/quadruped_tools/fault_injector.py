"""向硬件无关闭环注入可重复的软件故障。

示例：``ros2 run quadruped_tools fault_injector --estop on``，或使用
``--controller-fault silence`` 验证 Action 状态心跳超时。该工具只能用于台架、回放和
仿真；真机测试必须先架空并确保物理急停可用。
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


FAULT_CHOICES = ("none", "fail", "silence", "invalid_progress")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inject Wakula safety faults.")
    parser.add_argument("--estop", choices=("on", "off"))
    parser.add_argument("--controller-fault", choices=FAULT_CHOICES)
    parser.add_argument("--service-timeout", type=float, default=2.0)
    return parser


class FaultInjector(Node):
    """一次性调用急停服务并向 mock SDK 发布故障模式。"""

    def __init__(self):
        super().__init__("fault_injector")
        self.estop_client = self.create_client(
            SetBool, "/safety/set_emergency_stop"
        )
        self.fault_pub = self.create_publisher(
            String, "/testing/mock_controller_fault", 10
        )

    def set_estop(self, enabled: bool, timeout: float) -> None:
        if not self.estop_client.wait_for_service(timeout_sec=max(0.1, timeout)):
            raise RuntimeError("safety emergency-stop service is unavailable")
        future = self.estop_client.call_async(SetBool.Request(data=enabled))
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(0.1, timeout))
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError("failed to update software emergency stop")

    def set_controller_fault(self, fault: str) -> None:
        # 连续发布三次可降低命令早于 mock 订阅发现时被丢弃的概率。
        for _ in range(3):
            self.fault_pub.publish(String(data=fault))
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)


def main(argv=None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.estop is None and args.controller_fault is None:
        raise SystemExit("provide --estop or --controller-fault")
    rclpy.init()
    node = FaultInjector()
    try:
        if args.estop is not None:
            node.set_estop(args.estop == "on", args.service_timeout)
        if args.controller_fault is not None:
            node.set_controller_fault(args.controller_fault)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

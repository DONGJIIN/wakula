"""只发布标准反馈的模拟硬件节点，不实现动力学或基础控制。"""

import math
import signal

import rclpy
from quadruped_interfaces.msg import HardwareStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, JointState

from quadruped_hardware.sdk_adapter import JOINT_NAMES


class MockHardwareState(Node):
    """为上层安全、诊断和接口测试提供确定性的静止硬件心跳。"""

    def __init__(self):
        super().__init__("mock_hardware_state")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("battery_voltage", 24.0)
        self.declare_parameter("hardware_fault_code", 0)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.status_pub = self.create_publisher(HardwareStatus, "/hardware/status", 10)
        rate = min(200.0, max(1.0, float(self.get_parameter("publish_rate").value)))
        self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().warning("Mock hardware state active; no actuator output exists")

    def publish_state(self) -> None:
        stamp = self.get_clock().now().to_msg()
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = list(JOINT_NAMES)
        joint.position = [0.0] * 12
        joint.velocity = [0.0] * 12
        joint.effort = [math.nan] * 12
        self.joint_pub.publish(joint)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = float(self.get_parameter("battery_voltage").value)
        battery.percentage = float("nan")
        self.battery_pub.publish(battery)

        fault = max(0, int(self.get_parameter("hardware_fault_code").value))
        status = HardwareStatus()
        status.header.stamp = stamp
        status.state = HardwareStatus.FAULT if fault else HardwareStatus.STANDBY
        status.command_ready = fault == 0
        status.actuators_enabled = False
        status.fault_code = fault
        status.fault_text = "injected mock fault" if fault else "mock standby"
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = MockHardwareState()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

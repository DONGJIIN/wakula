"""未来厂商 SDK 或 ros2_control hardware plugin 的统一边界。

真机代码只应在这一层处理 CAN/串口、厂商枚举、关节方向和单位换算。上层继续使用
``/joint_states``、``/imu/data``、``/battery_state``、``/cmd_vel`` 与强类型越障合同。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import math
from typing import Sequence


JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for leg in ("front_left", "front_right", "rear_left", "rear_right")
    for joint in ("hip", "thigh", "calf")
)


@dataclass(frozen=True)
class JointSample:
    """SDK 读回的一次原子关节采样，必须共享同一硬件时间戳。"""

    stamp_seconds: float
    position: Sequence[float]
    velocity: Sequence[float]
    effort: Sequence[float]

    def validate(self) -> None:
        fields = (self.position, self.velocity, self.effort)
        if not math.isfinite(self.stamp_seconds):
            raise ValueError("joint sample timestamp is not finite")
        if any(len(field) != len(JOINT_NAMES) for field in fields):
            raise ValueError("joint sample must contain exactly 12 joints")
        if not all(math.isfinite(float(value)) for field in fields for value in field):
            raise ValueError("joint sample contains a non-finite value")


class VendorSdkAdapterBase(ABC):
    """不绑定 ROS 执行器的最小 SDK 抽象；实现类不得自行改变标准话题名。"""

    @abstractmethod
    def connect(self) -> None:
        """连接驱动并读取能力；失败时抛出异常，不能伪装为 STANDBY。"""

    @abstractmethod
    def disable_actuators(self) -> None:
        """进入硬件定义的无输出/安全阻尼状态。"""

    @abstractmethod
    def read_joint_sample(self) -> JointSample:
        """读取带硬件时间戳的完整 12 关节反馈。"""

    @abstractmethod
    def fault_code(self) -> int:
        """返回 0 或厂商原始故障码；映射表应写入 connect.txt。"""

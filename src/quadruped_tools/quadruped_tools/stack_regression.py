"""Gazebo—SLAM—Nav2 长时间联合回归与资源采样工具。

该工具只用于测试，不属于机器人运行时控制链。它通过仿真专用高优先级
``/cmd_vel_teleop`` 执行“整圈旋转—前进—倒退回原位—反向整圈旋转”，持续监视
``/map``、``/scan``、``/odom``、导航健康和正前方名称；随后可调用 Nav2 的
NavigateThroughPoses 与 NavigateToPose Action，验证多目标、障碍内不可达目标及恢复取消。

真机默认禁止运行运动阶段。只有显式 ``--allow-motion`` 才会发布速度，且命令话题默认是
Gazebo 专用接口；这避免诊断脚本被误当成真实机器狗运动控制器。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class StreamStats:
    """一个 ROS 数据流的到达次数和最大墙钟间隔。"""

    count: int = 0
    max_gap_seconds: float = 0.0
    _last_time: Optional[float] = field(default=None, repr=False)

    def update(self) -> None:
        now = time.monotonic()
        if self._last_time is not None:
            self.max_gap_seconds = max(self.max_gap_seconds, now - self._last_time)
        self._last_time = now
        self.count += 1

    def public(self) -> dict:
        return {
            "count": self.count,
            "max_gap_seconds": round(self.max_gap_seconds, 4),
        }


@dataclass
class ResourceStats:
    """核心进程合计的近似 CPU 与常驻内存峰值。"""

    peak_rss_mib: float = 0.0
    peak_cpu_percent: float = 0.0
    samples: int = 0
    _previous_ticks: Optional[int] = field(default=None, repr=False)
    _previous_time: Optional[float] = field(default=None, repr=False)

    def sample(self) -> None:
        fragments = (
            "async_slam_toolbox_node",
            "controller_server",
            "planner_server",
            "bt_navigator",
            "terrain_analyzer",
            "vision_obstacle_detector",
            "perception_fusion",
            "terrain_safety_assessor",
            "navigation_speed_gate",
            "collision_monitor",
        )
        total_ticks = 0
        total_rss_pages = 0
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                    errors="ignore"
                )
                if not any(fragment in command for fragment in fragments):
                    continue
                fields = (entry / "stat").read_text().split()
                total_ticks += int(fields[13]) + int(fields[14])
                total_rss_pages += max(0, int(fields[23]))
            except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
                continue
        now = time.monotonic()
        rss_mib = total_rss_pages * os.sysconf("SC_PAGE_SIZE") / (1024.0 * 1024.0)
        self.peak_rss_mib = max(self.peak_rss_mib, rss_mib)
        if self._previous_ticks is not None and self._previous_time is not None:
            elapsed = now - self._previous_time
            if elapsed > 0.0:
                tick_rate = float(os.sysconf("SC_CLK_TCK"))
                cpu = (total_ticks - self._previous_ticks) / tick_rate / elapsed * 100.0
                self.peak_cpu_percent = max(self.peak_cpu_percent, max(0.0, cpu))
        self._previous_ticks = total_ticks
        self._previous_time = now
        self.samples += 1

    def public(self) -> dict:
        return {
            "peak_rss_mib": round(self.peak_rss_mib, 2),
            # 100% 表示约占满一个 CPU 核，不是整机百分比。
            "peak_cpu_percent_one_core": round(self.peak_cpu_percent, 1),
            "samples": self.samples,
        }


def _yaw_from_quaternion(quaternion) -> float:
    """只提取平面偏航角；仿真测试载体不使用滚转和俯仰。"""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _angle_error(first: float, second: float) -> float:
    """返回两个偏航角的最小绝对差。"""
    return abs(math.atan2(math.sin(second - first), math.cos(second - first)))


class StackRegression(Node):
    """采集全栈健康指标，并在得到明确授权时执行仿真回归轨迹。"""

    def __init__(self, command_topic: str):
        super().__init__("stack_regression")
        self.command_pub = self.create_publisher(Twist, command_topic, 10)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.streams: Dict[str, StreamStats] = {
            name: StreamStats() for name in ("map", "scan", "odom", "front_name")
        }
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, map_qos)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, "/navigation/healthy", self._health_callback, 10)
        self.create_subscription(String, "/perception/front_obstacle_name", self._name_callback, 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.through_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.latest_known_cells = 0
        self.maximum_known_cells = 0
        self.health_true = 0
        self.health_false = 0
        self.obstacle_names = set()
        self.resource = ResourceStats()
        self.loop_errors: List[dict] = []
        self._last_resource_sample = 0.0

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.streams["map"].update()
        # -1 是未知；不复制整个数组，只在低频地图回调统计一次已观测面积。
        self.latest_known_cells = sum(value >= 0 for value in msg.data)
        self.maximum_known_cells = max(self.maximum_known_cells, self.latest_known_cells)

    def _scan_callback(self, _msg: LaserScan) -> None:
        self.streams["scan"].update()

    def _odom_callback(self, _msg: Odometry) -> None:
        self.streams["odom"].update()

    def _health_callback(self, msg: Bool) -> None:
        if msg.data:
            self.health_true += 1
        else:
            self.health_false += 1

    def _name_callback(self, msg: String) -> None:
        self.streams["front_name"].update()
        if msg.data:
            self.obstacle_names.add(msg.data)

    def spin_for(self, seconds: float, command: Optional[Twist] = None) -> None:
        """在墙钟期限内处理回调，并以 20 Hz 重发速度防止仿真 mux 超时。"""
        deadline = time.monotonic() + max(0.0, seconds)
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if command is not None and now >= next_publish:
                self.command_pub.publish(command)
                next_publish = now + 0.05
            if now - self._last_resource_sample >= 1.0:
                self.resource.sample()
                self._last_resource_sample = now
            rclpy.spin_once(self, timeout_sec=0.03)

    def stop(self) -> None:
        """连续发送几帧零速度，保证 mux 在阶段切换处没有残余命令。"""
        self.spin_for(0.25, Twist())

    def pose(self) -> Optional[Tuple[float, float, float]]:
        """读取最新 map→base_link，用于回到同一点后的闭环一致性检查。"""
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            _yaw_from_quaternion(transform.transform.rotation),
        )

    def wait_ready(self, timeout: float) -> bool:
        """等待地图、雷达、里程计、定位 TF 和健康状态全部出现。"""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin_for(0.1)
            if (
                all(self.streams[name].count for name in ("map", "scan", "odom"))
                and self.pose() is not None
                and self.health_true > 0
            ):
                return True
        return False

    def run_mapping_cycles(self, cycles: int, linear_speed: float, angular_speed: float) -> None:
        """执行可回到原点的旋转/正退轨迹，并记录每轮 map 位姿闭环误差。"""
        rotation_duration = 2.0 * math.pi / max(0.05, abs(angular_speed))
        travel_duration = 5.0
        for index in range(cycles):
            start = self.pose()
            command = Twist()
            command.angular.z = abs(angular_speed)
            self.spin_for(rotation_duration, command)
            command = Twist()
            command.linear.x = abs(linear_speed)
            self.spin_for(travel_duration, command)
            command.linear.x = -abs(linear_speed)
            self.spin_for(travel_duration, command)
            command = Twist()
            command.angular.z = -abs(angular_speed)
            self.spin_for(rotation_duration, command)
            self.stop()
            self.spin_for(1.0)
            end = self.pose()
            if start is not None and end is not None:
                self.loop_errors.append(
                    {
                        "cycle": index + 1,
                        "position_error_m": round(math.hypot(end[0] - start[0], end[1] - start[1]), 4),
                        "yaw_error_rad": round(_angle_error(start[2], end[2]), 4),
                    }
                )

    def _relative_pose(self, origin, forward: float, left: float, yaw_delta: float) -> PoseStamped:
        """把相对初始机身的测试点转换成 map 坐标，避免依赖固定 SLAM 原点。"""
        x, y, yaw = origin
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x + math.cos(yaw) * forward - math.sin(yaw) * left
        pose.pose.position.y = y + math.sin(yaw) * forward + math.cos(yaw) * left
        target_yaw = yaw + yaw_delta
        pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        pose.pose.orientation.w = math.cos(target_yaw / 2.0)
        return pose

    def _wait_future(self, future, timeout: float):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self.spin_for(0.05)
        return future.result() if future.done() else None

    def run_nav2_scenarios(self, timeout: float) -> List[dict]:
        """验证多目标闭环以及指向高墙内部的不可达目标和恢复/取消路径。"""
        origin = self.pose()
        results: List[dict] = []
        if origin is None:
            return [{"scenario": "all", "result": "no_map_pose"}]
        if not self.through_client.wait_for_server(timeout_sec=10.0):
            return [{"scenario": "multi_goal", "result": "action_unavailable"}]

        # 三点折线路径同时覆盖前进、转弯和返回；相对距离保持在初始 360° 雷达已观测区。
        goal = NavigateThroughPoses.Goal()
        goal.poses = [
            self._relative_pose(origin, 0.70, 0.00, 0.0),
            self._relative_pose(origin, 0.70, 0.60, math.pi / 2.0),
            self._relative_pose(origin, 0.05, 0.05, math.pi),
        ]
        handle = self._wait_future(self.through_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            results.append({"scenario": "multi_goal", "result": "rejected"})
        else:
            result = self._wait_future(handle.get_result_async(), timeout)
            if result is None:
                cancel = self._wait_future(handle.cancel_goal_async(), 5.0)
                # 等旧 Action 真正结束后再提交下一目标，否则 bt_navigator 会以
                # “another navigator is processing”拒绝后续恢复场景，形成伪失败。
                self._wait_future(handle.get_result_async(), 5.0)
                results.append(
                    {
                        "scenario": "multi_goal",
                        "result": "timeout_cancelled",
                        "cancel_acknowledged": cancel is not None,
                    }
                )
            else:
                results.append(
                    {
                        "scenario": "multi_goal",
                        "result": "succeeded" if result.status == GoalStatus.STATUS_SUCCEEDED else f"status_{result.status}",
                    }
                )

        # 比赛参考布局中，出生点西南方的直角绕杆区柱间距为 1.0 m。下面的折线路径
        # 从两根柱之间穿过、绕过直角拐点后返回，覆盖“窄通道 + 连续转角 + 返航”。
        # 坐标相对本轮初始位姿表达，因此 SLAM 的 map 原点发生平移也不影响测试。
        narrow = NavigateThroughPoses.Goal()
        narrow.poses = [
            self._relative_pose(origin, -1.50, -0.80, -2.65),
            self._relative_pose(origin, -2.20, -1.35, math.pi),
            self._relative_pose(origin, -2.20, -0.55, math.pi / 2.0),
            self._relative_pose(origin, 0.05, 0.05, 0.0),
        ]
        handle = self._wait_future(self.through_client.send_goal_async(narrow), 10.0)
        if handle is None or not handle.accepted:
            results.append({"scenario": "narrow_pole_passage", "result": "rejected"})
        else:
            result = self._wait_future(handle.get_result_async(), timeout)
            if result is None:
                cancel = self._wait_future(handle.cancel_goal_async(), 5.0)
                self._wait_future(handle.get_result_async(), 5.0)
                results.append(
                    {
                        "scenario": "narrow_pole_passage",
                        "result": "timeout_cancelled",
                        "cancel_acknowledged": cancel is not None,
                    }
                )
            else:
                results.append(
                    {
                        "scenario": "narrow_pole_passage",
                        "result": (
                            "succeeded"
                            if result.status == GoalStatus.STATUS_SUCCEEDED
                            else f"status_{result.status}"
                        ),
                    }
                )

        if not self.pose_client.wait_for_server(timeout_sec=10.0):
            results.append({"scenario": "blocked_high_wall", "result": "action_unavailable"})
            return results
        # 参考 world 中高墙相对出生点约为前 1.95 m、右 1.15 m。目标位于占用体内部，
        # 正确结果是规划失败/恢复后失败，而不是穿墙成功。正式坐标变化时可只改测试参数。
        blocked = NavigateToPose.Goal()
        blocked.pose = self._relative_pose(origin, 1.95, -1.15, 0.0)
        handle = self._wait_future(self.pose_client.send_goal_async(blocked), 10.0)
        if handle is None or not handle.accepted:
            results.append({"scenario": "blocked_high_wall", "result": "rejected_as_expected"})
        else:
            result = self._wait_future(handle.get_result_async(), min(timeout, 45.0))
            if result is None:
                cancel = self._wait_future(handle.cancel_goal_async(), 5.0)
                results.append(
                    {
                        "scenario": "blocked_high_wall",
                        "result": "recovery_timeout_cancelled",
                        "cancel_acknowledged": cancel is not None,
                    }
                )
            else:
                results.append(
                    {
                        "scenario": "blocked_high_wall",
                        "result": "unsafe_success" if result.status == GoalStatus.STATUS_SUCCEEDED else "failed_as_expected",
                        "status": int(result.status),
                    }
                )
        self.stop()
        return results

    def report(self, nav2_results: Sequence[dict]) -> dict:
        """形成可存档报告；pass 只代表本次仿真软件回归，不代表真机验收。"""
        max_position_error = max(
            (item["position_error_m"] for item in self.loop_errors), default=float("inf")
        )
        max_yaw_error = max(
            (item["yaw_error_rad"] for item in self.loop_errors), default=float("inf")
        )
        streams_ok = (
            self.streams["map"].count > 0
            and self.streams["scan"].count > 0
            and self.streams["odom"].count > 0
            and self.streams["map"].max_gap_seconds < 2.0
            and self.streams["scan"].max_gap_seconds < 1.0
            and self.streams["odom"].max_gap_seconds < 1.0
        )
        mapping_ok = bool(self.loop_errors) and max_position_error < 0.35 and max_yaw_error < 0.50
        nav2_ok = all(
            item.get("result") in {
                "succeeded",
                "failed_as_expected",
                "rejected_as_expected",
                "recovery_timeout_cancelled",
            }
            for item in nav2_results
        ) if nav2_results else True
        return {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "passed": bool(streams_ok and mapping_ok and nav2_ok and self.health_true > 0),
            "streams": {name: stats.public() for name, stats in self.streams.items()},
            "navigation_health": {"true_samples": self.health_true, "false_samples": self.health_false},
            "map": {
                "latest_known_cells": self.latest_known_cells,
                "maximum_known_cells": self.maximum_known_cells,
            },
            "loop_closure": {
                "cycles": self.loop_errors,
                "max_position_error_m": None if not self.loop_errors else max_position_error,
                "max_yaw_error_rad": None if not self.loop_errors else max_yaw_error,
            },
            "front_obstacle_names_seen": sorted(self.obstacle_names),
            "nav2": list(nav2_results),
            "resources": self.resource.public(),
            "scope": "Gazebo software regression only; not a real-quadruped safety certification",
        }


def parse_args(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-motion", action="store_true", help="明确允许发布仿真测试速度")
    parser.add_argument("--cycles", type=int, default=5, help="旋转/正退闭环轮数，默认约 4 分钟")
    parser.add_argument("--linear-speed", type=float, default=0.16)
    parser.add_argument("--angular-speed", type=float, default=0.45)
    parser.add_argument("--command-topic", default="/cmd_vel_teleop")
    parser.add_argument("--ready-timeout", type=float, default=45.0)
    parser.add_argument("--nav2-timeout", type=float, default=90.0)
    parser.add_argument("--skip-nav2", action="store_true")
    parser.add_argument("--report", default="reports/stack_regression.json")
    return parser.parse_args(argv)


def main(args=None):
    options = parse_args(args)
    if not options.allow_motion:
        raise SystemExit("Refusing to publish motion without --allow-motion")
    rclpy.init()
    node = StackRegression(options.command_topic)
    try:
        if not node.wait_ready(max(1.0, options.ready_timeout)):
            raise RuntimeError("stack did not provide map/scan/odom/TF/healthy before timeout")
        node.run_mapping_cycles(
            max(1, options.cycles),
            min(0.25, max(0.05, abs(options.linear_speed))),
            min(0.65, max(0.10, abs(options.angular_speed))),
        )
        nav2_results = [] if options.skip_nav2 else node.run_nav2_scenarios(max(10.0, options.nav2_timeout))
        report = node.report(nav2_results)
        destination = Path(options.report).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if not report["passed"]:
            raise SystemExit(1)
    except KeyboardInterrupt:
        # 人工中止长测时仍先走 finally 的零速度与 ROS 清理，不打印误导性的 traceback。
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

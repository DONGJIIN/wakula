#!/usr/bin/env python3
"""Gazebo 通用测试狗的 TraverseObstacle Action 适配器。

此节点只属于仿真包。通用狗没有腿部动力学，普通平面速度会被高墙/坑沿的碰撞体挡住；
因此它通过 Gazebo 标准 SetEntityPose 服务沿已对正方向平滑跨过障碍，用来验证任务编排、
取消和逐障碍继续探索。真机绝不能启动它；运动团队应以同名 Action server 替换，核心
导航节点无需修改。
"""

from math import atan2, cos, pi, sin
import signal
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from quadruped_interfaces.action import TraverseObstacle
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


def yaw_from_odometry(msg: Odometry) -> float:
    """提取平面航向；仿真替身不依赖 TF，避免与 SLAM 的 map 修正形成闭环。"""
    q = msg.pose.pose.orientation
    return atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def traversal_pose(
    start_x,
    start_y,
    start_yaw,
    distance,
    progress,
    pole=False,
    l_turn=0,
):
    """返回仿真流程在 world/odom 平面中的目标 ``(x, y, yaw)``。

    普通结构沿对正方向直线跨越；直角绕杆仅用一条平滑 S 曲线表达流程。该函数完全不含
    world 坐标和障碍名称，真实位置仍由实时里程计决定，方便替换场地布局。
    """
    progress = max(0.0, min(1.0, float(progress)))
    total_distance = max(0.0, float(distance))
    forward = total_distance * progress
    lateral = 0.0
    heading_delta = 0.0
    if int(l_turn):
        # 规则砂砾/碎木坑为 L 形：先沿第一臂进入，再在拐角转入第二臂。用两段局部
        # 直线近似流程，避免旧版沿入口法向直穿后落到赛台外。l_turn=+1 左转、-1 右转；
        # 仿真后端会预先选择留在场内的一侧。真机控制器应以足端/视觉闭环替换本轨迹。
        first_leg = total_distance * 0.60
        travelled = total_distance * progress
        forward = min(first_leg, travelled)
        second_leg = max(0.0, travelled - first_leg)
        lateral = float(int(l_turn)) * second_leg
        if second_leg > 0.0:
            # 拐角处逐步转向，最终机头与第二臂切向一致，不横着结束 Action。
            turn_progress = min(
                1.0,
                second_leg / max(0.01, total_distance - first_leg),
            )
            heading_delta = float(int(l_turn)) * (pi / 2.0) * turn_progress
    elif pole:
        # 起终点横偏均为零，中段左右绕行；导数决定机身切向方向，避免模型横着跳。
        lateral = 0.32 * sin(2.0 * pi * progress) * sin(pi * progress)
        lateral_derivative = 0.32 * (
            2.0 * pi * cos(2.0 * pi * progress) * sin(pi * progress)
            + pi * sin(2.0 * pi * progress) * cos(pi * progress)
        )
        heading_delta = atan2(
            lateral_derivative,
            max(0.01, float(distance)),
        )
    x = start_x + cos(start_yaw) * forward - sin(start_yaw) * lateral
    y = start_y + sin(start_yaw) * forward + cos(start_yaw) * lateral
    return x, y, start_yaw + heading_delta


def choose_safe_l_traversal(
    start_x,
    start_y,
    requested_yaw,
    distance,
    half_length,
    half_width,
    margin,
    maximum_adjustment=0.35,
):
    """选择不会越界的 L 形坑路线，返回 ``(入口航向, 转向符号)``。

    同时检查拐角和出口，避免只验证终点却让第一条臂穿过边界。优先小航向修正，并按
    左/右两种出口都尝试；算法任务层不知道这些仿真几何细节。
    """
    offsets = [0.0]
    maximum_steps = max(0, int(float(maximum_adjustment) / (pi / 36.0)))
    for step in range(1, maximum_steps + 1):
        angle = step * pi / 36.0
        offsets.extend((angle, -angle))
    for offset in offsets:
        candidate = float(requested_yaw) + offset
        for turn in (-1, 1):
            samples = (
                traversal_pose(
                    start_x,
                    start_y,
                    candidate,
                    distance,
                    progress,
                    l_turn=turn,
                )
                for progress in (0.60, 0.80, 1.0)
            )
            if all(
                pose_inside_arena(x, y, half_length, half_width, margin)
                for x, y, _yaw in samples
            ):
                return candidate, turn
    return None


def pose_inside_arena(x, y, half_length, half_width, margin) -> bool:
    """检查仿真替身是否仍在 14 m × 6 m 规则场地的安全内缩区域。"""
    usable_x = max(0.1, float(half_length) - max(0.0, float(margin)))
    usable_y = max(0.1, float(half_width) - max(0.0, float(margin)))
    return abs(float(x)) <= usable_x and abs(float(y)) <= usable_y


def choose_safe_traversal_heading(
    start_x,
    start_y,
    requested_yaw,
    distance,
    half_length,
    half_width,
    margin,
    maximum_adjustment=0.35,
):
    """为无腿仿真替身选择仍留在赛台内、且偏转最小的通过方向。

    真正四足控制器会在 PREPARING 阶段结合足端和边界状态生成轨迹；通用 Gazebo 狗
    只能做位姿覆盖。若机器人从障碍外侧接近，沿当前法向走完整个规则长度可能越出
    14 m × 6 m 赛台。这里依次尝试小角度到大角度的入口调整，只使用当前位姿和赛台
    尺寸，不读取任何障碍坐标或固定路线。
    """
    # 5° 分辨率只允许小幅修正。若必须转 90° 才能留在场内，说明入口方向或障碍身份
    # 尚未可靠确认；仿真后端应返回失败让任务换视角，绝不能为了“完成”而横穿场地。
    offsets = [0.0]
    maximum_steps = max(0, int(float(maximum_adjustment) / (pi / 36.0)))
    for step in range(1, maximum_steps + 1):
        angle = step * pi / 36.0
        offsets.extend((angle, -angle))
    for offset in offsets:
        candidate = float(requested_yaw) + offset
        end_x = float(start_x) + cos(candidate) * float(distance)
        end_y = float(start_y) + sin(candidate) * float(distance)
        if pose_inside_arena(
            end_x, end_y, half_length, half_width, margin
        ):
            return candidate
    return None


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
        self.declare_parameter("pole_duration", 9.0)
        self.declare_parameter("slope_duration", 9.0)
        # 语义专名用于区分共享粗几何的规则障碍。时长按当前参考模型总长度/0.20 m/s
        # 留少量出口余量；它们只验证任务编排，绝不是可移植到真机的开环控制参数。
        self.declare_parameter("main_slope_duration", 17.0)
        self.declare_parameter("wooden_bridge_a_duration", 23.0)
        self.declare_parameter("wooden_bridge_b_duration", 30.0)
        # 单视角只确认木桥平台、还未看到坡/板缝时，先模拟跨过当前横向结构。不能沿用
        # B 桥 5.70 m 的全长，否则从桥侧接近会把测试模型直接移出 6 m 宽赛场。
        self.declare_parameter("wooden_bridge_unknown_duration", 14.0)
        self.declare_parameter("t_shaped_stairs_duration", 16.0)
        self.declare_parameter("stabilize_duration", 0.6)
        self.declare_parameter("model_name", "generic_quadruped")
        self.declare_parameter(
            "pose_service", "/world/robocon_obstacle_field/set_pose"
        )
        self.declare_parameter("odometry_topic", "/odom")
        self.declare_parameter("pose_update_rate", 10.0)
        # 从 Action 触发点到障碍表面仍有 request.distance；跨越后不仅要让约 0.75 m 长的
        # 测试机身完全离开碰撞体，还要越过 Nav2 inflation layer。0.75 m 的旧值会让
        # base_link 落在高墙旁的 lethal cell 中，随后所有全局规划都会从非法起点失败。
        # 1.20 m 只属于无腿仿真替身；真机 Action server 必须用接触/里程计判断完成。
        self.declare_parameter("exit_clearance", 1.20)
        # 对长度已包含完整结构的坡、桥、坑和 T 台，只需让 0.75 m 测试机身离开出口；
        # 若仍叠加 1.20 m inflation 余量，会跨过出口附近尚未巡检的其他障碍。薄墙仍
        # 使用上面的 1.20 m，避免落点处于墙脚的 lethal inflation cell。
        self.declare_parameter("long_structure_exit_clearance", 0.75)
        # 规则结构沿通过方向的最小长度。Action 仍使用实时入口距离/航向；这些长度只让
        # 无腿动力学的测试替身落到结构另一侧，不包含任何 world 坐标或固定任务顺序。
        # 三根杆的 L 形中心线纵向包络是 1.00 m；S 形绕行产生的额外曲线长度已经由
        # traversal_pose 的横移表达，不能再把 1.80 m 曲线长度当成直线位移，否则从
        # 西侧参考布局进入时会错误预测落点越界并立即拒绝 Action。
        self.declare_parameter("right_angle_poles_span", 1.00)
        self.declare_parameter("gravel_wood_pit_span", 2.00)
        self.declare_parameter("height_bar_span", 0.05)
        self.declare_parameter("high_wall_span", 0.05)
        self.declare_parameter("main_slope_span", 3.00)
        self.declare_parameter("wooden_bridge_a_span", 4.35)
        self.declare_parameter("wooden_bridge_b_span", 5.70)
        # 未分型木桥也要一次离开整座结构，避免落在桥中段并把同一座桥计成第二座；
        # choose_safe_traversal_heading 会在侧向接近时自动选择不越界的通过方向。
        self.declare_parameter("wooden_bridge_unknown_span", 5.00)
        self.declare_parameter("t_shaped_stairs_span", 2.80)
        # 仅为 SetEntityPose 仿真替身提供最后一道越界保护；核心任务管理器仍完全不读取
        # 这些尺寸。正式坐标或真机 Action server 都不会使用这里的边界参数。
        self.declare_parameter("arena_half_length", 7.0)
        self.declare_parameter("arena_half_width", 3.0)
        # 测试狗机身长约 0.75 m；只让 base_link 留 0.35 m 余量会使碰撞盒压住边界，
        # 后续 Nav2 起点落在 lethal cell 中。0.75 m 同时给局部代价地图留出转向空间。
        self.declare_parameter("arena_margin", 0.75)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        self.latest_odom = None
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._odom_callback,
            10,
        )
        self.pose_client = self.create_client(
            SetEntityPose, str(self.get_parameter("pose_service").value)
        )
        self.busy = False
        self.shutdown_requested = False
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

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def goal_callback(self, goal):
        valid = (
            not self.busy
            # CLEAR=1 不应交接；坡面由任务层显式映射为 SLOPE=7。
            and goal.obstacle_type in (2, 3, 4, 5, 6, 7)
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

    def _simulation_seconds(self):
        """读取 ROS 仿真时间，暂停 Gazebo 时任务进度也必须同步暂停。

        若用 ``time.monotonic``，Gazebo 被暂停或机器负载过高时 Action 会在真实时间到点后
        误报成功，尽管测试狗在场景里一步也没走。这会让完整场地回归产生假阳性。
        """
        return self.get_clock().now().nanoseconds / 1e9

    def _set_model_pose(self, x, y, yaw) -> bool:
        """调用 Gazebo 位姿服务并等待有界墙钟时间，失败时绝不假报越障成功。"""
        request = SetEntityPose.Request()
        request.entity.name = str(self.get_parameter("model_name").value)
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = 0.0
        request.pose.orientation.z = sin(float(yaw) * 0.5)
        request.pose.orientation.w = cos(float(yaw) * 0.5)
        future = self.pose_client.call_async(request)
        deadline = time.monotonic() + 0.30
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            return False
        try:
            response = future.result()
        except Exception:
            return False
        return bool(response and response.success)

    def execute(self, handle):
        self.busy = True
        result = TraverseObstacle.Result()
        try:
            duration_parameter = {
                2: "step_duration",
                3: "pit_duration",
                4: "wall_duration",
                5: "bar_duration",
                6: "pole_duration",
                7: "slope_duration",
            }[int(handle.request.obstacle_type)]
            semantic_duration = {
                "right_angle_poles": "pole_duration",
                "gravel_wood_pit": "pit_duration",
                "height_bar": "bar_duration",
                "high_wall": "wall_duration",
                "main_slope": "main_slope_duration",
                "wooden_bridge_a": "wooden_bridge_a_duration",
                "wooden_bridge_b": "wooden_bridge_b_duration",
                "wooden_bridge_unknown": "wooden_bridge_unknown_duration",
                "t_shaped_stairs": "t_shaped_stairs_duration",
            }.get(str(handle.request.obstacle_id))
            if semantic_duration:
                duration_parameter = semantic_duration
            duration = max(0.5, float(self.get_parameter(duration_parameter).value))
            speed = max(0.03, min(0.25, float(self.get_parameter("forward_speed").value)))
            if self.latest_odom is None:
                result.success = False
                result.message = "simulation odometry unavailable"
                handle.abort()
                return result
            if not self.pose_client.wait_for_service(timeout_sec=2.0):
                result.success = False
                result.message = "Gazebo set_pose service unavailable"
                handle.abort()
                return result
            start_pose = self.latest_odom.pose.pose.position
            start_x, start_y = float(start_pose.x), float(start_pose.y)
            start_yaw = yaw_from_odometry(self.latest_odom)
            semantic_span_parameter = {
                "right_angle_poles": "right_angle_poles_span",
                "gravel_wood_pit": "gravel_wood_pit_span",
                "height_bar": "height_bar_span",
                "high_wall": "high_wall_span",
                "main_slope": "main_slope_span",
                "wooden_bridge_a": "wooden_bridge_a_span",
                "wooden_bridge_b": "wooden_bridge_b_span",
                "wooden_bridge_unknown": "wooden_bridge_unknown_span",
                "t_shaped_stairs": "t_shaped_stairs_span",
            }.get(str(handle.request.obstacle_id))
            semantic_span = (
                max(0.0, float(self.get_parameter(semantic_span_parameter).value))
                if semantic_span_parameter
                else 0.0
            )
            exit_clearance = max(
                0.20, float(self.get_parameter("exit_clearance").value)
            )
            if str(handle.request.obstacle_id) in {
                "right_angle_poles",
                "gravel_wood_pit",
                "main_slope",
                "wooden_bridge_a",
                "wooden_bridge_b",
                "wooden_bridge_unknown",
                "t_shaped_stairs",
            }:
                exit_clearance = max(
                    0.20,
                    float(
                        self.get_parameter("long_structure_exit_clearance").value
                    ),
                )
            travel_distance = max(
                speed * duration,
                max(0.0, float(handle.request.distance))
                + semantic_span
                + exit_clearance,
            )
            # Action 交接携带当前机身到障碍中心线的剩余航向误差。真实控制器会在
            # PREPARING 阶段闭环消除它；仿真替身过去忽略该字段，导致任务已算出对正
            # 方向后模型仍沿旧朝向横穿场地。这里只接受交接门限内的小角度修正。
            requested_yaw = start_yaw + max(
                -0.40, min(0.40, float(handle.request.heading_error))
            )
            l_turn = 0
            if str(handle.request.obstacle_id) == "gravel_wood_pit":
                safe_l_path = choose_safe_l_traversal(
                    start_x,
                    start_y,
                    requested_yaw,
                    travel_distance,
                    self.get_parameter("arena_half_length").value,
                    self.get_parameter("arena_half_width").value,
                    self.get_parameter("arena_margin").value,
                )
                if safe_l_path is None:
                    traversal_yaw = None
                else:
                    traversal_yaw, l_turn = safe_l_path
            else:
                traversal_yaw = choose_safe_traversal_heading(
                    start_x,
                    start_y,
                    requested_yaw,
                    travel_distance,
                    self.get_parameter("arena_half_length").value,
                    self.get_parameter("arena_half_width").value,
                    self.get_parameter("arena_margin").value,
                )
            if traversal_yaw is None:
                self._stop()
                handle.abort()
                result.success = False
                result.message = (
                    "confirmed heading has no safe simulation landing; "
                    "request another observation angle"
                )
                return result
            self.get_logger().info(
                f"sim traversal {str(handle.request.obstacle_id) or 'unclassified'}: "
                f"start=({start_x:.2f}, {start_y:.2f}, {start_yaw:.2f}), "
                f"entry={float(handle.request.distance):.2f} m, "
                f"span={semantic_span:.2f} m, travel={travel_distance:.2f} m, "
                f"heading_adjust={traversal_yaw - start_yaw:.2f} rad, "
                f"l_turn={l_turn}"
            )
            update_period = 1.0 / max(
                2.0, min(30.0, float(self.get_parameter("pose_update_rate").value))
            )
            last_update = -1e9
            started = self._simulation_seconds()
            while rclpy.ok() and self._simulation_seconds() - started < duration:
                if self.shutdown_requested:
                    self._stop()
                    handle.abort()
                    result.success = False
                    result.message = "simulation traversal stopped during shutdown"
                    return result
                if handle.is_cancel_requested:
                    self._stop()
                    handle.canceled()
                    result.success = False
                    result.message = "simulation traversal cancelled"
                    return result
                elapsed = self._simulation_seconds() - started
                progress = min(1.0, elapsed / duration)
                if elapsed - last_update >= update_period:
                    x, y, yaw = traversal_pose(
                        start_x,
                        start_y,
                        traversal_yaw,
                        travel_distance,
                        progress,
                        pole=int(handle.request.obstacle_type) == 6,
                        l_turn=l_turn,
                    )
                    if not pose_inside_arena(
                        x,
                        y,
                        self.get_parameter("arena_half_length").value,
                        self.get_parameter("arena_half_width").value,
                        self.get_parameter("arena_margin").value,
                    ):
                        self._stop()
                        handle.abort()
                        result.success = False
                        result.message = "simulation traversal would leave competition arena"
                        return result
                    if not self._set_model_pose(x, y, yaw):
                        self._stop()
                        handle.abort()
                        result.success = False
                        result.message = "Gazebo pose update failed"
                        return result
                    last_update = elapsed
                # 始终锁住平面速度，防止 DifferentialDrive 在两次 pose update 之间继续
                # 与碰撞体推挤；位姿服务是本仿真 Action 唯一的运动来源。
                self._stop()
                feedback = TraverseObstacle.Feedback()
                feedback.state = TraverseObstacle.Feedback.STATE_TRAVERSING
                feedback.progress = min(0.9, elapsed / duration * 0.9)
                feedback.message = "simulation pose-override traversal"
                handle.publish_feedback(feedback)
                time.sleep(0.05)
            # 确保最后一帧精确落在障碍出口，避免 10 Hz 离散更新留下短距离欠行程。
            final_x, final_y, final_yaw = traversal_pose(
                start_x,
                start_y,
                traversal_yaw,
                travel_distance,
                1.0,
                pole=int(handle.request.obstacle_type) == 6,
                l_turn=l_turn,
            )
            if not pose_inside_arena(
                final_x,
                final_y,
                self.get_parameter("arena_half_length").value,
                self.get_parameter("arena_half_width").value,
                self.get_parameter("arena_margin").value,
            ):
                handle.abort()
                result.success = False
                result.message = "final simulation pose outside competition arena"
                return result
            if not self._set_model_pose(final_x, final_y, final_yaw):
                handle.abort()
                result.success = False
                result.message = "final Gazebo pose update failed"
                return result
            self._stop()
            if not rclpy.ok():
                result.success = False
                result.message = "simulation shutting down"
                return result
            stabilize = max(0.0, float(self.get_parameter("stabilize_duration").value))
            deadline = self._simulation_seconds() + stabilize
            while rclpy.ok() and self._simulation_seconds() < deadline:
                self._stop()
                time.sleep(0.05)
            if self.shutdown_requested:
                handle.abort()
                result.success = False
                result.message = "simulation traversal stopped during shutdown"
                return result
            handle.succeed()
            result.success = True
            result.message = "simulation traversal completed"
            return result
        finally:
            self._stop()
            self.busy = False


def main(args=None):
    # launch 向整组进程发送 SIGINT 时，默认 rclpy handler 会先销毁 Action publisher，
    # execute callback 随后调用 abort/canceled 就产生 Ubuntu 崩溃弹窗。保持 context 存活，
    # 先让活动 goal 在 executor 内完成取消状态，再统一销毁节点。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimTraverseObstacle()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    stopping = False
    shutdown_deadline = 0.0

    def request_stop(_signum, _frame):
        nonlocal stopping, shutdown_deadline
        stopping = True
        node.shutdown_requested = True
        shutdown_deadline = time.monotonic() + 3.0

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        # Jazzy 的 MultiThreadedExecutor 在 Action 第一次完成后，某些 RMW 组合会让
        # ``spin()`` 的主线程持续命中已就绪 waitable，空闲时占满一个 CPU 核。这里给
        # 调度循环一个明确的 20 ms 让步：100 Hz 传感器由 Gazebo 独立发布、10 Hz 位姿
        # 服务仍有足够余量，
        # 但健康监控、SLAM 和 Nav2 不会再被仿真替身饿死。仍保留两个 executor 线程，
        # 因为 Action 执行期间必须由另一线程接收 SetEntityPose 的异步服务响应。
        while rclpy.ok() and (
            not stopping or (node.busy and time.monotonic() < shutdown_deadline)
        ):
            executor.spin_once(timeout_sec=0.05)
            time.sleep(0.020)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

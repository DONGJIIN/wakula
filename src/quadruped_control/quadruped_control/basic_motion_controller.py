"""Conservative joint-trajectory gait controller for simulation and bring-up.

The node consumes the final collision-monitored ``/cmd_vel`` and the existing
strongly typed crossing command.  It publishes the standard
``/leg_controller/joint_trajectory`` contract, so Gazebo's GenericSystem and a
future real ros2_control hardware plugin share the same upper layer.

This controller is position/open-loop by design.  It is useful for kinematics,
interface and low-speed suspended tests, but it is not a dynamically stable
MPC/WBC controller and must not be enabled on an unsupported free-standing
robot.
"""

import math
import signal
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from quadruped_interfaces.msg import CrossingCommand, CrossingStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .gait import GaitParameters, JOINT_NAMES, joint_targets


# Keep the quaternion helper local to avoid coupling control to safety package.
def quaternion_roll_pitch(x: float, y: float, z: float, w: float):
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        return None
    x, y, z, w = (value / norm for value in values)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def crossing_profile(mode: int, height: float, speed_scale: float):
    """Return bounded ``(vx, lift)`` for a crossing request or ``None``."""
    if (
        mode not in (CrossingCommand.STEP, CrossingCommand.CLIMB)
        or not math.isfinite(height)
        or not math.isfinite(speed_scale)
        or height < 0.0
        or not 0.0 < speed_scale <= 1.0
    ):
        return None
    base_speed = 0.16 if mode == CrossingCommand.STEP else 0.10
    maximum_lift = 0.16 if mode == CrossingCommand.STEP else 0.20
    lift = min(maximum_lift, max(0.08, height + 0.04))
    return base_speed * speed_scale, lift


class BasicMotionController(Node):
    """Publish bounded gait targets and crossing status at a fixed rate."""

    def __init__(self):
        super().__init__("basic_motion_controller")
        defaults = {
            "update_rate": 50.0,
            "command_timeout": 0.5,
            "safety_timeout": 0.5,
            "standing_height": 0.39,
            "cadence": 1.5,
            "duty_factor": 0.62,
            "swing_height": 0.06,
            "trajectory_horizon": 0.12,
            "attitude_gain": 0.8,
            "crossing_step_duration": 4.0,
            "crossing_climb_duration": 6.0,
            "require_contact_for_crossing": True,
            "allow_open_loop_crossing_success": False,
            "normal_gait": "crawl",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        rate = max(10.0, min(200.0, float(self.parameter("update_rate"))))
        self.command_timeout = max(0.05, float(self.parameter("command_timeout")))
        self.safety_timeout = max(0.05, float(self.parameter("safety_timeout")))
        self.horizon = max(0.04, min(0.5, float(self.parameter("trajectory_horizon"))))
        self.attitude_gain = max(0.0, min(2.0, float(self.parameter("attitude_gain"))))
        self.parameters = GaitParameters(
            standing_height=max(0.25, min(0.42, float(self.parameter("standing_height")))),
            cadence=max(0.4, min(3.0, float(self.parameter("cadence")))),
            duty_factor=max(0.52, min(0.85, float(self.parameter("duty_factor")))),
            swing_height=max(0.02, min(0.15, float(self.parameter("swing_height")))),
        )
        self.crossing_durations = {
            CrossingCommand.STEP: max(
                1.0, float(self.parameter("crossing_step_duration"))
            ),
            CrossingCommand.CLIMB: max(
                1.0, float(self.parameter("crossing_climb_duration"))
            ),
        }
        self.allow_open_loop_success = bool(
            self.parameter("allow_open_loop_crossing_success")
        )
        self.require_contact = bool(
            self.parameter("require_contact_for_crossing")
        ) and not self.allow_open_loop_success
        self.normal_crawl = str(self.parameter("normal_gait")).lower() != "trot"

        self.latest_cmd = Twist()
        self.last_cmd_time = None
        self.last_safety_time = None
        self.safety_stop = True
        self.roll = 0.0
        self.pitch = 0.0
        self.contacts = (False, False, False, False)
        self.gait_phase = 0.0
        self.last_update = self.get_clock().now()
        self.crossing_command = None
        self.crossing_started = None

        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, "/leg_controller/joint_trajectory", 10
        )
        self.status_pub = self.create_publisher(
            CrossingStatus, "/crossing/execution_status", 10
        )
        self.state_pub = self.create_publisher(String, "/motion/state", 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(Bool, "/safety/stop", self.safety_callback, safety_qos)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.create_subscription(
            Float32MultiArray, "/feet/contact", self.contact_callback, 10
        )
        self.create_subscription(
            CrossingCommand,
            "/crossing/execution_command",
            self.crossing_callback,
            10,
        )
        self.create_timer(1.0 / rate, self.update)
        self.get_logger().warning(
            "Basic position gait enabled: validate suspended before any real robot use"
        )

    def parameter(self, name):
        return self.get_parameter(name).value

    def cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def safety_callback(self, msg: Bool) -> None:
        self.safety_stop = bool(msg.data)
        self.last_safety_time = self.get_clock().now()
        if self.safety_stop and self.crossing_command is not None:
            self._finish_crossing(CrossingStatus.CANCELED, "safety stop")

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        orientation = quaternion_roll_pitch(q.x, q.y, q.z, q.w)
        if orientation is not None:
            self.roll, self.pitch = orientation

    def contact_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4 and all(math.isfinite(value) for value in msg.data[:4]):
            self.contacts = tuple(value > 0.5 for value in msg.data[:4])

    def crossing_callback(self, msg: CrossingCommand) -> None:
        goal_id = bytes(msg.goal_id.uuid)
        if msg.command == CrossingCommand.CANCEL:
            if self.crossing_command is not None and goal_id == bytes(
                self.crossing_command.goal_id.uuid
            ):
                self._finish_crossing(CrossingStatus.CANCELED, "cancel received")
            return
        if msg.command != CrossingCommand.START or crossing_profile(
            int(msg.mode), float(msg.obstacle_height), float(msg.speed_scale)
        ) is None:
            return
        if self.crossing_command is not None:
            self._publish_status(
                msg,
                CrossingStatus.FAILED,
                CrossingStatus.RECOVERING,
                0.0,
                False,
                "basic controller busy",
            )
            return
        self.crossing_command = msg
        self.crossing_started = time.monotonic()
        self._publish_status(
            msg,
            CrossingStatus.RUNNING,
            CrossingStatus.ACCEPTED,
            0.0,
            False,
            "basic controller accepted",
        )

    def _safety_fresh(self, now) -> bool:
        return self.last_safety_time is not None and (
            now - self.last_safety_time
        ).nanoseconds / 1e9 <= self.safety_timeout

    def update(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, min(0.1, (now - self.last_update).nanoseconds / 1e9))
        self.last_update = now
        safe = self._safety_fresh(now) and not self.safety_stop
        vx = vy = wz = lift = 0.0
        crawl = self.normal_crawl
        state = "HOLD"

        if self.crossing_command is not None:
            if not safe:
                self._finish_crossing(CrossingStatus.CANCELED, "safety heartbeat lost")
            else:
                elapsed = time.monotonic() - self.crossing_started
                duration = self.crossing_durations[int(self.crossing_command.mode)]
                progress = min(1.0, elapsed / duration)
                vx, lift = crossing_profile(
                    int(self.crossing_command.mode),
                    float(self.crossing_command.obstacle_height),
                    float(self.crossing_command.speed_scale),
                )
                crawl = True
                state = "CROSSING"
                phase = (
                    CrossingStatus.PREPARING
                    if progress < 0.10
                    else CrossingStatus.EXECUTING
                    if progress < 0.90
                    else CrossingStatus.VERIFYING_CONTACT
                )
                self._publish_status(
                    self.crossing_command,
                    CrossingStatus.RUNNING,
                    phase,
                    progress,
                    sum(self.contacts) >= 3,
                    "timed crawl profile",
                )
                if progress >= 1.0:
                    contact = sum(self.contacts) >= 3
                    if contact or (self.allow_open_loop_success and not self.require_contact):
                        self._finish_crossing(
                            CrossingStatus.SUCCEEDED,
                            "basic profile complete",
                            contact_verified=contact or self.allow_open_loop_success,
                        )
                    else:
                        self._finish_crossing(
                            CrossingStatus.FAILED,
                            "foot contact not verified",
                        )

        if self.crossing_command is None:
            command_fresh = self.last_cmd_time is not None and (
                now - self.last_cmd_time
            ).nanoseconds / 1e9 <= self.command_timeout
            if safe and command_fresh:
                vx = float(self.latest_cmd.linear.x)
                vy = float(self.latest_cmd.linear.y)
                wz = float(self.latest_cmd.angular.z)
                if abs(vx) + abs(vy) + abs(wz) > 1e-3:
                    state = "WALK"

        cadence = self.parameters.cadence * (0.7 if crawl else 1.0)
        if state in ("WALK", "CROSSING"):
            self.gait_phase = (self.gait_phase + dt * cadence) % 1.0
        targets = joint_targets(
            self.gait_phase,
            vx,
            vy,
            wz,
            self.parameters,
            swing_height_override=lift,
            crawl=crawl,
            roll=self.roll,
            pitch=self.pitch,
            attitude_gain=self.attitude_gain if safe else 0.0,
        )
        trajectory = JointTrajectory()
        trajectory.header.stamp = now.to_msg()
        trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [targets[name] for name in JOINT_NAMES]
        nanoseconds = int(self.horizon * 1e9)
        point.time_from_start = Duration(
            sec=nanoseconds // 1_000_000_000,
            nanosec=nanoseconds % 1_000_000_000,
        )
        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)
        self.state_pub.publish(String(data=state if safe else "SAFETY_HOLD"))

    def _finish_crossing(
        self, state: int, message: str, contact_verified: bool = False
    ) -> None:
        if self.crossing_command is None:
            return
        command = self.crossing_command
        phase = (
            CrossingStatus.VERIFYING_CONTACT
            if state == CrossingStatus.SUCCEEDED
            else CrossingStatus.RECOVERING
        )
        self._publish_status(
            command,
            state,
            phase,
            1.0 if state == CrossingStatus.SUCCEEDED else 0.0,
            contact_verified,
            message,
        )
        self.crossing_command = None
        self.crossing_started = None

    def _publish_status(
        self, command, state, phase, progress, contact, message
    ) -> None:
        status = CrossingStatus()
        status.goal_id = command.goal_id
        status.state = state
        status.phase = phase
        status.progress = float(max(0.0, min(1.0, progress)))
        status.contact_verified = bool(contact)
        status.message = message
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = BasicMotionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A bridge may disappear while rclpy is taking its final sensor sample
        # during launch shutdown.  Preserve genuine runtime conversion errors.
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

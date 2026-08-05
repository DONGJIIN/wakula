"""Competition-aware obstacle course state machine.

This node encodes the V1.0 Robocon obstacle-course constraints as a planning
layer. Perception and the vendor gait controller remain replaceable: they feed
obstacle hints, foot contacts, stair progress and completion events into this
node, while this node owns safe transitions, retry rules, score and timeout.
"""

from math import hypot
from typing import Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String, UInt8MultiArray


PROFILES: Dict[str, dict] = {
    "straight_poles": {
        "action": "FOLLOW_S_CORRIDOR",
        "score": 150,
        "speed": 0.35,
        "rule": "S path through all must-pass zones",
    },
    "gravel_wood_pit": {
        "action": "CROSS_SHORT_SIDE_SINGLE_SUPPORT",
        "score": 150,
        "speed": 0.25,
        "max_ground_contacts": 1,
        "rule": "enter and leave via the 1 m short side; max one foot on ground",
    },
    "height_bar": {
        "action": "LOW_PROFILE",
        "score": 150,
        "speed": 0.25,
        "max_body_height": 0.30,
        "rule": "pass below the 0.30 m bar without knocking it down",
    },
    "slope": {
        "action": "SLOPE_TRAVERSE",
        "score": 150,
        "speed": 0.30,
        "max_ground_contacts": 1,
        "min_travel_m": 1.0,
        "rule": "travel at least 1 m along the 3 m slope direction",
    },
    "bridge_a": {
        "action": "CROSS_BRIDGE_A_SINGLE_SUPPORT",
        "score": 150,
        "speed": 0.25,
        "max_ground_contacts": 1,
        "rule": "platform to platform; max one foot on ground",
    },
    "bridge_b": {
        "action": "CROSS_BRIDGE_B_SINGLE_SUPPORT",
        "score": 150,
        "speed": 0.25,
        "max_ground_contacts": 1,
        "rule": "platform to platform; max one foot on ground",
    },
    "t_stairs": {
        "action": "CROSS_T_STAIRS_SINGLE_SUPPORT",
        "score": 150,
        "speed": 0.20,
        "max_ground_contacts": 1,
        "required_stair_levels": 4,
        "rule": "touch every stair top; max one foot on ground",
    },
    "high_wall": {
        "action": "CLIMB_OR_JUMP_0P30_WALL",
        "score": 150,
        "speed": 0.15,
        "rule": "cross above or climb over the 0.30 m wall",
    },
}


class CompetitionObstacleManager(Node):
    """Score-aware FSM for the 210-second obstacle competition."""

    def __init__(self):
        super().__init__("competition_obstacle_manager")
        self.declare_parameter("time_limit_s", 210.0)
        self.declare_parameter("autonomous", True)
        self.declare_parameter("return_bonus", 100)
        self.declare_parameter("stair_levels_required", 4)
        self.declare_parameter("obstacle_order", list(PROFILES.keys()))
        self.time_limit = float(self.get_parameter("time_limit_s").value)
        self.autonomous = bool(self.get_parameter("autonomous").value)
        self.return_bonus = int(self.get_parameter("return_bonus").value)
        self.stair_levels_required = int(self.get_parameter("stair_levels_required").value)
        configured_order = list(self.get_parameter("obstacle_order").value)
        self.order = [name for name in configured_order if name in PROFILES]
        self.order = self.order or list(PROFILES.keys())

        self.mode_pub = self.create_publisher(String, "/crossing/mode", 10)
        self.action_pub = self.create_publisher(String, "/crossing/action", 10)
        self.speed_pub = self.create_publisher(Float32, "/crossing/speed_scale", 10)
        self.current_pub = self.create_publisher(String, "/competition/current_obstacle", 10)
        self.state_pub = self.create_publisher(String, "/competition/state", 10)
        self.score_pub = self.create_publisher(Int32, "/competition/score", 10)
        self.time_pub = self.create_publisher(Float32, "/competition/time_remaining", 10)

        self.create_subscription(String, "/competition/obstacle_hint", self.hint_callback, 10)
        self.create_subscription(Bool, "/competition/obstacle_complete", self.complete_callback, 10)
        self.create_subscription(Bool, "/competition/obstacle_failed", self.failed_callback, 10)
        self.create_subscription(Bool, "/competition/retry", self.retry_callback, 10)
        self.create_subscription(Bool, "/competition/returned_to_start", self.return_callback, 10)
        self.create_subscription(UInt8MultiArray, "/foot_contacts", self.contacts_callback, 10)
        self.create_subscription(Int32, "/competition/stair_levels_touched", self.stair_callback, 10)
        self.create_subscription(Int32, "/competition/stair_sides_completed", self.stair_sides_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.state = "SEARCH"
        self.index = 0
        self.completed = set()
        self.score = 0
        self.start_time = self.get_clock().now()
        self.obstacle_start_pose = None
        self.travel_m = 0.0
        self.ground_contacts = 0
        self.stair_levels = 0
        self.stair_sides = 0
        self.failure_latched = False
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"Competition FSM ready: {self.time_limit:.0f}s, {len(self.order)} obstacles, "
            f"autonomous scoring={self.autonomous}"
        )
        self.publish_outputs("STOP", "WAIT_FOR_OBSTACLE", 0.0)

    @property
    def current_obstacle(self):
        remaining = [name for name in self.order if name not in self.completed]
        return remaining[0] if remaining else None

    def tick(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        remaining = max(0.0, self.time_limit - elapsed)
        self.time_pub.publish(Float32(data=float(remaining)))
        if elapsed >= self.time_limit and self.state not in ("FINISHED", "TIMEOUT"):
            self.state = "TIMEOUT"
            self.publish_outputs("STOP", "TIME_LIMIT_REACHED", 0.0)
            self.get_logger().error("Competition time limit reached; stopping")
            return
        if self.state == "SEARCH" and self.current_obstacle:
            profile = PROFILES[self.current_obstacle]
            self.publish_outputs("WALK", f"APPROACH_{self.current_obstacle.upper()}", profile["speed"])
        elif self.state == "RETURN":
            self.publish_outputs("WALK", "RETURN_TO_SELECTED_START", 0.35)

    def hint_callback(self, msg: String) -> None:
        name = msg.data.strip().lower()
        if name not in PROFILES or name in self.completed or self.state in ("TIMEOUT", "FINISHED"):
            return
        self.index = self.order.index(name) if name in self.order else self.index
        self.obstacle_start_pose = None
        self.travel_m = 0.0
        self.stair_levels = 0
        self.stair_sides = 0
        self.failure_latched = False
        self.state = "EXECUTE"
        profile = PROFILES[name]
        self.publish_outputs("STEP", profile["action"], profile["speed"])
        self.get_logger().info(f"Selected obstacle: {name}; rule: {profile['rule']}")

    def contacts_callback(self, msg: UInt8MultiArray) -> None:
        self.ground_contacts = sum(1 for value in msg.data[:4] if value > 0)
        name = self.current_obstacle
        if self.state == "EXECUTE" and name:
            limit = PROFILES[name].get("max_ground_contacts")
            if limit is not None and self.ground_contacts > limit:
                self.failure_latched = True
                self.state = "RETRY_REQUIRED"
                self.publish_outputs("STOP", "RETRY_OBSTACLE_CONTACT_RULE", 0.0)
                self.get_logger().warning(
                    f"{name} failed contact rule: {self.ground_contacts} ground contacts > {limit}"
                )

    def stair_callback(self, msg: Int32) -> None:
        self.stair_levels = max(self.stair_levels, int(msg.data))

    def stair_sides_callback(self, msg: Int32) -> None:
        # 0 = none, 1 = only up or down, 2 = both directions.
        self.stair_sides = max(0, min(2, int(msg.data)))

    def odom_callback(self, msg: Odometry) -> None:
        if self.state != "EXECUTE":
            return
        point = msg.pose.pose.position
        current = (float(point.x), float(point.y))
        if self.obstacle_start_pose is None:
            self.obstacle_start_pose = current
            return
        # Euclidean distance is conservative and independent of obstacle heading.
        self.travel_m = hypot(current[0] - self.obstacle_start_pose[0], current[1] - self.obstacle_start_pose[1])

    def complete_callback(self, msg: Bool) -> None:
        if not msg.data or self.state != "EXECUTE" or self.failure_latched:
            return
        name = self.current_obstacle
        if not name:
            return
        profile = PROFILES[name]
        if self.ground_contacts > profile.get("max_ground_contacts", 4):
            self.failed_callback(Bool(data=True))
            return
        if self.travel_m < profile.get("min_travel_m", 0.0):
            self.publish_outputs("STOP", "CONTINUE_REQUIRED_DISTANCE", profile["speed"])
            self.get_logger().warning(f"{name} completion rejected: only {self.travel_m:.2f} m traversed")
            return
        if name == "t_stairs" and self.stair_levels < self.stair_levels_required:
            self.publish_outputs("STOP", "TOUCH_REMAINING_STAIR_TOPS", profile["speed"])
            self.get_logger().warning(
                f"T stairs completion rejected: {self.stair_levels}/{self.stair_levels_required} levels"
            )
            return
        if name == "t_stairs" and self.stair_sides < 1:
            self.publish_outputs("STOP", "REPORT_STAIR_DIRECTION", profile["speed"])
            self.get_logger().warning("T stairs completion rejected: no up/down direction reported")
            return
        self.completed.add(name)
        obstacle_score = profile["score"] if self.autonomous else round(profile["score"] * 2 / 3)
        if name == "t_stairs" and self.stair_sides == 1:
            obstacle_score = obstacle_score // 2
        self.score += int(obstacle_score)
        self.score_pub.publish(Int32(data=self.score))
        self.state = "SEARCH" if self.current_obstacle else "RETURN"
        self.publish_outputs("WALK", "SEARCH_NEXT_OBSTACLE" if self.current_obstacle else "RETURN_TO_SELECTED_START", 0.35)
        self.get_logger().info(f"Obstacle complete: {name}; score={self.score}")

    def failed_callback(self, msg: Bool) -> None:
        if not msg.data or self.state in ("FINISHED", "TIMEOUT"):
            return
        self.failure_latched = True
        self.state = "RETRY_REQUIRED"
        self.publish_outputs("STOP", "RETRY_OBSTACLE", 0.0)

    def retry_callback(self, msg: Bool) -> None:
        if msg.data and self.state == "RETRY_REQUIRED":
            self.failure_latched = False
            self.travel_m = 0.0
            self.stair_levels = 0
            self.stair_sides = 0
            self.obstacle_start_pose = None
            self.state = "EXECUTE"
            name = self.current_obstacle
            profile = PROFILES[name]
            self.publish_outputs("STEP", profile["action"], profile["speed"])

    def return_callback(self, msg: Bool) -> None:
        if msg.data and self.state == "RETURN":
            self.score += self.return_bonus
            self.score_pub.publish(Int32(data=self.score))
            self.state = "FINISHED"
            self.publish_outputs("STOP", "FINISHED_RETURN_BONUS", 0.0)
            self.get_logger().info(f"Returned to start; +{self.return_bonus} bonus, score={self.score}")

    def publish_outputs(self, mode: str, action: str, speed: float) -> None:
        self.mode_pub.publish(String(data=mode))
        self.action_pub.publish(String(data=action))
        self.current_pub.publish(String(data=self.current_obstacle or "none"))
        self.state_pub.publish(String(data=self.state))
        self.speed_pub.publish(Float32(data=float(speed)))
        self.score_pub.publish(Int32(data=self.score))


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

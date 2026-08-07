"""Nav2 waypoint bridge for the Robocon obstacle-course FSM."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CourseWaypointNavigator(Node):
    """Navigate to each configured obstacle approach pose before crossing."""

    def __init__(self):
        super().__init__("course_waypoint_navigator")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("obstacle_names", [""])
        self.declare_parameter("obstacle_x", [0.0])
        self.declare_parameter("obstacle_y", [0.0])
        self.declare_parameter("obstacle_yaw", [0.0])
        self.map_frame = str(self.get_parameter("map_frame").value)
        names = list(self.get_parameter("obstacle_names").value)
        xs = list(self.get_parameter("obstacle_x").value)
        ys = list(self.get_parameter("obstacle_y").value)
        yaws = list(self.get_parameter("obstacle_yaw").value)
        count = min(len(names), len(xs), len(ys), len(yaws))
        self.poses = {
            str(names[i]): (float(xs[i]), float(ys[i]), float(yaws[i]))
            for i in range(count)
        }
        self.current_obstacle = None
        self.last_goal_name = None
        self.goal_name = None
        self.goal_handle = None
        self.create_subscription(
            String, "/competition/current_obstacle", self.obstacle_callback, 10
        )
        self.hint_pub = self.create_publisher(String, "/competition/obstacle_hint", 10)
        self.failed_pub = self.create_publisher(Bool, "/competition/obstacle_failed", 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.timer = self.create_timer(0.2, self.try_send_goal)
        self.get_logger().info(
            f"Course waypoint navigator loaded {len(self.poses)} approach poses"
        )

    def obstacle_callback(self, msg: String) -> None:
        name = msg.data.strip()
        if name in self.poses and name != self.current_obstacle:
            self.current_obstacle = name
            self.last_goal_name = None
            self.goal_name = None
            self.goal_handle = None

    def try_send_goal(self) -> None:
        name = self.current_obstacle
        if not name or name == self.last_goal_name or self.goal_handle is not None:
            return
        if not self.client.wait_for_server(timeout_sec=0.0):
            return
        x, y, yaw = self.poses[name]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.last_goal_name = name
        self.goal_name = name
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f"Navigating to {name} approach pose ({x:.2f}, {y:.2f})")

    def goal_response_callback(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Could not send Nav2 goal: {exc}")
            self.goal_handle = None
            self.goal_name = None
            return
        if not handle.accepted:
            self.get_logger().warning("Nav2 rejected approach goal")
            self.failed_pub.publish(Bool(data=True))
            self.goal_handle = None
            self.goal_name = None
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future) -> None:
        status = future.result().status
        name = self.goal_name
        self.goal_handle = None
        self.goal_name = None
        if status == GoalStatus.STATUS_SUCCEEDED and name:
            self.hint_pub.publish(String(data=name))
            self.get_logger().info(f"Reached {name} approach pose; starting crossing")
        else:
            self.failed_pub.publish(Bool(data=True))
            self.get_logger().warning(f"Could not reach {name} approach pose (status {status})")


def main(args=None):
    rclpy.init(args=args)
    node = CourseWaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node


class NavigationNode(Node):
    """Extension point for quadruped-specific navigation behavior."""

    def __init__(self):
        super().__init__("quadruped_navigation")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("obstacle_topic", "/terrain/obstacle")
        self.get_logger().info("Quadruped navigation node is ready")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

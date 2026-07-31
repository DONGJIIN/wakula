from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    model = PathJoinSubstitution(
        [FindPackageShare("quadruped_description"), "urdf", "quadruped.urdf.xacro"]
    )
    robot_description = {"robot_description": Command([FindExecutable(name="xacro"), " ", model])}
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot_description]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", output="screen"),
    ])


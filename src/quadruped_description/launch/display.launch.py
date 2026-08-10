from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("quadruped_description")
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=PathJoinSubstitution(
                    [package_share, "urdf", "quadruped.urdf.xacro"]
                ),
                description="Absolute path to the quadruped Xacro model.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "quadruped.rviz"]
                ),
                description="Absolute path to the RViz configuration.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )

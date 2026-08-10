"""Start the shared robot, perception, planning, and safety pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_file(package: str, folder: str, filename: str):
    """Build a package share path without repeating launch boilerplate."""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_control = LaunchConfiguration("use_control")
    vision = LaunchConfiguration("vision")
    camera_topic = LaunchConfiguration("camera_topic")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    terrain_params_file = LaunchConfiguration("terrain_params_file")
    vision_params_file = LaunchConfiguration("vision_params_file")
    crossing_params_file = LaunchConfiguration("crossing_params_file")
    hardware_params_file = LaunchConfiguration("hardware_params_file")
    competition = LaunchConfiguration("competition")
    auto_crossing = LaunchConfiguration("auto_crossing")
    safety_supervisor = LaunchConfiguration("safety_supervisor")
    mock_hardware = LaunchConfiguration("mock_hardware")

    description_file = package_file(
        "quadruped_description", "urdf", "quadruped.urdf.xacro"
    )
    controllers_file = package_file(
        "quadruped_control", "config", "controllers.yaml"
    )
    default_terrain_file = package_file(
        "quadruped_perception", "config", "terrain.yaml"
    )
    default_vision_file = package_file(
        "quadruped_perception", "config", "vision.yaml"
    )
    default_crossing_file = package_file(
        "quadruped_planning", "config", "crossing.yaml"
    )
    competition_file = package_file(
        "quadruped_planning", "config", "competition.yaml"
    )
    waypoint_file = package_file(
        "quadruped_planning", "config", "course_waypoints.yaml"
    )
    default_hardware_file = package_file(
        "quadruped_hardware", "config", "hardware.yaml"
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file]),
        value_type=str,
    )

    common_time = {"use_sim_time": use_sim_time}
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_control", default_value="true"),
            DeclareLaunchArgument("vision", default_value="true"),
            DeclareLaunchArgument(
                "camera_topic",
                default_value="",
                description="RGB topic override; empty auto-detects common defaults.",
            ),
            DeclareLaunchArgument(
                "point_cloud_topic",
                default_value="",
                description="Point-cloud override; empty auto-detects common defaults.",
            ),
            DeclareLaunchArgument(
                "terrain_params_file", default_value=default_terrain_file
            ),
            DeclareLaunchArgument(
                "vision_params_file", default_value=default_vision_file
            ),
            DeclareLaunchArgument(
                "crossing_params_file", default_value=default_crossing_file
            ),
            DeclareLaunchArgument("competition", default_value="false"),
            DeclareLaunchArgument("auto_crossing", default_value="true"),
            DeclareLaunchArgument("safety_supervisor", default_value="true"),
            DeclareLaunchArgument("mock_hardware", default_value="false"),
            DeclareLaunchArgument(
                "hardware_params_file", default_value=default_hardware_file
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                condition=IfCondition(use_control),
                parameters=[
                    {"robot_description": robot_description},
                    controllers_file,
                    common_time,
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                condition=IfCondition(use_control),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "leg_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                condition=IfCondition(use_control),
            ),
            Node(
                package="quadruped_perception",
                executable="terrain_analyzer",
                output="screen",
                parameters=[
                    terrain_params_file,
                    {"input_topic": point_cloud_topic},
                    common_time,
                ],
            ),
            Node(
                package="quadruped_perception",
                executable="vision_obstacle_detector",
                output="screen",
                parameters=[
                    vision_params_file,
                    {"image_topic": camera_topic},
                    common_time,
                ],
                condition=IfCondition(vision),
            ),
            Node(
                package="quadruped_planning",
                executable="obstacle_crossing_manager",
                output="screen",
                parameters=[crossing_params_file, common_time],
                condition=UnlessCondition(competition),
            ),
            Node(
                package="quadruped_planning",
                executable="crossing_action_server",
                output="screen",
                parameters=[crossing_params_file, common_time],
            ),
            Node(
                package="quadruped_planning",
                executable="crossing_action_coordinator",
                output="screen",
                parameters=[crossing_params_file, common_time],
                condition=IfCondition(auto_crossing),
            ),
            Node(
                package="quadruped_hardware",
                executable="system_safety_supervisor",
                output="screen",
                parameters=[hardware_params_file, common_time],
                condition=IfCondition(safety_supervisor),
            ),
            Node(
                package="quadruped_hardware",
                executable="mock_sdk_adapter",
                output="screen",
                parameters=[hardware_params_file, common_time],
                condition=IfCondition(mock_hardware),
            ),
            Node(
                package="quadruped_planning",
                executable="competition_obstacle_manager",
                output="screen",
                parameters=[competition_file, common_time],
                condition=IfCondition(competition),
            ),
            Node(
                package="quadruped_planning",
                executable="course_waypoint_navigator",
                output="screen",
                parameters=[waypoint_file, common_time],
                condition=IfCondition(competition),
            ),
            Node(
                package="quadruped_planning",
                executable="cmd_vel_gate",
                output="screen",
                parameters=[crossing_params_file, common_time],
            ),
        ]
    )

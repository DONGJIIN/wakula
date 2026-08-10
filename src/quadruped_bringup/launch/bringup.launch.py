"""Start only the hardware-independent perception and navigation helpers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_file(package: str, folder: str, filename: str):
    """Build an installed package resource path."""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    """声明感知、地形安全评估和 Nav2 速度约束的公共启动入口。"""
    use_sim_time = LaunchConfiguration("use_sim_time")
    vision = LaunchConfiguration("vision")
    robot_model = LaunchConfiguration("robot_model")
    camera_topic = LaunchConfiguration("camera_topic")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    terrain_params_file = LaunchConfiguration("terrain_params_file")
    vision_params_file = LaunchConfiguration("vision_params_file")
    terrain_navigation_params_file = LaunchConfiguration(
        "terrain_navigation_params_file"
    )

    description_file = package_file(
        "quadruped_description", "urdf", "quadruped.urdf.xacro"
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file]),
        value_type=str,
    )
    common_time = {"use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("vision", default_value="true"),
            DeclareLaunchArgument("robot_model", default_value="true"),
            DeclareLaunchArgument("camera_topic", default_value=""),
            DeclareLaunchArgument("point_cloud_topic", default_value=""),
            DeclareLaunchArgument(
                "terrain_params_file",
                default_value=package_file(
                    "quadruped_perception", "config", "terrain.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "vision_params_file",
                default_value=package_file(
                    "quadruped_perception", "config", "vision.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "terrain_navigation_params_file",
                default_value=package_file(
                    "quadruped_planning", "config", "terrain_navigation.yaml"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}, common_time],
                condition=IfCondition(robot_model),
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
                package="quadruped_perception",
                executable="perception_fusion",
                output="screen",
                parameters=[common_time],
                condition=IfCondition(vision),
            ),
            Node(
                package="quadruped_planning",
                executable="terrain_safety_assessor",
                output="screen",
                parameters=[
                    terrain_navigation_params_file,
                    {
                        "prefer_fused_obstacle": ParameterValue(
                            vision, value_type=bool
                        )
                    },
                    common_time,
                ],
            ),
            Node(
                package="quadruped_planning",
                executable="navigation_speed_gate",
                output="screen",
                parameters=[terrain_navigation_params_file, common_time],
            ),
        ]
    )

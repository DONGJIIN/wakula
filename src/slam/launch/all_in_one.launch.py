"""One-command entry for SLAM, Nav2, OpenCV, terrain and safety nodes."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_file(package: str, folder: str, filename: str):
    """Build a package share path without hard-coded installation paths."""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    """Expose every common switch and delegate to the maintained stack."""
    compatibility_launch = package_file(
        "slam", "launch", "sensor_compat.launch.py"
    )
    argument_names = (
        "sensor_profile",
        "sensor_profiles_file",
        "scan_topic",
        "odom_topic",
        "camera_topic",
        "point_cloud_topic",
        "use_sim_time",
        "use_control",
        "rviz",
        "vision",
        "competition",
        "nav2_autostart",
    )
    forwarded_arguments = {
        name: LaunchConfiguration(name) for name in argument_names
    }

    default_profiles = package_file(
        "slam", "config", "sensor_profiles.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_profile",
                default_value="ros_default",
                description=(
                    "Common sensor profile; ros_default keeps ROS names."
                ),
            ),
            DeclareLaunchArgument(
                "sensor_profiles_file",
                default_value=default_profiles,
                description=(
                    "YAML containing replaceable sensor topic profiles."
                ),
            ),
            DeclareLaunchArgument("scan_topic", default_value=""),
            DeclareLaunchArgument("odom_topic", default_value=""),
            DeclareLaunchArgument("camera_topic", default_value=""),
            DeclareLaunchArgument("point_cloud_topic", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # Motor control stays off until a hardware plugin is ready.
            DeclareLaunchArgument("use_control", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("vision", default_value="true"),
            DeclareLaunchArgument("competition", default_value="false"),
            DeclareLaunchArgument("nav2_autostart", default_value="true"),
            LogInfo(
                msg=(
                    "Starting Wakula all-in-one: robot model, SLAM Toolbox, "
                    "Nav2, OpenCV, terrain analysis, crossing decisions, "
                    "velocity safety and optional RViz."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(compatibility_launch),
                launch_arguments=forwarded_arguments.items(),
            ),
        ]
    )

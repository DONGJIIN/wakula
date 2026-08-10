"""Backward-compatible alias for the unified slam.launch.py entry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_file(package: str, folder: str, filename: str):
    """Build one package share path."""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    """Forward legacy compatibility arguments to the unified entry."""
    forwarded_names = (
        "sensor_profile",
        "sensor_profiles_file",
        "scan_topic",
        "odom_topic",
        "camera_topic",
        "point_cloud_topic",
        "use_sim_time",
        "rviz",
        "vision",
        "nav2_autostart",
    )
    defaults = {
        "sensor_profile": "ros_default",
        "sensor_profiles_file": package_file(
            "slam", "config", "sensor_profiles.yaml"
        ),
        "scan_topic": "",
        "odom_topic": "",
        "camera_topic": "",
        "point_cloud_topic": "",
        "use_sim_time": "false",
        "rviz": "true",
        "vision": "true",
        "nav2_autostart": "true",
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=defaults[name])
        for name in forwarded_names
    ]
    return LaunchDescription(
        [
            *declarations,
            LogInfo(
                msg=(
                    "sensor_compat.launch.py is a compatibility alias; "
                    "prefer: ros2 launch slam slam.launch.py"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    package_file("slam", "launch", "slam.launch.py")
                ),
                launch_arguments={
                    name: LaunchConfiguration(name) for name in forwarded_names
                }.items(),
            ),
        ]
    )

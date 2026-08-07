"""Add SLAM Toolbox, Nav2, and RViz to the shared quadruped bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_file(package: str, folder: str, filename: str):
    """Build a package share path."""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_control = LaunchConfiguration("use_control")
    rviz = LaunchConfiguration("rviz")
    vision = LaunchConfiguration("vision")
    camera_topic = LaunchConfiguration("camera_topic")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    competition = LaunchConfiguration("competition")
    nav2_autostart = LaunchConfiguration("nav2_autostart")

    bringup_launch = package_file(
        "quadruped_bringup", "launch", "bringup.launch.py"
    )
    slam_launch = package_file(
        "slam_toolbox", "launch", "online_async_launch.py"
    )
    nav2_launch = package_file("slam", "launch", "navigation.launch.py")
    slam_params = package_file("slam", "config", "slam.yaml")
    nav2_params = package_file("slam", "config", "nav2.yaml")
    rviz_config = package_file("slam", "rviz", "slam.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_control", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("vision", default_value="true"),
            DeclareLaunchArgument("camera_topic", default_value=""),
            DeclareLaunchArgument("point_cloud_topic", default_value=""),
            DeclareLaunchArgument("competition", default_value="false"),
            DeclareLaunchArgument("nav2_autostart", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_control": use_control,
                    "vision": vision,
                    "camera_topic": camera_topic,
                    "point_cloud_topic": point_cloud_topic,
                    "competition": competition,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": slam_params,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "autostart": nav2_autostart,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(rviz),
                output="screen",
            ),
        ]
    )

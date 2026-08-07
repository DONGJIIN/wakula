"""Add SLAM Toolbox, Nav2, and RViz to the shared quadruped bringup."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def package_file(package: str, folder: str, filename: str):
    """Build a package share path."""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_control = LaunchConfiguration("use_control")
    rviz = LaunchConfiguration("rviz")
    vision = LaunchConfiguration("vision")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
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
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan input; keeps the ROS navigation default.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Odometry input; keeps the ROS navigation default.",
            ),
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
                    "odom_topic": odom_topic,
                    "camera_topic": camera_topic,
                    "point_cloud_topic": point_cloud_topic,
                    "competition": competition,
                }.items(),
            ),
            GroupAction(
                actions=[
                    # Slam Toolbox reads /scan from its standard parameter file;
                    # one scoped remap adapts any LaserScan driver.
                    SetRemap(src="/scan", dst=scan_topic),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(slam_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "slam_params_file": slam_params,
                        }.items(),
                    ),
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "autostart": nav2_autostart,
                    "scan_topic": scan_topic,
                    "odom_topic": odom_topic,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[("/scan", scan_topic)],
                condition=IfCondition(rviz),
                output="screen",
            ),
        ]
    )

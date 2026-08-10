"""One-command Wakula entry for SLAM, Nav2, OpenCV and safety nodes."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

from slam.sensor_profiles import load_sensor_profiles, resolve_sensor_topics


def package_file(package: str, folder: str, filename: str):
    """Build a package share path without hard-coded installation paths."""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def _launch_complete_stack(context):
    """Resolve the sensor profile and create the complete scoped stack."""
    profile_name = LaunchConfiguration("sensor_profile").perform(context)
    profiles_file = LaunchConfiguration("sensor_profiles_file").perform(
        context
    )
    profiles = load_sensor_profiles(profiles_file)
    overrides = {
        key: LaunchConfiguration(key).perform(context)
        for key in (
            "scan_topic",
            "odom_topic",
            "camera_topic",
            "point_cloud_topic",
        )
    }
    topics = resolve_sensor_topics(profiles, profile_name, overrides)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_control = LaunchConfiguration("use_control")
    vision = LaunchConfiguration("vision")
    competition = LaunchConfiguration("competition")
    nav2_autostart = LaunchConfiguration("nav2_autostart")
    slam_enabled = LaunchConfiguration("slam_enabled")
    nav2_enabled = LaunchConfiguration("nav2_enabled")
    rviz_enabled = LaunchConfiguration("rviz")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file("quadruped_bringup", "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_control": use_control,
            "vision": vision,
            "camera_topic": topics["camera_topic"],
            "point_cloud_topic": topics["point_cloud_topic"],
            "terrain_params_file": LaunchConfiguration("terrain_params_file"),
            "vision_params_file": LaunchConfiguration("vision_params_file"),
            "crossing_params_file": LaunchConfiguration("crossing_params_file"),
            "competition": competition,
        }.items(),
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file("slam_toolbox", "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": LaunchConfiguration("slam_params_file"),
        }.items(),
        condition=IfCondition(slam_enabled),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file("slam", "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": LaunchConfiguration("nav2_params_file"),
            "autostart": nav2_autostart,
            "log_level": LaunchConfiguration("nav2_log_level"),
        }.items(),
        condition=IfCondition(nav2_enabled),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz_enabled),
        output="screen",
    )

    summary = (
        f"Wakula one-command profile={profile_name}: "
        f"scan={topics['scan_topic']}, odom={topics['odom_topic']}, "
        f"image={topics['camera_topic'] or 'auto'}, "
        f"points={topics['point_cloud_topic'] or 'auto'}"
    )
    # Remaps are scoped to every included consumer, so one override reaches
    # SLAM Toolbox, Nav2, Collision Monitor, readiness checks and RViz.
    return [
        LogInfo(msg=summary),
        GroupAction(
            actions=[
                SetRemap(src="/scan", dst=topics["scan_topic"]),
                SetRemap(src="/odom", dst=topics["odom_topic"]),
                bringup,
                slam_toolbox,
                nav2,
                rviz,
            ]
        ),
    ]


def generate_launch_description():
    """Declare all common controls and start the stack with one command."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_profile",
                default_value="ros_default",
                description="Profile from sensor_profiles_file.",
            ),
            DeclareLaunchArgument(
                "sensor_profiles_file",
                default_value=package_file(
                    "slam", "config", "sensor_profiles.yaml"
                ),
                description="YAML containing common sensor topic profiles.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="",
                description="LaserScan override; empty uses the profile.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="",
                description="Odometry override; empty uses the profile.",
            ),
            DeclareLaunchArgument(
                "camera_topic",
                default_value="",
                description="Image override; empty uses profile/auto detection.",
            ),
            DeclareLaunchArgument(
                "point_cloud_topic",
                default_value="",
                description="PointCloud2 override; empty uses profile/auto.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the /clock topic from simulation or rosbag.",
            ),
            # Keep motor control disabled until a real hardware plugin is ready.
            DeclareLaunchArgument(
                "use_control",
                default_value="false",
                description="Start configured ros2_control nodes.",
            ),
            DeclareLaunchArgument(
                "slam_enabled",
                default_value="true",
                description="Start online SLAM Toolbox.",
            ),
            DeclareLaunchArgument(
                "nav2_enabled",
                default_value="true",
                description="Start the complete Nav2 runtime.",
            ),
            DeclareLaunchArgument(
                "nav2_autostart",
                default_value="true",
                description="Activate Nav2 after sensors and TF are ready.",
            ),
            DeclareLaunchArgument(
                "vision",
                default_value="true",
                description="Start the OpenCV obstacle detector.",
            ),
            DeclareLaunchArgument(
                "competition",
                default_value="false",
                description="Use the Robocon competition state machine.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the project display config.",
            ),
            DeclareLaunchArgument(
                "nav2_log_level",
                default_value="info",
                description="Log level passed to all Nav2 nodes.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=package_file("slam", "config", "slam.yaml"),
                description="SLAM Toolbox parameter YAML.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=package_file("slam", "config", "nav2.yaml"),
                description="Nav2 parameter YAML.",
            ),
            DeclareLaunchArgument(
                "vision_params_file",
                default_value=package_file(
                    "quadruped_perception", "config", "vision.yaml"
                ),
                description="OpenCV detector parameter YAML.",
            ),
            DeclareLaunchArgument(
                "terrain_params_file",
                default_value=package_file(
                    "quadruped_perception", "config", "terrain.yaml"
                ),
                description="Point-cloud terrain parameter YAML.",
            ),
            DeclareLaunchArgument(
                "crossing_params_file",
                default_value=package_file(
                    "quadruped_planning", "config", "crossing.yaml"
                ),
                description="Crossing fusion and velocity-gate YAML.",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=package_file("slam", "rviz", "slam.rviz"),
                description="RViz display configuration.",
            ),
            OpaqueFunction(function=_launch_complete_stack),
        ]
    )

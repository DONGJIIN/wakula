"""Select a common sensor profile, then start the canonical Wakula stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare

from slam.sensor_profiles import load_sensor_profiles, resolve_sensor_topics


def package_file(package: str, folder: str, filename: str):
    """Build a package share path."""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def _launch_stack(context):
    """Resolve strings and scope remaps to the included stack."""
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

    # SLAM/Nav2 continue to refer to /scan and /odom. Scoped remaps adapt only
    # their inputs; image/point-cloud nodes use their existing parameters.
    full_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file("slam", "launch", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_control": LaunchConfiguration("use_control"),
            "rviz": LaunchConfiguration("rviz"),
            "vision": LaunchConfiguration("vision"),
            "camera_topic": topics["camera_topic"],
            "point_cloud_topic": topics["point_cloud_topic"],
            "competition": LaunchConfiguration("competition"),
            "nav2_autostart": LaunchConfiguration("nav2_autostart"),
        }.items(),
    )
    summary = (
        f"sensor_profile={profile_name}: scan={topics['scan_topic']}, "
        f"odom={topics['odom_topic']}, "
        f"image={topics['camera_topic'] or 'auto'}, "
        f"points={topics['point_cloud_topic'] or 'auto'}"
    )
    return [
        LogInfo(msg=summary),
        GroupAction(
            actions=[
                SetRemap(src="/scan", dst=topics["scan_topic"]),
                SetRemap(src="/odom", dst=topics["odom_topic"]),
                full_stack,
            ]
        ),
    ]


def generate_launch_description():
    """Expose one profile plus explicit overrides as the hardware boundary."""
    profiles_file = package_file("slam", "config", "sensor_profiles.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_profile",
                default_value="ros_default",
                description="Profile name from sensor_profiles_file.",
            ),
            DeclareLaunchArgument(
                "sensor_profiles_file",
                default_value=profiles_file,
                description=(
                    "Replaceable YAML file containing sensor topic profiles."
                ),
            ),
            DeclareLaunchArgument("scan_topic", default_value=""),
            DeclareLaunchArgument("odom_topic", default_value=""),
            DeclareLaunchArgument("camera_topic", default_value=""),
            DeclareLaunchArgument("point_cloud_topic", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_control", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("vision", default_value="true"),
            DeclareLaunchArgument("competition", default_value="false"),
            DeclareLaunchArgument("nav2_autostart", default_value="true"),
            OpaqueFunction(function=_launch_stack),
        ]
    )

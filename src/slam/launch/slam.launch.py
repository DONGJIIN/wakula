"""一键启动 Wakula 的 SLAM、Nav2、OpenCV 与点云感知栈。"""

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
    """返回安装空间内的资源路径，避免写死本机目录。"""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def _launch_complete_stack(context):
    """解析传感器 profile 后创建硬件无关的完整导航栈。"""
    profile_name = LaunchConfiguration("sensor_profile").perform(context)
    profiles_file = LaunchConfiguration("sensor_profiles_file").perform(context)
    topics = resolve_sensor_topics(
        load_sensor_profiles(profiles_file),
        profile_name,
        {
            key: LaunchConfiguration(key).perform(context)
            for key in ("scan_topic", "odom_topic", "camera_topic", "point_cloud_topic")
        },
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file("quadruped_bringup", "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "vision": LaunchConfiguration("vision"),
            "robot_model": LaunchConfiguration("robot_model"),
            "camera_topic": topics["camera_topic"],
            "point_cloud_topic": topics["point_cloud_topic"],
            "terrain_params_file": LaunchConfiguration("terrain_params_file"),
            "vision_params_file": LaunchConfiguration("vision_params_file"),
            "terrain_navigation_params_file": LaunchConfiguration(
                "terrain_navigation_params_file"
            ),
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
        condition=IfCondition(LaunchConfiguration("slam_enabled")),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package_file("slam", "launch", "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": LaunchConfiguration("nav2_params_file"),
            "autostart": LaunchConfiguration("nav2_autostart"),
            "log_level": LaunchConfiguration("nav2_log_level"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("nav2_enabled")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    summary = (
        f"Wakula profile={profile_name}: scan={topics['scan_topic']}, "
        f"odom={topics['odom_topic']}, image={topics['camera_topic'] or 'auto'}, "
        f"points={topics['point_cloud_topic'] or 'auto'}"
    )
    return [
        LogInfo(msg=summary),
        GroupAction(
            actions=[
                # 一个 remap 同时覆盖 SLAM、Nav2 和健康检查，换设备时只改入口参数。
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
    """声明公共入口参数；本文件不启动仿真、底盘或关节控制。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument("sensor_profile", default_value="ros_default"),
            DeclareLaunchArgument(
                "sensor_profiles_file",
                default_value=package_file("slam", "config", "sensor_profiles.yaml"),
            ),
            DeclareLaunchArgument("scan_topic", default_value=""),
            DeclareLaunchArgument("odom_topic", default_value=""),
            DeclareLaunchArgument("camera_topic", default_value=""),
            DeclareLaunchArgument("point_cloud_topic", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("slam_enabled", default_value="true"),
            DeclareLaunchArgument("nav2_enabled", default_value="true"),
            DeclareLaunchArgument("nav2_autostart", default_value="true"),
            DeclareLaunchArgument("vision", default_value="true"),
            DeclareLaunchArgument("robot_model", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("nav2_log_level", default_value="info"),
            DeclareLaunchArgument(
                "slam_params_file", default_value=package_file("slam", "config", "slam.yaml")
            ),
            DeclareLaunchArgument(
                "nav2_params_file", default_value=package_file("slam", "config", "nav2.yaml")
            ),
            DeclareLaunchArgument(
                "vision_params_file",
                default_value=package_file("quadruped_perception", "config", "vision.yaml"),
            ),
            DeclareLaunchArgument(
                "terrain_params_file",
                default_value=package_file("quadruped_perception", "config", "terrain.yaml"),
            ),
            DeclareLaunchArgument(
                "terrain_navigation_params_file",
                default_value=package_file(
                    "quadruped_planning", "config", "terrain_navigation.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config_file", default_value=package_file("slam", "rviz", "slam.rviz")
            ),
            OpaqueFunction(function=_launch_complete_stack),
        ]
    )

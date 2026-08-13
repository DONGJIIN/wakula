"""兼容入口：核心功能现已直接归入 slam.launch.py，本文件仅转发参数。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_file(package, folder, filename):
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    core_arguments = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "robot_model": LaunchConfiguration("robot_model"),
        "rviz": LaunchConfiguration("rviz"),
        "sensor_profile": LaunchConfiguration("sensor_profile"),
        "camera_topic": LaunchConfiguration("camera_topic"),
        "point_cloud_topic": LaunchConfiguration("point_cloud_topic"),
        "autonomy": "true",
        "autonomy_autostart": LaunchConfiguration("autostart_mission"),
        "mission_params_file": LaunchConfiguration("mission_params_file"),
    }
    return LaunchDescription([
        # 任务节点需要真正的 bool 参数；真机默认系统时钟，仿真包装入口显式传 true。
        # slam.launch.py 自身支持 auto，但不能把字符串 "auto" 直接强制转换成 bool。
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("robot_model", default_value="auto"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("sensor_profile", default_value="ros_default"),
        DeclareLaunchArgument("camera_topic", default_value=""),
        DeclareLaunchArgument("point_cloud_topic", default_value=""),
        DeclareLaunchArgument("autostart_mission", default_value="false"),
        DeclareLaunchArgument(
            "mission_params_file",
            default_value=package_file("quadruped_planning", "config", "autonomous_mission.yaml"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("slam", "launch", "slam.launch.py")),
            launch_arguments=core_arguments.items(),
        ),
    ])

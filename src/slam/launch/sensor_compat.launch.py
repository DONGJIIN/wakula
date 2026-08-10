"""旧传感器启动入口的兼容别名，所有参数转发给统一的 slam.launch.py。

保留该文件只为避免队员旧命令立即失效；新功能不得在此重复实现，否则两个入口会产生
不同的节点图和参数行为。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_file(package: str, folder: str, filename: str):
    """构造安装空间中的 package share 路径。"""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    """声明旧入口参数，并保持名称不变地转发到统一入口。"""
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
    descriptions = {
        "sensor_profile": "兼容入口：传感器话题预设名称",
        "sensor_profiles_file": "兼容入口：传感器 profile YAML",
        "scan_topic": "兼容入口：显式 LaserScan 话题覆盖",
        "odom_topic": "兼容入口：显式 Odometry 话题覆盖",
        "camera_topic": "兼容入口：显式 Image 话题覆盖",
        "point_cloud_topic": "兼容入口：显式 PointCloud2 话题覆盖",
        "use_sim_time": "兼容入口：是否使用 /clock",
        "rviz": "兼容入口：是否启动 RViz",
        "vision": "兼容入口：是否启动 OpenCV 融合",
        "nav2_autostart": "兼容入口：输入就绪后是否激活 Nav2",
    }
    declarations = [
        DeclareLaunchArgument(
            name, default_value=defaults[name], description=descriptions[name]
        )
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

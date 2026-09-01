"""仿真专用的 Wakula 算法启动入口。

Gazebo 场地仍由 ``quadruped_gazebo/robocon_field.launch.py`` 独立启动；本文件只包装
核心 ``slam.launch.py``，固定两项最容易漏写、却会直接破坏时间和 TF 的参数：

* ``use_sim_time=true``：SLAM、Nav2、OpenCV/点云节点统一使用 Gazebo 的 ``/clock``；
* ``robot_model=false``：不再发布仓库占位 URDF 的传感器 TF，避免与 Gazebo 测试狗重复。

真机仍使用 ``slam.launch.py``，因此这个便捷入口不会改变硬件侧默认行为，也不会让核心
算法反向依赖仿真包。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _core_launch_file():
    """返回已安装核心入口，避免依赖源码工作区的绝对路径。"""
    return PathJoinSubstitution([FindPackageShare("slam"), "launch", "slam.launch.py"])


def _package_file(package, folder, filename):
    """构造安装空间资源路径，使仿真覆盖参数与核心入口使用同一默认文件。"""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    """启动使用仿真时间和仿真 TF 的 SLAM/Nav2/感知栈，但不启动 Gazebo。"""
    forwarded = {
        # 以下参数仍允许测试人员关闭重模块或显式绑定非默认仿真话题。
        "sensor_profile": LaunchConfiguration("sensor_profile"),
        "sensor_profiles_file": LaunchConfiguration("sensor_profiles_file"),
        "scan_topic": LaunchConfiguration("scan_topic"),
        "odom_topic": LaunchConfiguration("odom_topic"),
        "camera_topic": LaunchConfiguration("camera_topic"),
        "point_cloud_topic": LaunchConfiguration("point_cloud_topic"),
        "slam_enabled": LaunchConfiguration("slam_enabled"),
        "nav2_enabled": LaunchConfiguration("nav2_enabled"),
        "nav2_autostart": LaunchConfiguration("nav2_autostart"),
        "vision": LaunchConfiguration("vision"),
        "speed_gate": LaunchConfiguration("speed_gate"),
        "rviz": LaunchConfiguration("rviz"),
        "nav2_log_level": LaunchConfiguration("nav2_log_level"),
        "slam_params_file": LaunchConfiguration("slam_params_file"),
        "nav2_params_file": LaunchConfiguration("nav2_params_file"),
        "vision_params_file": LaunchConfiguration("vision_params_file"),
        "terrain_params_file": LaunchConfiguration("terrain_params_file"),
        "terrain_navigation_params_file": LaunchConfiguration(
            "terrain_navigation_params_file"
        ),
        "rviz_config_file": LaunchConfiguration("rviz_config_file"),
        # 仿真入口刻意不公开这两个值，防止再次形成双时钟或重复 TF。
        "use_sim_time": "true",
        "robot_model": "false",
    }
    declarations = [
        DeclareLaunchArgument("sensor_profile", default_value="ros_default"),
        DeclareLaunchArgument(
            "sensor_profiles_file",
            default_value=_package_file("slam", "config", "sensor_profiles.yaml"),
        ),
        DeclareLaunchArgument("scan_topic", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value=""),
        DeclareLaunchArgument("camera_topic", default_value=""),
        DeclareLaunchArgument("point_cloud_topic", default_value=""),
        DeclareLaunchArgument("slam_enabled", default_value="true"),
        DeclareLaunchArgument("nav2_enabled", default_value="true"),
        DeclareLaunchArgument("nav2_autostart", default_value="true"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("speed_gate", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("nav2_log_level", default_value="info"),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=_package_file("slam", "config", "slam.yaml"),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=_package_file("slam", "config", "nav2.yaml"),
        ),
        DeclareLaunchArgument(
            "vision_params_file",
            default_value=_package_file(
                "quadruped_perception", "config", "vision.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "terrain_params_file",
            default_value=_package_file(
                "quadruped_perception", "config", "terrain.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "terrain_navigation_params_file",
            default_value=_package_file(
                "quadruped_planning", "config", "terrain_navigation.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=_package_file("slam", "rviz", "slam.rviz"),
        ),
    ]
    return LaunchDescription(
        [
            *declarations,
            LogInfo(
                msg=(
                    "Wakula simulation mode: use_sim_time=true, robot_model=false; "
                    "Gazebo must be running in another terminal"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_core_launch_file()),
                launch_arguments=forwarded.items(),
            ),
        ]
    )

"""一键启动核心 SLAM/Nav2/OpenCV 与自主探索任务（不包含 Gazebo/运动控制器）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_file(package, folder, filename):
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    core_arguments = {
        "use_sim_time": use_sim_time,
        "robot_model": LaunchConfiguration("robot_model"),
        "rviz": LaunchConfiguration("rviz"),
        "sensor_profile": LaunchConfiguration("sensor_profile"),
        "camera_topic": LaunchConfiguration("camera_topic"),
        "point_cloud_topic": LaunchConfiguration("point_cloud_topic"),
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
        Node(
            package="quadruped_planning",
            executable="autonomous_mission",
            name="autonomous_mission",
            output="screen",
            parameters=[
                LaunchConfiguration("mission_params_file"),
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "autostart": ParameterValue(
                        LaunchConfiguration("autostart_mission"), value_type=bool
                    ),
                },
            ],
        ),
    ])

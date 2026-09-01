"""单独启动自主探索与越障任务；不启动或 include 核心 SLAM 和仿真环境。

这是 ``slam`` 包内的可选功能入口。运行前必须已有 ``slam.launch.py``（或等价真机
SLAM/Nav2/OpenCV 数据链）；关闭本 launch 只停止自主任务，不影响地图、导航节点和传感器。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from slam.runtime_detection import clock_publisher_is_available


def package_file(package, folder, filename):
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def _clock_is_available() -> bool:
    """复用核心入口的有界发布者计数探测，避免订阅者造成仿真误判。"""
    return clock_publisher_is_available()


def _launch_mission(context):
    requested = LaunchConfiguration("use_sim_time").perform(context).lower()
    if requested not in {"auto", "true", "false"}:
        raise RuntimeError("use_sim_time must be auto, true or false")
    use_sim_time = _clock_is_available() if requested == "auto" else requested == "true"
    return [
        Node(
            package="quadruped_planning",
            executable="autonomous_mission",
            name="autonomous_mission",
            output="screen",
            parameters=[
                LaunchConfiguration("mission_params_file"),
                {"use_sim_time": use_sim_time, "autostart": True},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="auto",
            description="auto 检测 /clock 发布者；仿真用 true，真机用 false",
        ),
        DeclareLaunchArgument(
            "mission_params_file",
            default_value=package_file(
                "quadruped_planning", "config", "autonomous_mission.yaml"
            ),
        ),
        OpaqueFunction(function=_launch_mission),
    ])

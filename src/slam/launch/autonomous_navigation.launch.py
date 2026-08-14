"""单独启动自主探索与越障任务；不启动或 include 核心 SLAM 和仿真环境。

这是 ``slam`` 包内的可选功能入口。运行前必须已有 ``slam.launch.py``（或等价真机
SLAM/Nav2/OpenCV 数据链）；关闭本 launch 只停止自主任务，不影响地图、导航节点和传感器。
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_file(package, folder, filename):
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def _clock_is_available() -> bool:
    """短窗口自动识别仿真时钟；第三个命令不再固定等待 2 秒。"""
    for _attempt in range(2):
        try:
            result = subprocess.run(
                ["ros2", "topic", "list", "--no-daemon", "--spin-time", "0.50"],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.2,
            )
        except (OSError, subprocess.TimeoutExpired):
            result = None
        if result is not None:
            if "/clock" in result.stdout.splitlines():
                return True
    return False


def _launch_mission(context):
    requested = LaunchConfiguration("use_sim_time").perform(context).lower()
    if requested not in {"auto", "true", "false"}:
        raise RuntimeError("use_sim_time must be auto, true or false")
    use_sim_time = _clock_is_available() if requested == "auto" else requested == "true"
    requested_backend = LaunchConfiguration("simulation_traversal_backend").perform(
        context
    ).lower()
    if requested_backend not in {"auto", "true", "false"}:
        raise RuntimeError(
            "simulation_traversal_backend must be auto, true or false"
        )
    start_sim_backend = (
        use_sim_time if requested_backend == "auto" else requested_backend == "true"
    )
    actions = [
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
    if start_sim_backend:
        # 这是无腿部动力学测试狗的可替换 Action 后端，不读取 world 坐标，也不进入
        # slam.launch.py。真机 use_sim_time=false 时绝不会启动，由真实运动控制器提供
        # 完全相同的 /traverse_obstacle Action。
        actions.insert(
            0,
            Node(
                package="quadruped_gazebo",
                executable="sim_traverse_obstacle",
                name="sim_traverse_obstacle",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="auto",
            description="auto detects /clock; true for simulation, false for hardware",
        ),
        DeclareLaunchArgument(
            "simulation_traversal_backend",
            default_value="auto",
            description=(
                "auto starts the generic TraverseObstacle test backend only when "
                "simulation time is detected; set false for a real controller"
            ),
        ),
        DeclareLaunchArgument(
            "mission_params_file",
            default_value=package_file(
                "quadruped_planning", "config", "autonomous_mission.yaml"
            ),
        ),
        OpaqueFunction(function=_launch_mission),
    ])

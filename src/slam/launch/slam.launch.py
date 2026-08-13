"""一键启动 Wakula 的 SLAM、Nav2、OpenCV 与点云感知栈。

本文件只负责组合模块和统一 remap，不复制子节点参数。启动顺序由 ROS 2 launch 管理，
Nav2 是否真正激活则由 readiness monitor 根据 /scan、/odom 和 TF 决定。
"""

import re
import subprocess
import time

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


def _robocon_simulation_is_running() -> bool:
    """重复查询当前 ROS 域的 /clock 发布者，避免 DDS 冷启动时漏判 Gazebo。

    ROS 2 CLI 每次使用 ``--no-daemon`` 都会创建临时 DDS participant。Gazebo 刚启动或同机
    负载较高时，1 秒内可能尚未发现桥接器；单次查询会把仿真误判为真机，造成传感器时间戳
    与算法时钟完全不一致。这里最多重试 4 次，只要任一次看到真实发布者就选择仿真时间。
    显式传入 true/false 时完全跳过探测。
    """
    for attempt in range(4):
        try:
            result = subprocess.run(
                [
                    "ros2",
                    "topic",
                    "info",
                    "/clock",
                    "--no-daemon",
                    "--spin-time",
                    "2.0",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.8,
            )
        except (OSError, subprocess.TimeoutExpired):
            result = None
        if result is not None:
            match = re.search(r"Publisher count:\s*(\d+)", result.stdout)
            if match and int(match.group(1)) > 0:
                return True
        if attempt < 3:
            time.sleep(0.15)
    return False


def _resolve_runtime_mode(context) -> bool:
    """把 auto 时间/模型参数解析成 ROS 节点可接受的 true/false 字符串。"""
    requested_time = LaunchConfiguration("use_sim_time").perform(context).lower()
    requested_model = LaunchConfiguration("robot_model").perform(context).lower()
    valid = {"auto", "true", "false"}
    if requested_time not in valid or requested_model not in valid:
        raise RuntimeError("use_sim_time and robot_model must be auto, true or false")

    detected = False
    if "auto" in (requested_time, requested_model):
        detected = _robocon_simulation_is_running()
    if requested_time == "auto":
        context.launch_configurations["use_sim_time"] = "true" if detected else "false"
    if requested_model == "auto":
        context.launch_configurations["robot_model"] = "false" if detected else "true"
    return detected


def _launch_complete_stack(context):
    """解析传感器 profile 后创建硬件无关的完整导航栈。"""
    simulation_detected = _resolve_runtime_mode(context)
    # OpaqueFunction 让我们在运行期取得字符串值，再执行 YAML profile 校验与覆盖合并。
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

    # 三个 Include 的职责互不重叠：感知安全链、SLAM Toolbox、Nav2 在线节点组。
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
        # Snap 版 VS Code 会向集成终端注入 GTK_PATH=/snap/code/...；RViz 的 Qt/GTK
        # 插件随后可能错误加载 core20 的 libpthread，并以 GLIBC_PRIVATE 报错退出。
        # 只清空 RViz 子进程的 GTK 模块搜索路径，不改 ROS/Gazebo 或用户系统环境。
        additional_env={"GTK_PATH": "", "GTK_EXE_PREFIX": "", "GIO_MODULE_DIR": ""},
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    # 把时间源和模型 TF 选择直接打印在启动终端中。漏写仿真参数时，使用者无需翻查
    # launch 临时参数文件即可立即发现 use_sim_time=false / robot_model=true。
    summary = (
        f"Wakula profile={profile_name}, "
        f"simulation_detected={str(simulation_detected).lower()}, "
        f"use_sim_time={LaunchConfiguration('use_sim_time').perform(context)}, "
        f"robot_model={LaunchConfiguration('robot_model').perform(context)}: "
        f"scan={topics['scan_topic']}, "
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
    # 空 camera/point_cloud 参数表示由感知节点从常见候选话题中自动锁定一个来源；
    # scan/odom 始终必须由 profile 或显式参数解析为非空标准接口。
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_profile",
                default_value="ros_default",
                description="sensor_profiles.yaml 中的话题预设名称",
            ),
            DeclareLaunchArgument(
                "sensor_profiles_file",
                default_value=package_file("slam", "config", "sensor_profiles.yaml"),
                description="传感器话题 profile YAML 的路径",
            ),
            DeclareLaunchArgument(
                "scan_topic", default_value="", description="非空时覆盖 profile 的 LaserScan 话题"
            ),
            DeclareLaunchArgument(
                "odom_topic", default_value="", description="非空时覆盖 profile 的 Odometry 话题"
            ),
            DeclareLaunchArgument(
                "camera_topic", default_value="", description="非空时固定 Image 输入；空值自动选源"
            ),
            DeclareLaunchArgument(
                "point_cloud_topic",
                default_value="",
                description="非空时固定 PointCloud2 输入；空值自动选源",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="auto",
                description="auto 自动检测 /clock；也可显式设为 true/false",
            ),
            DeclareLaunchArgument(
                "slam_enabled", default_value="true", description="是否启动在线 SLAM Toolbox"
            ),
            DeclareLaunchArgument(
                "nav2_enabled", default_value="true", description="是否启动 Nav2 节点组和健康监控"
            ),
            DeclareLaunchArgument(
                "nav2_autostart",
                default_value="true",
                description="输入就绪后是否由 readiness monitor 自动激活 Nav2",
            ),
            DeclareLaunchArgument(
                "vision", default_value="true", description="是否启动 OpenCV 与相机/点云融合节点"
            ),
            DeclareLaunchArgument(
                "robot_model",
                default_value="auto",
                description="auto 在本项目 Gazebo 中关闭占位 TF，真机默认开启",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="是否启动 RViz；无显示器部署应关闭"
            ),
            DeclareLaunchArgument(
                "nav2_log_level", default_value="info", description="Nav2 节点的 ROS 日志等级"
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=package_file("slam", "config", "slam.yaml"),
                description="SLAM Toolbox 参数 YAML",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=package_file("slam", "config", "nav2.yaml"),
                description="Nav2 与导航监控参数 YAML",
            ),
            DeclareLaunchArgument(
                "vision_params_file",
                default_value=package_file("quadruped_perception", "config", "vision.yaml"),
                description="OpenCV 障碍检测参数 YAML",
            ),
            DeclareLaunchArgument(
                "terrain_params_file",
                default_value=package_file("quadruped_perception", "config", "terrain.yaml"),
                description="点云地形分析参数 YAML",
            ),
            DeclareLaunchArgument(
                "terrain_navigation_params_file",
                default_value=package_file(
                    "quadruped_planning", "config", "terrain_navigation.yaml"
                ),
                description="地形安全决策和速度门参数 YAML",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=package_file("slam", "rviz", "slam.rviz"),
                description="RViz 显示配置路径",
            ),
            OpaqueFunction(function=_launch_complete_stack),
        ]
    )

"""启动硬件无关的感知、融合与导航安全辅助节点。

本文件是 ``slam.launch.py`` 的子入口：它不启动 SLAM Toolbox 或 Nav2，也不包含底盘、
关节和越障控制。单独运行时适合只调试相机、点云及安全决策链。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_file(package: str, folder: str, filename: str):
    """构造安装空间中的资源路径，兼容源码和 install 工作空间。"""
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    """声明感知、地形安全评估和 Nav2 速度约束的公共启动入口。"""
    use_sim_time = LaunchConfiguration("use_sim_time")
    vision = LaunchConfiguration("vision")
    robot_model = LaunchConfiguration("robot_model")
    camera_topic = LaunchConfiguration("camera_topic")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    terrain_params_file = LaunchConfiguration("terrain_params_file")
    vision_params_file = LaunchConfiguration("vision_params_file")
    terrain_navigation_params_file = LaunchConfiguration(
        "terrain_navigation_params_file"
    )

    description_file = package_file(
        "quadruped_description", "urdf", "quadruped.urdf.xacro"
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file]),
        value_type=str,
    )
    common_time = {"use_sim_time": use_sim_time}

    # 数据流顺序：点云/图像 -> 感知 -> 融合 -> 地形决策 -> Nav2 速度门。
    # robot_state_publisher 仅提供占位传感器 TF，可由真机模型整体替换。
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="使用 /clock 作为 ROS 时间"
            ),
            DeclareLaunchArgument(
                "vision", default_value="true", description="启动 OpenCV 与视觉/点云融合节点"
            ),
            DeclareLaunchArgument(
                "robot_model", default_value="true", description="发布仓库内占位 URDF 和固定 TF"
            ),
            DeclareLaunchArgument(
                "camera_topic", default_value="", description="指定 Image 话题；空值表示自动选源"
            ),
            DeclareLaunchArgument(
                "point_cloud_topic",
                default_value="",
                description="指定 PointCloud2 话题；空值表示自动选源",
            ),
            DeclareLaunchArgument(
                "terrain_params_file",
                default_value=package_file(
                    "quadruped_perception", "config", "terrain.yaml"
                ),
                description="点云地形分析参数 YAML",
            ),
            DeclareLaunchArgument(
                "vision_params_file",
                default_value=package_file(
                    "quadruped_perception", "config", "vision.yaml"
                ),
                description="OpenCV 障碍检测参数 YAML",
            ),
            DeclareLaunchArgument(
                "terrain_navigation_params_file",
                default_value=package_file(
                    "quadruped_planning", "config", "terrain_navigation.yaml"
                ),
                description="地形决策和速度门参数 YAML",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}, common_time],
                condition=IfCondition(robot_model),
            ),
            Node(
                package="quadruped_perception",
                executable="terrain_analyzer",
                output="screen",
                parameters=[
                    terrain_params_file,
                    {"input_topic": point_cloud_topic},
                    common_time,
                ],
            ),
            Node(
                package="quadruped_perception",
                executable="vision_obstacle_detector",
                output="screen",
                parameters=[
                    vision_params_file,
                    {"image_topic": camera_topic},
                    common_time,
                ],
                condition=IfCondition(vision),
            ),
            Node(
                package="quadruped_perception",
                executable="perception_fusion",
                output="screen",
                parameters=[vision_params_file, common_time],
                condition=IfCondition(vision),
            ),
            Node(
                package="quadruped_planning",
                executable="terrain_safety_assessor",
                output="screen",
                parameters=[
                    terrain_navigation_params_file,
                    {
                        "prefer_fused_obstacle": ParameterValue(
                            vision, value_type=bool
                        )
                    },
                    common_time,
                ],
            ),
            # 将已确认比赛障碍转换成“接近入口—对正—等待接管”状态。该节点只发布
            # 相对入口位姿和 READY 信号，不调用 Nav2 Action，也不产生腿部控制命令。
            Node(
                package="quadruped_planning",
                executable="traversal_guidance",
                output="screen",
                parameters=[terrain_navigation_params_file, common_time],
            ),
            Node(
                package="quadruped_planning",
                executable="navigation_speed_gate",
                output="screen",
                parameters=[terrain_navigation_params_file, common_time],
            ),
        ]
    )

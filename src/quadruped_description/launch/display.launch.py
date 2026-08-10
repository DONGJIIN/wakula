"""在 RViz 中显示未标定外形和传感器占位 TF，不启动任何控制器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """声明模型/RViz 调试入口；关节状态必须由外部真实系统提供。"""
    package_share = FindPackageShare("quadruped_description")
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
    )

    # 不启动 joint_state_publisher_gui：它曾在当前 Ubuntu/Qt 组合上崩溃，而且占位模型的
    # 关节角不应被误认为真实反馈。真机显示时由驱动发布 /joint_states。
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=PathJoinSubstitution(
                    [package_share, "urdf", "quadruped.urdf.xacro"]
                ),
                description="四足机器人 Xacro 模型的绝对路径",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "quadruped.rviz"]
                ),
                description="RViz 配置文件的绝对路径",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )

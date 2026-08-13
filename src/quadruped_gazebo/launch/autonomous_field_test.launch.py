"""独立启动比赛场地与仿真越障适配器；不装载任何 SLAM/导航算法。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_file(package, folder, filename):
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "start_gazebo",
            default_value="true",
            description="false 时复用当前 ROS 域中已经运行的 Gazebo 场地",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                package_file("quadruped_gazebo", "launch", "robocon_field.launch.py")
            ),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
            condition=IfCondition(LaunchConfiguration("start_gazebo")),
        ),
        # 仿真替身只验证 Action 合同，属于 Gazebo 测试设施而非算法。SLAM 需在另一个
        # 终端启动；真机则必须由运动控制团队提供真实 /traverse_obstacle 服务端。
        Node(
            package="quadruped_gazebo",
            executable="sim_traverse_obstacle",
            name="sim_traverse_obstacle",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])

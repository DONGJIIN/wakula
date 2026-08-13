"""一键启动独立比赛场地、核心算法、自主探索和仿真越障适配器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_file(package, folder, filename):
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("autostart_mission", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                package_file("quadruped_gazebo", "launch", "robocon_field.launch.py")
            ),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                package_file("slam", "launch", "autonomous_navigation.launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "robot_model": "false",
                "rviz": LaunchConfiguration("rviz"),
                "autostart_mission": LaunchConfiguration("autostart_mission"),
            }.items(),
        ),
        # 仿真替身只验证 Action 编排。真机启动 autonomous_navigation.launch.py 时不会
        # 自动包含它，必须由真实运动控制团队提供 /traverse_obstacle。
        Node(
            package="quadruped_gazebo",
            executable="sim_traverse_obstacle",
            name="sim_traverse_obstacle",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])

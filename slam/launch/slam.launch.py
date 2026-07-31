from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = PathJoinSubstitution(
        [FindPackageShare("slam"), "config", "slam.yaml"]
    )
    nav2_params = PathJoinSubstitution(
        [FindPackageShare("slam"), "config", "nav2.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam"), "rviz", "slam.rviz"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params_file": slam_params,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
            }.items(),
        ),
        Node(
            package="slam",
            executable="navigation_node",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "base_frame": "base_link",
                "obstacle_topic": "/terrain/obstacle",
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])

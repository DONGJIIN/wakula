"""Bring up the simulation-ready SLAM + Nav2 + terrain-crossing skeleton.

Required external topics/transforms:
  /scan (sensor_msgs/LaserScan), /odom (nav_msgs/Odometry), and
  /camera/depth/color/points (sensor_msgs/PointCloud2). A hardware driver must
  provide odom -> base_link and replace the mock ros2_control system.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    competition = LaunchConfiguration("competition")
    model = PathJoinSubstitution(
        [FindPackageShare("quadruped_description"), "urdf", "quadruped.urdf.xacro"]
    )
    slam_params = PathJoinSubstitution([FindPackageShare("slam"), "config", "slam.yaml"])
    nav2_params = PathJoinSubstitution([FindPackageShare("slam"), "config", "nav2.yaml"])
    terrain_params = PathJoinSubstitution(
        [FindPackageShare("quadruped_perception"), "config", "terrain.yaml"]
    )
    crossing_params = PathJoinSubstitution(
        [FindPackageShare("quadruped_planning"), "config", "crossing.yaml"]
    )
    competition_params = PathJoinSubstitution(
        [FindPackageShare("quadruped_planning"), "config", "competition.yaml"]
    )
    waypoint_params = PathJoinSubstitution(
        [FindPackageShare("quadruped_planning"), "config", "course_waypoints.yaml"]
    )
    rviz_config = PathJoinSubstitution([FindPackageShare("slam"), "rviz", "slam.rviz"])
    robot_description = ParameterValue(Command([FindExecutable(name="xacro"), " ", model]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "competition",
                default_value="false",
                description="Use Robocon obstacle-course scoring/state machine.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": slam_params,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "autostart": "True",
                }.items(),
            ),
            Node(
                package="quadruped_perception",
                executable="terrain_analyzer",
                parameters=[terrain_params, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="quadruped_planning",
                executable="obstacle_crossing_manager",
                parameters=[crossing_params, {"use_sim_time": use_sim_time}],
                condition=UnlessCondition(competition),
                output="screen",
            ),
            Node(
                package="quadruped_planning",
                executable="cmd_vel_gate",
                parameters=[crossing_params, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="quadruped_planning",
                executable="competition_obstacle_manager",
                parameters=[competition_params, {"use_sim_time": use_sim_time}],
                condition=IfCondition(competition),
                output="screen",
            ),
            Node(
                package="quadruped_planning",
                executable="course_waypoint_navigator",
                parameters=[waypoint_params, {"use_sim_time": use_sim_time}],
                condition=IfCondition(competition),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(rviz),
                output="screen",
            ),
        ]
    )

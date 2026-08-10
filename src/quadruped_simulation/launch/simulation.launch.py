"""Launch Gazebo Harmonic, spawn Wakula and bridge standard ROS interfaces."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_file(package: str, folder: str, filename: str):
    return PathJoinSubstitution([FindPackageShare(package), folder, filename])


def generate_launch_description():
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    bridge_file = LaunchConfiguration("bridge_file")
    robot_name = LaunchConfiguration("robot_name")
    start_description = LaunchConfiguration("start_robot_state_publisher")
    description_file = package_file(
        "quadruped_description", "urdf", "quadruped.urdf.xacro"
    )
    controllers_file = package_file(
        "quadruped_control", "config", "controllers.yaml"
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"), " ", description_file,
                " simulation:=true controllers_file:=", controllers_file,
            ]
        ),
        value_type=str,
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file("ros_gz_sim", "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world, " -r -v 2"]}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file("ros_gz_sim", "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world, " -r -s -v 2"]}.items(),
        condition=IfCondition(headless),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", "wakula_training",
            "-topic", "/robot_description",
            "-name", robot_name,
            "-allow_renaming", "false",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
    )
    spawn_controllers = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "leg_controller",
            "--activate-as-group",
            "--controller-manager-timeout", "30",
        ],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wakula_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_file, "use_sim_time": True}],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        condition=IfCondition(start_description),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=package_file(
                    "quadruped_simulation", "worlds", "wakula_training.sdf"
                ),
            ),
            DeclareLaunchArgument(
                "bridge_file",
                default_value=package_file(
                    "quadruped_simulation", "config", "bridge.yaml"
                ),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("robot_name", default_value="wakula_quadruped"),
            DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
            DeclareLaunchArgument("x", default_value="-2.5"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.02"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            gazebo_gui,
            gazebo_headless,
            bridge,
            robot_state_publisher,
            spawn,
            RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[spawn_controllers])
            ),
        ]
    )

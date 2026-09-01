"""启动独立 Gazebo 场地和“对准后传送到出口”的仿真越障替身。

本入口只组合 ``robocon_field.launch.py`` 与 Gazebo 包内的 TraverseObstacle Action
服务端，不启动 SLAM、Nav2、OpenCV 或自主任务。核心算法始终只看到标准
``/traverse_obstacle`` 接口；真机调试改用纯场地入口或完全不启动本文件。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Keep every simulation-only process under one Gazebo-owned launch."""
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    gui = LaunchConfiguration("gui")
    keyboard_teleop = LaunchConfiguration("keyboard_teleop")
    spawn_robot = LaunchConfiguration("spawn_test_robot")
    robot_sdf = LaunchConfiguration("robot_sdf")
    robot_name = LaunchConfiguration("robot_name")
    publish_test_sensor_tf = LaunchConfiguration("publish_test_sensor_tf")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_yaw = LaunchConfiguration("robot_yaw")
    benchmark_staging = LaunchConfiguration("benchmark_staging")

    field_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("quadruped_gazebo"),
                "launch",
                "robocon_field.launch.py",
            ])
        ),
        launch_arguments={
            "world": world,
            "world_name": world_name,
            "gui": gui,
            "keyboard_teleop": keyboard_teleop,
            "spawn_test_robot": spawn_robot,
            "robot_sdf": robot_sdf,
            "robot_name": robot_name,
            "publish_test_sensor_tf": publish_test_sensor_tf,
            "enable_point_cloud_bridge": LaunchConfiguration(
                "benchmark_raw_point_cloud"
            ),
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_yaw": robot_yaw,
        }.items(),
    )
    teleport_backend = Node(
        package="quadruped_gazebo",
        executable="sim_traverse_obstacle",
        name="sim_traverse_obstacle",
        output="screen",
        condition=IfCondition(spawn_robot),
        parameters=[{
            "use_sim_time": True,
            "model_name": robot_name,
            "maximum_alignment_error": ParameterValue(
                LaunchConfiguration("maximum_alignment_error"), value_type=float
            ),
            "benchmark_staging_enabled": ParameterValue(
                benchmark_staging, value_type=bool
            ),
            "benchmark_semantic_hint_enabled": ParameterValue(
                LaunchConfiguration("benchmark_semantic_hint"), value_type=bool
            ),
            # Read the same world layout that Gazebo starts.  Observation stations
            # therefore follow official coordinates when only the world is updated.
            "benchmark_world_path": world,
            "world_name": world_name,
        }],
    )

    package_share = FindPackageShare("quadruped_gazebo")
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                package_share, "worlds", "robocon_obstacle_field.sdf"
            ]),
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="robocon_obstacle_field",
            description="必须与 world 文件中的 <world name> 完全一致",
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("keyboard_teleop", default_value="true"),
        DeclareLaunchArgument("spawn_test_robot", default_value="true"),
        DeclareLaunchArgument(
            "robot_sdf",
            default_value=PathJoinSubstitution([
                package_share, "models", "generic_quadruped", "model.sdf"
            ]),
        ),
        DeclareLaunchArgument("robot_name", default_value="generic_quadruped"),
        DeclareLaunchArgument("publish_test_sensor_tf", default_value="true"),
        DeclareLaunchArgument(
            "benchmark_raw_point_cloud",
            default_value="false",
            description=(
                "Enable raw depth points during the timed workflow; leave false "
                "to avoid competing with its deterministic fused-sensor input"
            ),
        ),
        DeclareLaunchArgument("robot_x", default_value="-2.5"),
        DeclareLaunchArgument("robot_y", default_value="-0.2"),
        DeclareLaunchArgument("robot_yaw", default_value="3.141593"),
        DeclareLaunchArgument(
            "maximum_alignment_error",
            # Goal fields are one immutable snapshot, so there is no transport-time
            # heading drift to hide here.  Match the mission PREPARING envelope exactly;
            # a wider launch override would make the combined simulator less strict
            # than the standalone Action server and the future controller contract.
            default_value="0.22",
            description=(
                "Maximum absolute Action heading error accepted before teleport [rad]"
            ),
        ),
        DeclareLaunchArgument(
            "benchmark_staging",
            default_value="true",
            description=(
                "Gazebo-only: stage at the next pending obstacle after verified completion"
            ),
        ),
        DeclareLaunchArgument(
            "benchmark_semantic_hint",
            default_value="true",
            description=(
                "Gazebo-only standard perception-contract hint for the timed workflow benchmark"
            ),
        ),
        field_launch,
        teleport_backend,
    ])

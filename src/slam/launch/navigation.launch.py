"""Minimal Nav2 runtime without unused routing or docking processes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def nav2_node(package, executable, parameters, log_level, remappings, name=None):
    """Create one lifecycle node with consistent logging and TF remapping."""
    return Node(
        package=package,
        executable=executable,
        name=name or executable,
        output="screen",
        parameters=[parameters],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    cmd_remaps = tf_remaps + [("cmd_vel", "cmd_vel_nav")]
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
    ]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            # Rewrite every occurrence in nav2.yaml.  A GroupAction parameter
            # alone can lose to node-local YAML values and split the stack
            # between wall time and /clock.
            param_rewrites={
                "autostart": autostart,
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    nodes = [
        nav2_node(
            "nav2_controller",
            "controller_server",
            configured_params,
            log_level,
            cmd_remaps,
        ),
        nav2_node(
            "nav2_smoother",
            "smoother_server",
            configured_params,
            log_level,
            tf_remaps,
        ),
        nav2_node(
            "nav2_planner",
            "planner_server",
            configured_params,
            log_level,
            tf_remaps,
        ),
        nav2_node(
            "nav2_behaviors",
            "behavior_server",
            configured_params,
            log_level,
            cmd_remaps,
        ),
        nav2_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            configured_params,
            log_level,
            cmd_remaps,
        ),
        nav2_node(
            "nav2_collision_monitor",
            "collision_monitor",
            configured_params,
            log_level,
            tf_remaps,
        ),
        nav2_node(
            "nav2_bt_navigator",
            "bt_navigator",
            configured_params,
            log_level,
            tf_remaps,
        ),
        nav2_node(
            "nav2_waypoint_follower",
            "waypoint_follower",
            configured_params,
            log_level,
            tf_remaps,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                # Activation is requested by the readiness monitor only after
                # scan, odometry and localization TF are available.
                {"autostart": False},
                {"node_names": lifecycle_nodes},
            ],
        ),
        Node(
            package="slam",
            executable="nav2_readiness_monitor",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(autostart),
        ),
    ]
    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("log_level", default_value="info"),
            GroupAction(actions=[SetParameter("use_sim_time", use_sim_time), *nodes]),
        ]
    )

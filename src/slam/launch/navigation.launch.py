"""只启动当前建图导航链实际需要的 Nav2 生命周期节点。

不启动 Docking、Route Server 或 Waypoint Follower；这些属于以后任务层需求。控制器仍只
产生标准 ``Twist``，由速度平滑和带雷达急停的最终地形速度门依次约束。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def nav2_node(package, executable, parameters, log_level, remappings, name=None):
    """用统一日志等级、参数文件和 TF remap 创建一个 Nav2 生命周期节点。"""
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
    """创建最小 Nav2 节点集、生命周期管理和输入健康监控。"""
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    controller_remaps = tf_remaps + [("cmd_vel", "/cmd_vel_nav")]
    smoother_remaps = tf_remaps + [
        ("cmd_vel", "/cmd_vel_nav"),
        ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
    ]
    lifecycle_nodes = [
        # 顺序是生命周期管理器的配置/激活顺序；新增服务器时必须同步此列表。
        "controller_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "bt_navigator",
    ]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            # 重写 nav2.yaml 中每个同名参数。仅在 GroupAction 设置 use_sim_time 可能被节点
            # 自己的 YAML 值覆盖，造成一部分节点用系统时间、另一部分使用 /clock。
            param_rewrites={
                "autostart": autostart,
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 速度链刻意分成三个命名话题，便于逐段定位“谁把速度归零”：
    # controller -> cmd_vel_nav -> smoother -> cmd_vel_smoothed -> 最终地形/雷达安全门 ->
    # cmd_vel。Wakula gate 启用时由它持续发布最终速度；关闭时目标仲裁器必须接管
    # cmd_vel_smoothed 与全部安全/停车心跳，再唯一发布 cmd_vel。
    nodes = [
        nav2_node(
            "nav2_controller",
            "controller_server",
            configured_params,
            log_level,
            controller_remaps,
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
            controller_remaps,
        ),
        nav2_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            configured_params,
            log_level,
            smoother_remaps,
        ),
        nav2_node(
            "nav2_bt_navigator",
            "bt_navigator",
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
                # 固定关闭自动激活；readiness monitor 确认 scan、odom、定位 TF 后再请求启动。
                {"autostart": False},
                {"node_names": lifecycle_nodes},
            ],
        ),
        Node(
            package="slam",
            executable="nav2_readiness_monitor",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            condition=IfCondition(autostart),
        ),
        Node(
            package="slam",
            executable="navigation_health_monitor",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
    ]
    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="使用 /clock 作为 ROS 时间"
            ),
            DeclareLaunchArgument(
                "autostart", default_value="true", description="输入就绪后允许自动激活 Nav2"
            ),
            DeclareLaunchArgument("params_file", description="Nav2 完整参数 YAML 路径"),
            DeclareLaunchArgument(
                "log_level", default_value="info", description="Nav2 节点日志等级"
            ),
            GroupAction(actions=[SetParameter("use_sim_time", use_sim_time), *nodes]),
        ]
    )

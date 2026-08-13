"""单独启动仿真越障 Action 替身，不启动 Gazebo、SLAM 或自主导航。

它只用于在没有真实运动控制器时验证 ``TraverseObstacle`` 接口。未来真机接入后无需
运行本 launch，由运动控制团队提供同名 Action 服务端即可。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="quadruped_gazebo",
            executable="sim_traverse_obstacle",
            name="sim_traverse_obstacle",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            }],
        ),
    ])

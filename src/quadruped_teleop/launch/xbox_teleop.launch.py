"""独立启动 Xbox 设备驱动和 Wakula 手柄速度适配节点。

该入口刻意不包含 SLAM、Nav2、Collision Monitor 或运动控制器。``joy_node`` 负责读取
Linux 手柄设备并发布 ``sensor_msgs/Joy``；``xbox_teleop`` 负责按键安全状态机和 Twist
转换。默认结果仍写入独立的 ``/cmd_vel_joy``，不会直接抢占 Nav2 的 ``/cmd_vel``。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """创建 joy_node 与 xbox_teleop，并公开常用设备及话题参数。"""
    use_sim_time = LaunchConfiguration("use_sim_time")
    joy_topic = LaunchConfiguration("joy_topic")
    output_topic = LaunchConfiguration("output_topic")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="joy_node 使用的手柄编号；多手柄时先运行 joy_enumerate_devices",
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="/joy",
                description="joy_node 与 xbox_teleop 之间的 sensor_msgs/Joy 话题",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/cmd_vel_joy",
                description="手柄速度候选输出；默认不直接覆盖 Nav2 的 /cmd_vel",
            ),
            DeclareLaunchArgument(
                "autorepeat_rate",
                default_value="20.0",
                description="摇杆不变时 joy_node 重发状态的频率，单位 Hz",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("quadruped_teleop"), "config", "xbox.yaml"]
                ),
                description="Xbox 按键、轴、死区、超时和速度上限参数 YAML",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="通常保持 false；仅回放带 /clock 的 Joy rosbag 时启用",
            ),
            # joy_node 只负责 Linux 输入设备到 /joy。死区统一由 xbox_teleop 处理，
            # 因此这里设置为 0，避免两次死区缩放改变摇杆手感。
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "device_id": ParameterValue(
                            LaunchConfiguration("device_id"), value_type=int
                        ),
                        "deadzone": 0.0,
                        "autorepeat_rate": ParameterValue(
                            LaunchConfiguration("autorepeat_rate"), value_type=float
                        ),
                        "use_sim_time": use_sim_time,
                    }
                ],
                remappings=[("joy", joy_topic)],
            ),
            # 参数文件给出完整 Xbox 默认映射，launch 参数只覆盖两端话题和时钟。
            Node(
                package="quadruped_teleop",
                executable="xbox_teleop",
                name="xbox_teleop",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "input_topic": joy_topic,
                        "output_topic": output_topic,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )

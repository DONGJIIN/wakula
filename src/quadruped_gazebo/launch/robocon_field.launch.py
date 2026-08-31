"""独立启动 2026 Robocon 障碍赛参考场地和可替换通用机械狗测试载体。

本入口只启动 Gazebo Sim、ROS—Gazebo 桥和仿真专用固定 TF。它不会 include Wakula 的
``slam.launch.py``、Nav2、OpenCV、手柄或任何越障控制；需要联合测试时由使用者在另一个
终端显式启动算法，这样仿真环境和核心算法保持单向、可替换依赖。
"""

from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _reject_duplicate_world(_context):
    """在启动前拒绝同名 Gazebo world，避免 ROS 话题连接到另一份旧场景。

    Gazebo Transport 允许两个同名 world 同时存在，但 ROS bridge 只能看到同名服务和
    话题，最终会形成“画面里机器人在 A、里程计来自 B”的隐蔽故障。这里不擅自杀进程，
    而是让第二次启动给出明确错误，用户 Ctrl-C 旧 Gazebo 后再执行同一条命令即可。
    """
    try:
        result = subprocess.run(
            [
                "gz",
                "service",
                "-i",
                "-s",
                "/world/robocon_obstacle_field/scene/info",
            ],
            check=False,
            capture_output=True,
            text=True,
            # Transport discovery on a busy ROS/Gazebo workstation commonly needs 2～3 s.
            # A too-short query would silently miss the exact duplicate this guard prevents.
            timeout=5.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        # 缺少 gz 时后续 ros_gz_sim 会给出标准依赖错误；查询超时也不能据此误判重复。
        return []
    if "tcp://" in result.stdout:
        raise RuntimeError(
            "A robocon_obstacle_field Gazebo server is already running. "
            "Stop the old Gazebo launch with Ctrl-C before starting a new one."
        )
    return []


def generate_launch_description():
    """构造独立 Gazebo 场地、可选测试底盘、标准传感器桥和 TF。"""
    package_share = Path(get_package_share_directory("quadruped_gazebo"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    default_world = package_share / "worlds" / "robocon_obstacle_field.sdf"
    default_robot = package_share / "models" / "generic_quadruped" / "model.sdf"

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    spawn_robot = LaunchConfiguration("spawn_test_robot")
    robot_sdf = LaunchConfiguration("robot_sdf")
    robot_name = LaunchConfiguration("robot_name")
    publish_test_sensor_tf = LaunchConfiguration("publish_test_sensor_tf")
    keyboard_teleop = LaunchConfiguration("keyboard_teleop")
    enable_point_cloud_bridge = LaunchConfiguration("enable_point_cloud_bridge")
    use_sim_time = {"use_sim_time": True}

    # ros_gz_sim 自带的 launch 负责启动 Gazebo。这里默认实时运行；gui=false 时只启动
    # 服务端，适合 CI、远程 RK3588 或无显示器环境。
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s ", world]}.items(),
        condition=UnlessCondition(gui),
    )

    robot_group = GroupAction(
        condition=IfCondition(spawn_robot),
        actions=[
            LogInfo(
                msg=[
                    "Spawning Gazebo test quadruped: name=",
                    robot_name,
                    ", sdf=",
                    robot_sdf,
                    ". Wait for 'Entity creation successful' before starting SLAM.",
                ]
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world", "robocon_obstacle_field",
                    "-name", robot_name,
                    "-file", robot_sdf,
                    "-x", LaunchConfiguration("robot_x"),
                    "-y", LaunchConfiguration("robot_y"),
                    # 通用机械狗的不可见平面测试底座已位于 z=0。
                    "-z", "0.0",
                    "-Y", LaunchConfiguration("robot_yaw"),
                ],
            ),
            # /clock 独立桥接。仿真时间是所有 use_sim_time 节点的共同心跳，不能让高带宽
            # 点云或某个辅助传感器的转换阻塞它。
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="robocon_clock_bridge",
                output="screen",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                ],
                parameters=[use_sim_time],
            ),
            # SLAM/Nav2 的四条关键链路放在轻量桥中。这里刻意不桥接未被算法使用的
            # /scan/points，避免重复激光点云占用 DDS 和同一转换线程。
            # 桥接方向符号：[ 为 Gazebo->ROS，] 为 ROS->Gazebo，@ 为双向。
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="robocon_navigation_bridge",
                output="screen",
                arguments=[
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                ],
                # ROS 侧只接收仲裁后的唯一速度；Gazebo 侧仍保持模型插件默认 /cmd_vel。
                remappings=[("/cmd_vel", "/cmd_vel_gazebo")],
                parameters=[use_sim_time],
            ),
            # 仅暴露 Gazebo 的标准模型位姿服务。第三条“自动导航”命令中的仿真
            # TraverseObstacle adapter 用它越过测试狗无法靠平面轮式插件跨越的实体
            # 碰撞；SLAM、Nav2 和真机控制器均不订阅或调用此服务。
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="robocon_pose_service_bridge",
                output="screen",
                arguments=[
                    "/world/robocon_obstacle_field/set_pose@"
                    "ros_gz_interfaces/srv/SetEntityPose",
                ],
                parameters=[use_sim_time],
            ),
            # 仿真专用速度仲裁：手动 /cmd_vel_teleop 短时优先，算法继续使用标准
            # /cmd_vel。节点使用墙钟看门狗，因此明确不启用 use_sim_time。
            Node(
                package="quadruped_gazebo",
                executable="sim_cmd_vel_mux",
                name="sim_cmd_vel_mux",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # IMU 与 CameraInfo 属于辅助数据；独立桥确保其频率或格式异常不会拖住导航。
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="robocon_aux_sensor_bridge",
                output="screen",
                arguments=[
                    "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
                parameters=[use_sim_time],
            ),
            # Gazebo Harmonic 的 RGB-D PointCloudPacked 数值轴是 camera_link 约定
            # （x 前、y 左、z 上），但相机的 optical_frame_id 会把消息头标成光学坐标系。
            # 若直接交给算法，点云会被错误地再旋转一次。单独桥接并覆盖 frame_id，保证
            # 数据数值和 Header 一致；真机标准光学点云不需要也不应使用此仿真修正。
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="robocon_point_cloud_bridge",
                output="screen",
                condition=IfCondition(enable_point_cloud_bridge),
                arguments=[
                    "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                ],
                remappings=[("/camera/points", "/camera/depth/points")],
                parameters=[use_sim_time, {"override_frame_id": "camera_link"}],
            ),
            # 图像单独桥接，便于只重映射 ROS 侧默认相机名。相比 ros_gz_image，该路径
            # 在 Gazebo 仍发布最后一帧而 launch 正在关闭时不会触发 invalid Publisher。
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="robocon_image_bridge",
                output="screen",
                arguments=[
                    "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                ],
                remappings=[("/camera/image", "/camera/image_raw")],
                parameters=[use_sim_time],
            ),
            # OdometryPublisher 从 Gazebo 平面真值发布 odom->base_link；这一组固定 TF
            # 只匹配仓库自带的通用测试机械狗。换入真实模型时将参数设为 false，由新模型
            # 自己发布传感器外参，算法话题与 frame 名称保持不变。
            GroupAction(
                condition=IfCondition(publish_test_sensor_tf),
                actions=[
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="lidar_static_tf",
                        arguments=[
                            "--x", "0.0", "--y", "0.0", "--z", "0.28",
                            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                            "--frame-id", "base_link", "--child-frame-id", "lidar_link",
                        ],
                        parameters=[use_sim_time],
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="camera_static_tf",
                        arguments=[
                            "--x", "0.52", "--y", "0.0", "--z", "0.42",
                            "--roll", "0.0", "--pitch", "0.24", "--yaw", "0.0",
                            "--frame-id", "base_link", "--child-frame-id", "camera_link",
                        ],
                        parameters=[use_sim_time],
                    ),
                    # camera_link 使用机器人坐标约定（x 前、y 左、z 上）；图像使用 ROS
                    # 光学坐标约定（z 前、x 右、y 下）。
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="camera_optical_static_tf",
                        arguments=[
                            "--x", "0.0", "--y", "0.0", "--z", "0.0",
                            "--roll", "-1.570796", "--pitch", "0.0", "--yaw", "-1.570796",
                            "--frame-id", "camera_link", "--child-frame-id", "camera_optical_frame",
                        ],
                        parameters=[use_sim_time],
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="imu_static_tf",
                        arguments=[
                            "--x", "0.0", "--y", "0.0", "--z", "0.38",
                            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                            "--frame-id", "base_link", "--child-frame-id", "imu_link",
                        ],
                        parameters=[use_sim_time],
                    ),
                ],
            ),
        ],
    )

    # GUI 仿真默认自动打开一个键盘遥控终端，因此完整联调仍然只有 Gazebo、SLAM、
    # 自动导航三条用户命令。键盘进程只发布仿真专用 /cmd_vel_teleop，不进入 SLAM 或
    # 自主任务 launch；关闭 Gazebo 主 launch 时该终端也会随 --wait 进程一起结束。
    keyboard_terminal = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="sim_keyboard_teleop",
        output="screen",
        emulate_tty=True,
        prefix="gnome-terminal --wait --title='Wakula Simulation Keyboard' --",
        remappings=[("cmd_vel", "/cmd_vel_teleop")],
        parameters=[{"repeat_rate": 20.0, "key_timeout": 0.6}],
        condition=IfCondition(
            PythonExpression([
                "'", gui, "'.lower() in ('true', '1') and '",
                keyboard_teleop, "'.lower() in ('true', '1')",
            ])
        ),
    )

    return LaunchDescription(
        [
            # Snap 版 VS Code 会把 GTK_PATH 指向 /snap/code；Gazebo GUI 若继承它，可能
            # 错误加载 core20 的 libpthread 并报 GLIBC_PRIVATE。只清理本 launch 的
            # GUI 模块搜索路径，ROS/Gazebo 的 LD_LIBRARY_PATH 和系统环境保持不变。
            SetEnvironmentVariable("GTK_PATH", ""),
            SetEnvironmentVariable("GTK_EXE_PREFIX", ""),
            SetEnvironmentVariable("GIO_MODULE_DIR", ""),
            DeclareLaunchArgument("world", default_value=str(default_world)),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "keyboard_teleop",
                default_value="true",
                description=(
                    "Open an independent i/j/k/l simulation teleop terminal when GUI is enabled"
                ),
            ),
            DeclareLaunchArgument("spawn_test_robot", default_value="true"),
            DeclareLaunchArgument(
                "robot_sdf",
                default_value=str(default_robot),
                description="Absolute SDF path for the replaceable Gazebo test robot",
            ),
            DeclareLaunchArgument("robot_name", default_value="generic_quadruped"),
            DeclareLaunchArgument(
                "publish_test_sensor_tf",
                default_value="true",
                description="Publish sensor TF for the bundled generic test quadruped",
            ),
            DeclareLaunchArgument(
                "enable_point_cloud_bridge",
                default_value="true",
                description=(
                    "Bridge Gazebo depth points; the timed teleport workflow may "
                    "disable it while retaining RGB, lidar, odometry and TF"
                ),
            ),
            DeclareLaunchArgument("robot_x", default_value="-2.5"),
            DeclareLaunchArgument("robot_y", default_value="-0.2"),
            # 地面启动框西侧是当前参考布局中更开阔的自由区。默认朝西可避免相机启动
            # 第一帧斜视主坡的长侧边（该轮廓会同时像台阶和墙）；正式坐标公布后仍可
            # 用 robot_yaw:=... 覆盖，不影响 SLAM 或自动任务 launch。
            DeclareLaunchArgument("robot_yaw", default_value="3.141593"),
            OpaqueFunction(function=_reject_duplicate_world),
            gazebo_gui,
            gazebo_headless,
            robot_group,
            keyboard_terminal,
        ]
    )

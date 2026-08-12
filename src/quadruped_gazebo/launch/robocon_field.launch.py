"""独立启动 2026 Robocon 障碍赛参考场地和可选传感器测试底盘。

本入口只启动 Gazebo Sim、ROS—Gazebo 桥和仿真专用固定 TF。它不会 include Wakula 的
``slam.launch.py``、Nav2、OpenCV、手柄或任何越障控制；需要联合测试时由使用者在另一个
终端显式启动算法，这样仿真环境和核心算法保持单向、可替换依赖。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """构造独立 Gazebo 场地、可选测试底盘、标准传感器桥和 TF。"""
    package_share = Path(get_package_share_directory("quadruped_gazebo"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    default_world = package_share / "worlds" / "robocon_obstacle_field.sdf"
    default_robot = package_share / "models" / "sensor_test_base" / "model.sdf"

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    spawn_robot = LaunchConfiguration("spawn_test_robot")
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
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world", "robocon_obstacle_field",
                    "-name", "sensor_test_base",
                    "-file", str(default_robot),
                    "-x", LaunchConfiguration("robot_x"),
                    "-y", LaunchConfiguration("robot_y"),
                    # 模型内部的轮底已经位于 z=0；额外抬高会造成落地冲击和首帧里程计跳变。
                    "-z", "0.0",
                    "-Y", LaunchConfiguration("robot_yaw"),
                ],
            ),
            # 单一桥节点提供 SLAM/Nav2 所需的时钟、激光、里程计、速度和深度点云。
            # 桥接方向符号：[ 为 Gazebo->ROS，] 为 ROS->Gazebo，@ 为双向。
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="robocon_sensor_bridge",
                output="screen",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
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
                arguments=[
                    "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                ],
                remappings=[("/camera/points", "/camera/depth/points")],
                parameters=[use_sim_time, {"override_frame_id": "camera_link"}],
            ),
            # 图像使用 ros_gz_image 可避免 parameter_bridge 的额外图像复制路径。
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                name="robocon_image_bridge",
                output="screen",
                arguments=["/camera/image"],
                remappings=[("/camera/image", "/camera/image_raw")],
                parameters=[use_sim_time],
            ),
            # Gazebo 的 DiffDrive 发布 odom->base_link；以下仅补齐传感器固定外参。
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_static_tf",
                arguments=[
                    "--x", "0.0", "--y", "0.0", "--z", "0.34",
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
                    "--x", "0.22", "--y", "0.0", "--z", "0.28",
                    "--roll", "0.0", "--pitch", "0.20", "--yaw", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "camera_link",
                ],
                parameters=[use_sim_time],
            ),
            # camera_link 使用机器人坐标约定（x 前、y 左、z 上）；图像和点云使用 ROS
            # 光学坐标约定。显式发布两者关系，RViz 和 terrain_analyzer 才能正确解释点云。
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
                    "--x", "0.0", "--y", "0.0", "--z", "0.22",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "imu_link",
                ],
                parameters=[use_sim_time],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=str(default_world)),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("spawn_test_robot", default_value="true"),
            DeclareLaunchArgument("robot_x", default_value="-2.5"),
            DeclareLaunchArgument("robot_y", default_value="-0.2"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0"),
            gazebo_gui,
            gazebo_headless,
            robot_group,
        ]
    )

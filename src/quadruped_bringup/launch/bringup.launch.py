from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_control = LaunchConfiguration("use_control")
    yolo = LaunchConfiguration("yolo")
    yolo_model = LaunchConfiguration("yolo_model")
    description_file = PathJoinSubstitution([
        FindPackageShare("quadruped_description"), "urdf", "quadruped.urdf.xacro"
    ])
    controllers_file = PathJoinSubstitution([
        FindPackageShare("quadruped_control"), "config", "controllers.yaml"
    ])
    terrain_file = PathJoinSubstitution([
        FindPackageShare("quadruped_perception"), "config", "terrain.yaml"
    ])
    vision_file = PathJoinSubstitution([
        FindPackageShare("quadruped_perception"), "config", "vision.yaml"
    ])
    yolo_file = PathJoinSubstitution([
        FindPackageShare("quadruped_perception"), "config", "yolo.yaml"
    ])
    crossing_file = PathJoinSubstitution([
        FindPackageShare("quadruped_planning"), "config", "crossing.yaml"
    ])
    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", description_file])
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_control", default_value="true"),
        DeclareLaunchArgument(
            "yolo",
            default_value="false",
            description="Start optional YOLO ONNX inference (off by default).",
        ),
        DeclareLaunchArgument(
            "yolo_model",
            default_value="",
            description="Absolute path to a YOLO ONNX model.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            condition=IfCondition(use_control),
            parameters=[robot_description, controllers_file],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            condition=IfCondition(use_control),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["leg_controller", "--controller-manager", "/controller_manager"],
            condition=IfCondition(use_control),
        ),
        Node(
            package="quadruped_perception",
            executable="terrain_analyzer",
            name="terrain_analyzer",
            output="screen",
            parameters=[terrain_file],
        ),
        Node(
            package="quadruped_perception",
            executable="vision_obstacle_detector",
            output="screen",
            parameters=[vision_file],
        ),
        Node(
            package="quadruped_perception",
            executable="yolo_obstacle_detector",
            output="screen",
            parameters=[
                yolo_file,
                {
                    "enabled": ParameterValue(yolo, value_type=bool),
                    "model_path": yolo_model,
                },
            ],
            condition=IfCondition(yolo),
        ),
        Node(
            package="quadruped_planning",
            executable="obstacle_crossing_manager",
            output="screen",
            parameters=[crossing_file],
        ),
    ])

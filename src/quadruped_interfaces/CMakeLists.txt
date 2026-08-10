cmake_minimum_required(VERSION 3.8)
project(quadruped_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TraverseObstacle.action"
  "msg/CrossingCommand.msg"
  "msg/CrossingStatus.msg"
  DEPENDENCIES unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

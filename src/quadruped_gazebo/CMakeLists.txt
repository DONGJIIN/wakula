cmake_minimum_required(VERSION 3.8)
project(quadruped_gazebo)

find_package(ament_cmake REQUIRED)

# 场地、测试模型与 launch 安装到 share；唯一的 Python 节点只仲裁 Gazebo 测试速度，
# 不实现真机控制器、步态或 SDK。
install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

# 仿真速度仲裁器是一个轻量 Python 可执行文件；不引入真机控制器或 SDK。
install(PROGRAMS scripts/sim_cmd_vel_mux.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_cmd_vel_mux
)
install(PROGRAMS scripts/sim_traverse_obstacle.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_traverse_obstacle
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_competition_field test/test_competition_field.py)
endif()

ament_package()

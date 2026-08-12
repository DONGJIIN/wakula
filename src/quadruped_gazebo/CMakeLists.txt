cmake_minimum_required(VERSION 3.8)
project(quadruped_gazebo)

find_package(ament_cmake REQUIRED)

# 仿真包只有 SDF、launch 和测试，不编译控制器。安装到 share 后可被 ros2 launch
# 与 Gazebo Sim 在源码/安装工作空间中统一查找。
install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_competition_field test/test_competition_field.py)
endif()

ament_package()

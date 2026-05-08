cmake_minimum_required(VERSION 3.8)
project(zyarm_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_real_ros2_control_launch tests/test_real_ros2_control_launch.py)
endif()

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

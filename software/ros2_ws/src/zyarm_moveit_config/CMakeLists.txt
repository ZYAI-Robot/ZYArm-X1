cmake_minimum_required(VERSION 3.8)
project(zyarm_moveit_config)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_demo_x1_standard_gazebo_launch tests/test_demo_x1_standard_gazebo_launch.py)
  ament_add_pytest_test(test_demo_x1_standard_real_launch tests/test_demo_x1_standard_real_launch.py)
endif()

install(
  DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

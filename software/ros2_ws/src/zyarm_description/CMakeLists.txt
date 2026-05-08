cmake_minimum_required(VERSION 3.8)
project(zyarm_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_x1_standard_xacro_gazebo tests/test_x1_standard_xacro_gazebo.py)
endif()

install(
  DIRECTORY config launch meshes rviz urdf docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

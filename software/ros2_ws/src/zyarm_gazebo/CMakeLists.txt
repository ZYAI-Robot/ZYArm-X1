cmake_minimum_required(VERSION 3.8)
project(zyarm_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/grasp_manager.py
  scripts/wait_for_controller_types.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

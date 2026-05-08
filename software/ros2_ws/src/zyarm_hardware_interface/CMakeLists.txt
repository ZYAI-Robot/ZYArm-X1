cmake_minimum_required(VERSION 3.8)
project(zyarm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(${PROJECT_NAME} SHARED
  src/diagnostics.cpp
  src/joint_mapping.cpp
  src/serial_transport.cpp
  src/shell_protocol.cpp
  src/zyarm_system_hardware.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface plugin/zyarm_hardware_interface.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY plugin
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_shell_protocol test/test_shell_protocol.cpp)
  target_link_libraries(test_shell_protocol ${PROJECT_NAME})

  ament_add_gtest(test_joint_mapping test/test_joint_mapping.cpp)
  target_link_libraries(test_joint_mapping ${PROJECT_NAME})

  ament_add_gtest(test_serial_transport test/test_serial_transport.cpp)
  target_link_libraries(test_serial_transport ${PROJECT_NAME})

  ament_add_gtest(test_system_hardware_fake test/test_system_hardware_fake.cpp)
  target_link_libraries(test_system_hardware_fake ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

cmake_minimum_required(VERSION 3.16)
project(zyarm_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(zyarm_sdk
  src/arm.cpp
  src/mapping.cpp
  src/protocol.cpp
  src/retarget.cpp
  src/safety.cpp
  src/teleop.cpp
  src/transport.cpp
  src/types.cpp
)
add_library(zyarm_sdk::zyarm_sdk ALIAS zyarm_sdk)

target_include_directories(zyarm_sdk
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(UNIX AND NOT APPLE)
  target_link_libraries(zyarm_sdk PUBLIC pthread)
endif()

install(TARGETS zyarm_sdk EXPORT zyarm_sdkTargets)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT zyarm_sdkTargets
  NAMESPACE zyarm_sdk::
  DESTINATION lib/cmake/zyarm_sdk
)

option(ZYARM_SDK_BUILD_TESTS "Build ZYArm SDK tests" ON)
if(ZYARM_SDK_BUILD_TESTS)
  enable_testing()
  add_executable(test_protocol tests/test_protocol.cpp)
  target_link_libraries(test_protocol PRIVATE zyarm_sdk)
  add_test(NAME test_protocol COMMAND test_protocol)

  add_executable(test_arm tests/test_arm.cpp)
  target_link_libraries(test_arm PRIVATE zyarm_sdk)
  target_include_directories(test_arm PRIVATE tests)
  add_test(NAME test_arm COMMAND test_arm)

  add_executable(test_teleop tests/test_teleop.cpp)
  target_link_libraries(test_teleop PRIVATE zyarm_sdk)
  target_include_directories(test_teleop PRIVATE tests)
  add_test(NAME test_teleop COMMAND test_teleop)
endif()

option(ZYARM_SDK_BUILD_EXAMPLES "Build ZYArm SDK examples" OFF)
if(ZYARM_SDK_BUILD_EXAMPLES)
  add_executable(read_state examples/read_state.cpp)
  target_link_libraries(read_state PRIVATE zyarm_sdk)

  add_executable(fast_io_loop examples/fast_io_loop.cpp)
  target_link_libraries(fast_io_loop PRIVATE zyarm_sdk)

  add_executable(teleop_step examples/teleop_step.cpp)
  target_link_libraries(teleop_step PRIVATE zyarm_sdk)
endif()

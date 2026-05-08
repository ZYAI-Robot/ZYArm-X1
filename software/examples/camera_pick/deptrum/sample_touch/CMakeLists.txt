cmake_minimum_required(VERSION 3.16)
project(sample_touch)

# 添加 ZYArm SDK 头文件路径
include_directories(${CMAKE_SOURCE_DIR}/../lib/zyarm_sdk/include)

set(ZYARM_SDK_SRC_DIR ${CMAKE_SOURCE_DIR}/../lib/zyarm_sdk/src)
add_executable(sample_touch
  sample_touch.cpp
  ../sample_helper.cc
  ${ZYARM_SDK_SRC_DIR}/arm.cpp
  ${ZYARM_SDK_SRC_DIR}/mapping.cpp
  ${ZYARM_SDK_SRC_DIR}/protocol.cpp
  ${ZYARM_SDK_SRC_DIR}/retarget.cpp
  ${ZYARM_SDK_SRC_DIR}/safety.cpp
  ${ZYARM_SDK_SRC_DIR}/teleop.cpp
  ${ZYARM_SDK_SRC_DIR}/transport.cpp
  ${ZYARM_SDK_SRC_DIR}/types.cpp
)
target_link_libraries(sample_touch ${DEPTRUM_STREAM_LIB_NAME} ${OpenCV_LIBS} ${EXTRA_LIBS})
install(TARGETS sample_touch DESTINATION samples/bin)

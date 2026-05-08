cmake_minimum_required(VERSION 3.10)
project(RobotArmPick)

set(CMAKE_CXX_STANDARD 17)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 设置Orbbec SDK路径
set(OrbbecSDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/OrbbecSDK/build" CACHE PATH "Path to Orbbec SDK build directory")
find_package(OrbbecSDK REQUIRED CONFIG PATHS ${OrbbecSDK_DIR} NO_DEFAULT_PATH)

add_subdirectory(../../../zyarm_sdk/cpp ${CMAKE_BINARY_DIR}/zyarm_sdk_build)

# 构建示例可执行文件
add_executable(${PROJECT_NAME}
    Sample-RobotArmPick.cpp
)

# 链接Orbbec SDK和OpenCV
target_link_libraries(${PROJECT_NAME} PRIVATE 
    OrbbecSDK::OrbbecSDK 
    zyarm_sdk::zyarm_sdk
    ${OpenCV_LIBS}
)

# 包含头文件目录
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${OrbbecSDK_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# 设置输出目录为build/bin
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

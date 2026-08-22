cmake_minimum_required(VERSION 4.0)
project(raphael_robot_hardware)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(${PROJECT_NAME}
        SHARED
        src/tricycle_hardware_interface.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp dynamixel_sdk hardware_interface)

# 添加私有包含目录（仅编译时使用，不传播至依赖目标）
target_include_directories(${PROJECT_NAME} PRIVATE include)


ament_export_dependencies(hardware_interface rclcpp dynamixel_sdk)

ament_package()

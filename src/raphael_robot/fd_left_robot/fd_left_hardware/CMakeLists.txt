cmake_minimum_required(VERSION 3.28)
project(fd_left_hardware)

include(GenerateExportHeader)

find_package(ament_cmake REQUIRED)
find_package(fd_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/fd_left_hardware_interface.cpp
)

# ✅ 创建子目录 + 指定导出文件路径
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/fd_left_hardware)
generate_export_header(${PROJECT_NAME}
        EXPORT_MACRO_NAME FD_LEFT_HARDWARE_PUBLIC
        EXPORT_FILE_NAME fd_left_hardware/fd_left_hardware_visibility.h
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        fd_vendor::fd_vendor
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        hardware_interface::hardware_interface
)

add_executable(test_main src/fd_left_hardware_interface.cpp)
target_link_libraries(test_main PRIVATE ${PROJECT_NAME})

# install 时也要对应新路径
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/fd_left_hardware/fd_left_hardware_visibility.h
        DESTINATION include/fd_left_hardware
)

ament_package()
cmake_minimum_required(VERSION 3.28)
project(fd_left_description)

# 启用 compile_commands.json 导出，以支持 IDE 代码索引
set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS} CACHE INTERNAL "")

# 设定 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
        DIRECTORY urdf ros2_control config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()

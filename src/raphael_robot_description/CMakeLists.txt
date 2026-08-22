cmake_minimum_required(VERSION 4.0)
project(raphael_robot_description)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        urdf meshes rviz
        DESTINATION share/${PROJECT_NAME}
)


ament_package()

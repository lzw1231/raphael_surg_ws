cmake_minimum_required(VERSION 3.11)
project(st3215)

set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS} CACHE INTERNAL "")

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

find_package(ament_cmake REQUIRED)

set(ST_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libSCServo.so)

# Setup vendor library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:.>
)

# 核心：使用绝对路径链接，确保导出时不会变成 -l 错误
target_link_libraries(${PROJECT_NAME} INTERFACE
        ${CMAKE_INSTALL_PREFIX}/lib/libSCServo.so
)

# Install
# DESTINATION . 表示安装到 include 根目录，配合 include/st3215/ 文件夹结构
# 实现 #include <st3215/SCServo.hpp>
install(DIRECTORY include/ DESTINATION .)

install(FILES ${ST_LIB_SRC} DESTINATION lib)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
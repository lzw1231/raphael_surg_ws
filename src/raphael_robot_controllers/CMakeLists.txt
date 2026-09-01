# CMake 最低版本要求
cmake_minimum_required(VERSION 4.0)
project(raphael_robot_controllers)

# 指定 C++17 标准并强制生效
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GCC/Clang 启用严格编译警告
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# 声明 ROS 2 及第三方依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(pluginlib REQUIRED)

# 构建共享库目标
add_library(${PROJECT_NAME} SHARED
        src/my_controller.cpp
)

# 指定私有头文件搜索路径
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 关联 ROS 2 依赖（自动注入 include/link/compile 属性）
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_lifecycle
        controller_interface
        hardware_interface
        example_interfaces
        pluginlib
)

# 导出 controller 插件描述文件供 pluginlib 运行时加载
pluginlib_export_plugin_description_file(controller_interface raphael_robot_controller_plugin.xml)

# 安装共享库至 lib 目录
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
)

# 安装头文件至包级 include 目录
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

# 生成 ament 包索引与配置元数据
ament_package()
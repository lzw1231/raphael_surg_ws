cmake_minimum_required(VERSION 4.0)
project(raphael_robot_hardware)

# ==============================================================================
# 编译配置
# ==============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用基础警告
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# 抑制特定编译器警告 (Clang)
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 1. 忽略 __COUNTER__ 宏产生的 C2y 扩展警告 (源自 PLUGINLIB_EXPORT_CLASS)
    add_compile_options(-Wno-c2y-extensions)
    # 2. 忽略非虚析构函数警告 (源自第三方库 SMS_STS)
    add_compile_options(-Wno-delete-non-abstract-non-virtual-dtor)
endif ()

# ==============================================================================
# 依赖查找
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ==============================================================================
# 第三方库配置 (STS3215/SCServo)
# ==============================================================================
set(STS3215_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/sts3215")
set(STS3215_INCLUDE "${STS3215_ROOT}/include")
set(STS3215_LIB "${STS3215_ROOT}/lib/libSCServo.a")

# ==============================================================================
# 目标构建
# ==============================================================================
add_library(${PROJECT_NAME} SHARED
        src/tricycle_hardware_interface.cpp
        src/snake_hardware_interface.cpp
)

# 包含头文件目录
target_include_directories(${PROJECT_NAME} PRIVATE
        ${STS3215_INCLUDE}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# 链接 ROS 2 依赖
ament_target_dependencies(${PROJECT_NAME} PUBLIC
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        pluginlib
)

# 链接第三方静态库
target_link_libraries(${PROJECT_NAME} PRIVATE ${STS3215_LIB})

# ==============================================================================
# 插件导出
# ==============================================================================
pluginlib_export_plugin_description_file(hardware_interface raphael_robot_hardware_interface_plugin.xml)

# ==============================================================================
# 安装规则
# ==============================================================================
install(
        TARGETS ${PROJECT_NAME}
        DESTINATION lib
)

install(
        FILES raphael_robot_hardware_interface_plugin.xml
        DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# 打包
# ==============================================================================
ament_package()
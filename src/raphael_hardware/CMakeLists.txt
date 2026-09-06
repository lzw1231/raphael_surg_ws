# ==============================================================================
# 项目配置
# ==============================================================================
# 最低 CMake 版本要求 (适配 Jazzy + Clang-22 环境)
cmake_minimum_required(VERSION 3.28)

# CMP0167: FindPython 行为策略。设为 OLD 以兼容 ament_cmake 的现有逻辑
cmake_policy(SET CMP0167 OLD)

project(raphael_hardware)

# 强制使用 C++17 标准，禁用编译器特定扩展以确保跨平台兼容性
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# 依赖查找
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fd_vendor REQUIRED)

# ==============================================================================
# 库构建
# ==============================================================================
# 构建共享库作为 ros2_control 硬件接口插件
add_library(${PROJECT_NAME} SHARED src/MTM/fd_left/fd_left_hardware.cpp)

# 配置头文件搜索路径
# BUILD_INTERFACE: 构建时当前源码目录
# INSTALL_INTERFACE: 安装后相对路径
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 定义依赖列表，统一管理以便复用
set(DEPENDENCIES
        fd_vendor
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

# 启用 DLL 符号导出宏，配合 visibility_control.h 控制动态库符号可见性
target_compile_definitions(${PROJECT_NAME} PRIVATE RAPHAEL_HARDWARE_BUILDING_DLL)

# 自动注入依赖的头文件路径、编译定义及链接库
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# ==============================================================================
# 插件注册
# ==============================================================================
# 注册 hardware_interface 插件描述文件
pluginlib_export_plugin_description_file(hardware_interface raphael_hardware_plugins.xml)

# ==============================================================================
# 安装与导出
# ==============================================================================
# 安装目标库并导出 CMake Target，供其他包通过 find_package 调用
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# 安装公共头文件
install(DIRECTORY include/
        DESTINATION include
)

# 导出配置，确保依赖传递性
# 其他包链接本库时，会自动链接上述 DEPENDENCIES
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

# 生成并安装 package.xml 元数据
ament_package()
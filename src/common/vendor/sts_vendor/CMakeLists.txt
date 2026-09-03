cmake_minimum_required(VERSION 3.28)
project(sts_vendor)

# 导出 compile_commands.json 供 IDE 代码补全工具使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS} CACHE INTERNAL "")

# 设置 C++ 标准（若未指定则默认使用 C++17）
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

# 查找 ROS2 ament 构建系统
find_package(ament_cmake REQUIRED)

# 定义第三方动态库 (.so) 的源文件路径
set(ST_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libSCServo.so)

# ==========================================
# 定义接口库 (Interface Library)
# ==========================================
# 接口库不生成实际二进制文件，仅用于向依赖方传递头文件路径和链接属性
add_library(${PROJECT_NAME} INTERFACE)

# 配置头文件包含目录（使用生成器表达式区分构建与安装状态）
target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 构建时：使用源码树中的 include 目录
        $<INSTALL_INTERFACE:.>                                  # 安装后：使用安装根目录
)

target_link_libraries(${PROJECT_NAME} INTERFACE
        ${CMAKE_INSTALL_PREFIX}/lib/libSCServo.so
)

# ==========================================
# 安装规则 (Install)
# ==========================================
# 安装头文件至安装根目录
install(DIRECTORY include/ DESTINATION .)

# 安装第三方动态库至 lib 目录
install(FILES ${ST_LIB_SRC} DESTINATION lib)

# 安装接口库目标并生成导出集
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

# ==========================================
# 导出规则 (Export)
# ==========================================
# 导出 CMake 目标，使其他 ROS2 包可通过 find_package(sts_vendor) 发现并使用此包
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


# 生成 ament 包配置文件 (package.xml 对应的 CMake 注册)
ament_package()
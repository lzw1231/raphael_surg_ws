cmake_minimum_required(VERSION 3.28)
project(fd_vendor)

set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS} CACHE INTERNAL "")

# 定义 SDK 版本号
set(FD_SDK_VERSION "3.17.8")

# 设置 C++ 标准
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

find_package(ament_cmake REQUIRED)

# ==========================================
# 1. 版本解析与库文件路径定义
# ==========================================
# 将版本号字符串按 "." 分割，提取主版本号（用于库文件重命名）
string(REPLACE "." ";" VERSION_SDK_LIST ${FD_SDK_VERSION})
list(GET VERSION_SDK_LIST 0 VERSION_SDK_MAJOR)

# 定义源目录中的库文件完整路径
set(DHD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.so.${FD_SDK_VERSION})
set(DRD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.so.${FD_SDK_VERSION})


# ==========================================
# 2. 创建接口库目标 (Interface Library)
# ==========================================
add_library(${PROJECT_NAME} INTERFACE)

# 设置头文件包含目录
# BUILD_INTERFACE: 构建当前包时使用源码目录下的 include
# INSTALL_INTERFACE: 安装后，下游包引用时使用安装前缀根目录 (.)
target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:.>
)

# 链接共享库
# 注意：此处使用绝对路径链接安装后的库文件，确保下游包能正确找到 .so
#target_link_libraries(${PROJECT_NAME} INTERFACE
#        ${CMAKE_INSTALL_PREFIX}/lib/libdhd.so.${VERSION_SDK_MAJOR}
#        ${CMAKE_INSTALL_PREFIX}/lib/libdrd.so.${VERSION_SDK_MAJOR}
#)

target_link_libraries(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.so.${FD_SDK_VERSION}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.so.${FD_SDK_VERSION}>
        $<INSTALL_INTERFACE:lib/libdhd.so.${VERSION_SDK_MAJOR}>
        $<INSTALL_INTERFACE:lib/libdrd.so.${VERSION_SDK_MAJOR}>
)

# ==========================================
# 3. 安装规则 (Install)
# ==========================================
# 安装头文件目录到安装前缀的根目录 (保持原有目录结构)
install(
        DIRECTORY include/
        DESTINATION .
)

# 安装共享库文件，并重命名为带主版本号的形式 (例如 libdhd.so.3)
install(FILES ${DHD_LIB_SRC} DESTINATION lib RENAME libdhd.so.${VERSION_SDK_MAJOR})
install(FILES ${DRD_LIB_SRC} DESTINATION lib RENAME libdrd.so.${VERSION_SDK_MAJOR})

# 安装接口库目标本身，并导出配置
install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

# ==========================================
# 4. 导出配置 (Export)
# ==========================================
# 导出 CMake 目标配置，使其他包可通过 find_package(fd_vendor) 使用
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_dhd
            test/test_dhd.cpp
    )

    target_link_libraries(test_dhd
            ${PROJECT_NAME}
    )
endif ()

ament_package()
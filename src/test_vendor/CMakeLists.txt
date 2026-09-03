cmake_minimum_required(VERSION 3.28)
project(test_vendor)

# 设定 C++ 标准（若未指定则默认为 C++17）
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

# 依赖 ROS2 ament 构建系统
find_package(ament_cmake REQUIRED)

# 查找依赖的 Vendor 包
find_package(fd_vendor REQUIRED)
find_package(sts_vendor REQUIRED)

# 定义测试可执行文件
add_executable(test_vendor src/test_vendor.cpp)

# 链接 Vendor 库目标
target_link_libraries(test_vendor
        fd_vendor::fd_vendor
        sts_vendor::sts_vendor
)

# 安装可执行文件至指定目录
install(TARGETS test_vendor DESTINATION lib/${PROJECT_NAME})

# 生成 ament 包元数据
ament_package()
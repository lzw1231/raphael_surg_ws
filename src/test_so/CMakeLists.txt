cmake_minimum_required(VERSION 3.28)
project(test_so)


if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

find_package(ament_cmake REQUIRED)

# ✅ 分别 find 两个 vendor 包
find_package(fd_vendor REQUIRED)
find_package(sts_vendor REQUIRED)

add_executable(test_so src/test_so.cpp)

# ✅ 链接对应的 target
target_link_libraries(test_so
        fd_vendor::fd_vendor
        sts_vendor::sts_vendor
)

install(TARGETS test_so DESTINATION lib/${PROJECT_NAME})

ament_package()
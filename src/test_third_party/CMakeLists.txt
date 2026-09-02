cmake_minimum_required(VERSION 3.11)
project(test_third_party)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

find_package(ament_cmake REQUIRED)
find_package(third_party REQUIRED)

add_executable(test_link src/test_link.cpp)

# 直接链接 ament 导出的 target
target_link_libraries(test_link third_party::third_party)

install(TARGETS test_link
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
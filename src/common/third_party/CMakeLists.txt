cmake_minimum_required(VERSION 3.11)
project(third_party)

set(FD_SDK_VERSION "3.17.8")

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

find_package(ament_cmake REQUIRED)

# ==========================================
# 1. 设置版本与源文件路径
# ==========================================
string(REPLACE "." ";" VERSION_SDK_LIST ${FD_SDK_VERSION})
list(GET VERSION_SDK_LIST 0 VERSION_SDK_MAJOR)

set(DHD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.so.${FD_SDK_VERSION})
set(DRD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.so.${FD_SDK_VERSION})
set(ST_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libSCServo.so)

# ==========================================
# 2. Setup vendor library (完全照搬官方)
# ==========================================
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 【核心天机】：官方直接写死 CMAKE_INSTALL_PREFIX 绝对路径！
# 不用 BUILD_INTERFACE，不用 INSTALL_INTERFACE，直接写绝对路径！
target_link_libraries(${PROJECT_NAME} INTERFACE
        ${CMAKE_INSTALL_PREFIX}/lib/libdhd.so.${VERSION_SDK_MAJOR}
        ${CMAKE_INSTALL_PREFIX}/lib/libdrd.so.${VERSION_SDK_MAJOR}
        ${CMAKE_INSTALL_PREFIX}/lib/libSCServo.so
)

# ==========================================
# 3. Install (完全照搬官方结构)
# ==========================================
install(
        DIRECTORY include/
        DESTINATION include
)

install(FILES ${DHD_LIB_SRC} DESTINATION lib RENAME libdhd.so.${VERSION_SDK_MAJOR})
install(FILES ${DRD_LIB_SRC} DESTINATION lib RENAME libdrd.so.${VERSION_SDK_MAJOR})
install(FILES ${ST_LIB_SRC} DESTINATION lib)

# 【核心天机】：只 install ${PROJECT_NAME}，绝不 install dhd/drd！
install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

# ==========================================
# 4. Export
# ==========================================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
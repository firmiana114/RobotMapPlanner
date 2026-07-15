cmake_minimum_required(VERSION 3.16)

project(unitree_odometer_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/unitree_sdk2/CMakeLists.txt")
    set(BUILD_EXAMPLES OFF CACHE BOOL "Build Unitree SDK2 examples" FORCE)
    add_subdirectory(unitree_sdk2)
else()
    find_package(unitree_sdk2 REQUIRED)
endif()

add_executable(g1_odometer_probe
    src/g1_odometer_probe.cpp
)

target_link_libraries(g1_odometer_probe
    unitree_sdk2
)

cmake_minimum_required(VERSION 3.18)
project(RobotMapPlanner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PYBIND11_FINDPYTHON ON)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

add_library(rmp_core STATIC cpp/src/core.cpp)
target_include_directories(rmp_core PUBLIC cpp/include)
target_compile_options(rmp_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(rmp_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(_core cpp/src/bindings.cpp)
target_link_libraries(_core PRIVATE rmp_core)
install(TARGETS _core LIBRARY DESTINATION robot_map_planner)

include(CTest)
if(BUILD_TESTING)
  add_executable(rmp_core_tests cpp/tests/test_core.cpp)
  target_link_libraries(rmp_core_tests PRIVATE rmp_core)
  add_test(NAME rmp_core_tests COMMAND rmp_core_tests)
endif()

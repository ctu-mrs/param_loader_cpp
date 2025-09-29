cmake_minimum_required(VERSION 3.12)
project("param_loader_cpp" VERSION 0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-O0")
add_definitions("-g")

set(DEPENDENCIES rclcpp yaml-cpp Eigen3)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# --------------------------------------------------------------
# |                       compile                              |
# --------------------------------------------------------------

# example param provider
add_executable(run_ParamProvider examples/example_param_provider.cpp)
target_include_directories(
  run_ParamProvider PUBLIC include)
target_link_libraries(run_ParamProvider PUBLIC yaml-cpp Eigen3::Eigen)

# example param loader
add_executable(run_ParamLoader examples/example_param_loader.cpp)
target_include_directories(
  run_ParamLoader PUBLIC include)
target_link_libraries(run_ParamLoader PUBLIC yaml-cpp Eigen3::Eigen)

# example node
add_executable(run_Node examples/example_node.cpp)
target_include_directories(
  run_Node PUBLIC include)
target_link_libraries(run_Node PUBLIC rclcpp::rclcpp yaml-cpp Eigen3::Eigen)

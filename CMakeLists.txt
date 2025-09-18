cmake_minimum_required(VERSION 3.12)
project(param_loader_cpp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-O0")
add_definitions("-g")

set(DEPENDENCIES yaml-cpp Eigen3)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(include)
# --------------------------------------------------------------
# |                       compile                              |
# --------------------------------------------------------------

# Param provider

add_library(ParamProvider SHARED src/param_provider/param_provider.cpp)
target_link_libraries(ParamProvider PUBLIC yaml-cpp Eigen3)

# Param loader

add_library(ParamLoader SHARED src/param_loader/param_loader.cpp)
target_link_libraries(ParamLoader PUBLIC ParamProvider)

# --------------------------------------------------------------
# |                           install                          |
# --------------------------------------------------------------

install(
  TARGETS ParamLoader
  EXPORT export_${PROJECT_NAME}_Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include DESTINATION .)

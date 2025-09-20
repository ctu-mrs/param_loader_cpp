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

set(DEPENDENCIES yaml-cpp Eigen3)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# provides variables containing installation directories
include(GNUInstallDirs)
include(CMakePrintHelpers)

# --------------------------------------------------------------
# |                       compile                              |
# --------------------------------------------------------------

# Param provider
add_library(ParamProvider SHARED src/param_provider/param_provider.cpp)
target_include_directories(
  ParamProvider PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
target_link_libraries(ParamProvider PUBLIC yaml-cpp Eigen3::Eigen)

# Param loader
add_library(ParamLoader SHARED src/param_loader/param_loader.cpp)
target_include_directories(
  ParamLoader PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
target_link_libraries(ParamLoader PUBLIC ParamProvider)

# --------------------------------------------------------------
# |                           install                          |
# --------------------------------------------------------------
# ##############################################################################
# install and export all the targets together              #
# ##############################################################################
# setting up the install for the target, export name will be used when exporting
install(
  TARGETS ParamProvider ParamLoader
  EXPORT ${PROJECT_NAME}_Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# export command takes the target and generates a cmake file
install(
  EXPORT ${PROJECT_NAME}_Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

# ##############################################################################
# configure cmake config for export                        #
# ##############################################################################
# prepare the basic version config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

# set the variables that must be exported for use by other libraries
set(INCLUDE_INSTALL_DIR
    ${CMAKE_INSTALL_INCLUDEDIR}
    CACHE PATH "Location of header files")
set(LIB_INSTALL_DIR
    ${CMAKE_INSTALL_LIBDIR}
    CACHE PATH "Location of library files")

# cmake helper to generate config file using cmake.in file
configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
  PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR)

# ##############################################################################
# install/copy the header files and cmake config           #
# ##############################################################################
# copy the generated cmake files to the install destination
install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

# finally copy the header files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "param_loader_cpp::ParamProvider" for configuration ""
set_property(TARGET param_loader_cpp::ParamProvider APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(param_loader_cpp::ParamProvider PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libParamProvider.so"
  IMPORTED_SONAME_NOCONFIG "libParamProvider.so"
  )

list(APPEND _cmake_import_check_targets param_loader_cpp::ParamProvider )
list(APPEND _cmake_import_check_files_for_param_loader_cpp::ParamProvider "${_IMPORT_PREFIX}/lib/libParamProvider.so" )

# Import target "param_loader_cpp::ParamLoader" for configuration ""
set_property(TARGET param_loader_cpp::ParamLoader APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(param_loader_cpp::ParamLoader PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libParamLoader.so"
  IMPORTED_SONAME_NOCONFIG "libParamLoader.so"
  )

list(APPEND _cmake_import_check_targets param_loader_cpp::ParamLoader )
list(APPEND _cmake_import_check_files_for_param_loader_cpp::ParamLoader "${_IMPORT_PREFIX}/lib/libParamLoader.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

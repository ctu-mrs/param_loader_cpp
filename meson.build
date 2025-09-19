project('param_loader_cpp', 'cpp',
  version : '0.1',
  default_options : ['cpp_std=c++20', 'warning_level=3', 'buildtype=release'])

# Compiler flags
add_project_arguments('-Wall', '-Wextra', '-Wpedantic', '-O0', '-g', language : 'cpp')

# Dependencies
yamlcpp_dep = dependency('yaml-cpp', required : true)
eigen3_dep = dependency('eigen3', required : true)

# ParamProvider library
param_provider = shared_library('ParamProvider',
  'src/param_provider/param_provider.cpp',
  include_directories : 'include',
  dependencies : [yamlcpp_dep, eigen3_dep],
  install: true)

# ParamLoader library
param_loader = shared_library('ParamLoader',
  'src/param_loader/param_loader.cpp',
  include_directories : 'include',
  link_with: param_provider,
  dependencies : [yamlcpp_dep, eigen3_dep],
  install: true)

executable('exec_pp', 'examples/example_param_provider.cpp',
  include_directories : 'include',
  link_with: param_provider,
  dependencies : [yamlcpp_dep, eigen3_dep])

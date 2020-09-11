cmake_minimum_required(VERSION 3.10)
project(particle-filter LANGUAGES CXX)

# -------------------------- Set Standard Warnings and Settings ----------------------------
include(cmake/StandardProjectSettings.cmake)
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

include(cmake/StaticAnalyzers.cmake)

add_subdirectory(src)

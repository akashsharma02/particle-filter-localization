cmake_minimum_required(VERSION 3.10)
project(particle-filter)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(particle-filter
                main.cpp
                map_reader.cpp
                motion_model.cpp
              )

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})

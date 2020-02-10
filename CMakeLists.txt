cmake_minimum_required(VERSION 3.10)
project(particle-filter)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(OpenMP)

add_executable(particle-filter
                main.cpp
                map_reader.cpp
                motion_model.cpp
                sensor_model.cpp
              )

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()


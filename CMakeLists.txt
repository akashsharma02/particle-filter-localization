cmake_minimum_required(VERSION 3.10)
project(particle-filter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(OpenMP)


add_executable(particle-filter
                main.cpp
                map_reader.cpp
                motion_model.cpp
                sensor_model.cpp
              )

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries( ${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()


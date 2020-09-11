include(${PROJECT_SOURCE_DIR}/cmake/CLI11.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/spdlog.cmake)
find_package(
  OpenCV REQUIRED
  COMPONENTS core imgproc highgui
)
find_package(OpenMP)


add_executable(
  ${PROJECT_NAME}
  main.cpp
  map_reader.cpp
  motion_model.cpp
  sensor_model.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  project_warnings
  spdlog
  CLI11
  opencv_core
  opencv_imgproc
  opencv_highgui
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

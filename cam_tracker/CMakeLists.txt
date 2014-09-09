cmake_minimum_required(VERSION 2.8)
project(cam_tracker)
set(CMAKE_BUILD_TYPE Debug)
find_package(OpenCV REQUIRED)
add_executable(calibrate_color_filter calibrate_color_filter.cpp)
target_link_libraries(calibrate_color_filter ${OpenCV_LIBS})


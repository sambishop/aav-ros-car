cmake_minimum_required(VERSION 2.8)
project(capture)
find_package(OpenCV REQUIRED)
add_executable(capture main.cpp)
target_link_libraries(capture ${OpenCV_LIBS})


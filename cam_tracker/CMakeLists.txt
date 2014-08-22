cmake_minimum_required(VERSION 2.8)
project(cam_tracker)
find_package(OpenCV REQUIRED)
add_executable(capture capture.cpp)
add_executable(manipulate manipulate.cpp)
target_link_libraries(capture ${OpenCV_LIBS})
target_link_libraries(manipulate ${OpenCV_LIBS})


cmake_minimum_required(VERSION 2.8)
project(cam_tracker)
set(CMAKE_BUILD_TYPE Debug)
find_package(OpenCV REQUIRED)
add_definitions(-D__STDC_CONSTANT_MACROS)

add_executable(calibrate_color_filter
    calibrate_color_filter.cpp
    util.h util.cpp)
target_link_libraries(calibrate_color_filter
    ${OpenCV_LIBS}
    -lavformat)

add_executable(track_car2
    track_car2.cpp
    util.h util.cpp)
target_link_libraries(track_car2
    ${OpenCV_LIBS})

add_executable(track_car
    track_car.cpp
    util.h util.cpp)
target_link_libraries(track_car
    ${OpenCV_LIBS})

add_executable(track_car3
    track_car3.cpp
    util.h util.cpp)
target_link_libraries(track_car3
    ${OpenCV_LIBS})

add_executable(measure_noise
    measure_noise.cpp
    util.h util.cpp)
target_link_libraries(measure_noise
    ${OpenCV_LIBS})


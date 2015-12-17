cmake_minimum_required(VERSION 2.8.3)
project(aav_quintic)

find_package(
  catkin REQUIRED
    aav_msgs
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
    aav_msgs
    roscpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_library(
  ${PROJECT_NAME}
    src/path_segment_calculator.cpp
    src/spline_segment_calculator.cpp
)

add_dependencies(
  ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)


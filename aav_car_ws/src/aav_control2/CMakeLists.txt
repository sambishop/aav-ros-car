cmake_minimum_required(VERSION 2.8.3)
project(aav_control2)

find_package(
  catkin REQUIRED
    nav_msgs
    roscpp
    std_msgs
)

catkin_package()

add_executable(
  aav_control2
    src/main.cpp
    src/QuinticControl.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  aav_control2
  ${catkin_LIBRARIES}
)


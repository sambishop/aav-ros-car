cmake_minimum_required(VERSION 2.8.3)
project(aav_control)

find_package(
  catkin REQUIRED
    aav_msgs
    ackermann_msgs
    actionlib
    cmake_modules
    nav_msgs
    roscpp
    roslaunch
)

find_package(
  GSL REQUIRED
)

catkin_package(
  CATKIN_DEPENDS
    aav_msgs
    ackermann_msgs
    roscpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_executable(
  aav_control
    src/DistanceCalculator.cpp
    src/QuinticControl.cpp
    src/main.cpp
)

add_dependencies(
  aav_control
    ${aav_msgs_EXPORTED_TARGETS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  aav_control
  ${catkin_LIBRARIES}
  ${GSL_LIBRARIES}
)

roslaunch_add_file_check(launch)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


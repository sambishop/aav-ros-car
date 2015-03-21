cmake_minimum_required(VERSION 2.8.3)
project(aav_control)

find_package(
  catkin REQUIRED
    aav_msgs
    ackermann_msgs
    actionlib
    actionlib_msgs
    cmake_modules
    genmsg
    nav_msgs
    roscpp
    roslaunch
)

find_package(
  GSL REQUIRED
)

add_action_files(
  DIRECTORY
    action
  FILES
    DoQuinticPath.action
)

generate_messages(
  DEPENDENCIES
    aav_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    aav_msgs
    ackermann_msgs
    actionlib_msgs
    roscpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_executable(
  aav_control
    src/DistanceCalculator.cpp
    src/QuinticControl.cpp
    src/main.cpp
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


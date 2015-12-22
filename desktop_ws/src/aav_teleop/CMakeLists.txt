cmake_minimum_required(VERSION 2.8.3)
project(aav_teleop)

find_package(
  catkin REQUIRED
    roslaunch
)

catkin_package(
  CATKIN_DEPENDS
    ackermann_msgs
)

roslaunch_add_file_check(launch)

install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


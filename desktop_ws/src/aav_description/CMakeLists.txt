cmake_minimum_required(VERSION 2.8.3)
project(aav_description)

find_package(
  catkin REQUIRED
    roslaunch
)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY
    launch
    urdf
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


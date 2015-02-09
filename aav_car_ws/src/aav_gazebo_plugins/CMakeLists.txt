cmake_minimum_required(VERSION 2.8.3)
project(aav_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  urdf
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
  INCLUDE_DIRS include
  DEPENDS gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(pwm_steer src/pwm_steer.cpp)
target_link_libraries(pwm_steer ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


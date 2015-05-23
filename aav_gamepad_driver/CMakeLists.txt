cmake_minimum_required(VERSION 2.8.3)
project(aav_gamepad_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ackermann_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
  INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aav_gamepad_driver_node src/aav_gamepad_driver_node.cpp)
add_library(joystick src/Joystick.cpp)
target_link_libraries(aav_gamepad_driver_node joystick ${catkin_LIBRARIES})


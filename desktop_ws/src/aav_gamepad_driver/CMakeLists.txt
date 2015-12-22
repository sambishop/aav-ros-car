cmake_minimum_required(VERSION 2.8.3)
project(aav_gamepad_driver)

find_package(
  catkin REQUIRED
    aav_msgs
    ackermann_msgs
    roscpp
)

catkin_package()

add_executable(
  ${PROJECT_NAME}
    src/Gamepad.cpp
    src/main.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  aav_gamepad_driver
  ${catkin_LIBRARIES}
)


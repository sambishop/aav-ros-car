cmake_minimum_required(VERSION 2.8.3)
project(aav_control)

find_package(
  catkin REQUIRED
    aav_msgs
    aav_quintic
    ackermann_msgs
    actionlib
    cmake_modules
    gazebo_msgs
    nav_msgs
    roscpp
    roslaunch
)

find_package(GSL REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_executable(
  ${PROJECT_NAME}
    src/cte_calculator.cpp
    src/gazebo_state_forwarder.cpp
    src/main.cpp
    src/pid.cpp
    src/quintic_control.cpp
)

add_dependencies(
  ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

include_directories(
  include
  ${aav_quintic_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
    ${aav_quintic_LIBRARIES}
    ${catkin_LIBRARIES}
    ${GSL_LIBRARIES}
)

catkin_add_gtest(
  cte_calculator_test
    src/cte_calculator.cpp
    test/cte_calculator_test.cpp
)

target_link_libraries(
  cte_calculator_test
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


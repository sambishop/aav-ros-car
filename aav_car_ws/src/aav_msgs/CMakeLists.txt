cmake_minimum_required(VERSION 2.8.3)
project(aav_msgs)

find_package(
  catkin REQUIRED
    message_generation
    roscpp
    std_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    QuinticPath.msg
    QuinticSegment.msg
    PlanningPoint.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)


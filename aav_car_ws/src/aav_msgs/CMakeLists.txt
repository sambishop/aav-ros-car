cmake_minimum_required(VERSION 2.8.3)
project(aav_msgs)

find_package(
  catkin REQUIRED
    actionlib_msgs
    genmsg
    message_generation
    roscpp
    std_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    PlanningPoint.msg
    QuinticPath.msg
    QuinticPathSegment.msg
    QuinticSplineSegment.msg
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
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
    std_msgs
)


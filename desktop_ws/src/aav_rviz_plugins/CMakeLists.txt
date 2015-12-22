cmake_minimum_required(VERSION 2.8.3)
project(aav_rviz_plugins)

find_package(
  catkin REQUIRED
    aav_msgs
    aav_quintic
    roscpp
    rviz
)

find_package(
  Qt4 REQUIRED
    QtCore
    QtGui
)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

include(${QT_USE_FILE})

include_directories(
  include
  ${aav_quintic_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(
  HEADERS
    include/${PROJECT_NAME}/quintic_path_display.h
)

set(
  SOURCES
    src/quintic_path_display.cpp
)

qt4_wrap_cpp(
  MOC_HEADERS
    ${HEADERS}
)

add_library(
  ${PROJECT_NAME}
    ${MOC_HEADERS}
    ${SOURCES}
)

target_link_libraries(
  ${PROJECT_NAME}
    ${aav_quintic_LIBRARIES}
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    ${OGRE_OV_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


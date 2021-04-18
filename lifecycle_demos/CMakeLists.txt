cmake_minimum_required(VERSION 3.0.2)
project(lifecycle_demos)

find_package(catkin REQUIRED COMPONENTS
  lifecycle
  roscpp
  roslint
  rospy
)

roslint_python()

catkin_package(
  # CATKIN_DEPENDS lifecycle roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/lm_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

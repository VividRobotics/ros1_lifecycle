cmake_minimum_required(VERSION 3.0.2)
project(lifecycle_demos)

find_package(catkin REQUIRED COMPONENTS
  lifecycle
  roscpp
  roslint
  rospy
)

set(CMAKE_CXX_STANDARD 17)

set(ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references"
)
roslint_cpp()

set(ROSLINT_PYTHON_OPTS "1>&2")
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

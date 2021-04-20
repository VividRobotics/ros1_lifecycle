cmake_minimum_required(VERSION 3.0.2)
project(lifecycle_demos)

find_package(catkin REQUIRED COMPONENTS
  lifecycle
  nodelet
  roscpp
  roslint
  rospy
  std_msgs
)

set(ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references"
)
roslint_cpp()
roslint_python()

catkin_package(
  CATKIN_DEPENDS lifecycle nodelet roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/lifecycle_demo.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(PROGRAMS
  scripts/lm_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES
  nodelet_${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

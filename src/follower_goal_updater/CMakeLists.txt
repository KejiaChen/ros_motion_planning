cmake_minimum_required(VERSION 3.0.2)
project(follower_goal_updater)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp tf2 tf2_geometry_msgs geometry_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(follower_goal_updater_node src/follower_goal_updater_node.cpp src/follower_goal_updater.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(follower_goal_updater_node
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS follower_goal_updater_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install include files (if any)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


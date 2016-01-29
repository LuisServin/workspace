cmake_minimum_required(VERSION 2.8.3)
project(atwork_launch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES atwork_launch
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
cmake_minimum_required(VERSION 2.8.3)
project(stella_3d_navigation_pc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stella_3d_slam
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

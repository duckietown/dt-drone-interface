cmake_minimum_required(VERSION 2.8.3)
project(pid_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  duckietown_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
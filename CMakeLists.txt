cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_trajectory_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES GoToGoal.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs geometry_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs rospy actionlib actionlib_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

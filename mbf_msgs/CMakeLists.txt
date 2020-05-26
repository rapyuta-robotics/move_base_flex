cmake_minimum_required(VERSION 2.8.3)
project(mbf_msgs)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib_msgs
  genmsg
  geometry_msgs
  forklift_interfaces
  message_generation
  message_runtime
  nav_msgs
  std_msgs
  rr_service
)

add_service_files(
  DIRECTORY
  srv
  FILES
  CheckPoint.srv
  CheckPose.srv
  CheckPath.srv
)

add_action_files(
  DIRECTORY
  action
  FILES
  GetPath.action
  ExePath.action
  Recovery.action
  MoveBase.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  forklift_interfaces
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  forklift_interfaces
  message_runtime
  nav_msgs
  std_msgs
)

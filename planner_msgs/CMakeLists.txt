cmake_minimum_required(VERSION 2.8.3)
project(planner_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  NavigationStatus.msg
  TerrainInfo.msg
)

add_service_files(
  FILES
  SetString.srv
  SetVector3.srv
  SetService.srv
  SetPlannerState.srv
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)

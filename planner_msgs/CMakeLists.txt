cmake_minimum_required(VERSION 3.14.4)
project(planner_msgs)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/NavigationStatus.msg
  msg/TerrainInfo.msg
  srv/SetPlannerState.srv
  srv/SetService.srv
  srv/SetString.srv
  srv/SetVector3.srv
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

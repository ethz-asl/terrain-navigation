cmake_minimum_required(VERSION 2.8.12)
project(terrain_navigation_ros)
add_definitions(-std=c++17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(ompl REQUIRED)
find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/HeightRateTuning.cfg
)
find_package(Boost REQUIRED COMPONENTS serialization system filesystem)
find_package(GeographicLib REQUIRED)
include(CheckGeographicLibDatasets)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  grid_map_core
  grid_map_cv
  grid_map_msgs
  planner_msgs
  grid_map_ros
  grid_map_pcl
  grid_map_geo
  interactive_markers
  terrain_navigation
  terrain_planner
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES terrain_navigation_ros
  CATKIN_DEPENDS roscpp rospy dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/terrain_planner.cpp
  src/geo_conversions.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(terrain_planner_node
  src/terrain_planner_node.cpp
)
add_dependencies(terrain_planner_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(terrain_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

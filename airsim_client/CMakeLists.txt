cmake_minimum_required(VERSION 2.8.12)
project(airsim_client)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  grid_map_core
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  grid_map_pcl
  grid_map_geo
  adaptive_viewutility
  terrain_navigation
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy adaptive_viewutility
)

#############
# LIBRARIES #
#############

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

set(AIRSIM_ROOT "/home/jaeyoung/dev/AirSim")

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/include
    ${AIRSIM_ROOT}/AirLib/include
    ${AIRSIM_ROOT}/AirLib/deps/MavLinkCom/include
)

add_library(${PROJECT_NAME}
  src/airsim_client.cpp
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${AIRSIM_ROOT}/AirLib/lib/x64/Release
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(test_airsim_node
  src/test_airsim_node.cpp
)
add_dependencies(test_airsim_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_airsim_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread AirLib rpc MavLinkCom)

add_executable(test_planner_airsim_node
  src/test_planner_airsim_node.cpp
)
add_dependencies(test_planner_airsim_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_planner_airsim_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread AirLib rpc MavLinkCom)

add_executable(test_random_airsim_node
  src/test_random_airsim_node.cpp
)
add_dependencies(test_random_airsim_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_random_airsim_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread AirLib rpc MavLinkCom)


add_executable(test_gridplanner_node
  src/test_gridplanner_node.cpp
)
add_dependencies(test_gridplanner_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_gridplanner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread AirLib rpc MavLinkCom)


add_executable(get_airsim_position_node
  src/get_airsim_position_node.cpp
)
add_dependencies(get_airsim_position_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_airsim_position_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread AirLib rpc MavLinkCom)

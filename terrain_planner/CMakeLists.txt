cmake_minimum_required(VERSION 2.8.12)
project(terrain_planner)

add_definitions(-std=c++11 -Wall)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(GDAL)
find_package(OpenCV REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(ompl REQUIRED)
include(CheckGeographicLibDatasets)

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
  interactive_markers
  terrain_navigation
  adaptive_viewutility
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES terrain_planner
  CATKIN_DEPENDS roscpp rospy
  DEPENDS GeographicLib
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/maneuver_library.cpp
  src/terrain_planner.cpp
  src/mcts_planner.cpp
  src/primitive_planner.cpp
  src/terrain_ompl.cpp
  src/terrain_ompl_rrt.cpp
  src/DubinsPath.cpp
  src/DubinsAirplane2.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARIES} ${OMPL_LIBRARIES})

############
# BINARIES #
############
add_executable(terrain_planner_node
  src/terrain_planner_node.cpp
)
add_dependencies(terrain_planner_node terrain_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(terrain_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_distance_surface
  src/test_distance_surface.cpp
)
add_dependencies(test_distance_surface terrain_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_distance_surface ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_dubins_planner
  src/test_dubins_planner.cpp
)
add_dependencies(test_dubins_planner terrain_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_dubins_planner ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_mcts
  src/test_mcts.cpp
)
add_dependencies(test_mcts terrain_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_mcts ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_mcts_mapping
  src/test_mcts_mapping.cpp
)
add_dependencies(test_mcts_mapping terrain_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_mcts_mapping ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_rrt_node
  src/test_rrt_node.cpp
)
add_dependencies(test_rrt_node terrain_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_rrt_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})


##########
# EXPORT #
##########
# cs_install()
# cs_export()
#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
    # Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test 
      test/main.cpp
      test/test_mcts_planner.cpp)

    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                 ${catkin_LIBRARIES}
                                                 ${YAML_CPP_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})
    endif()

endif()

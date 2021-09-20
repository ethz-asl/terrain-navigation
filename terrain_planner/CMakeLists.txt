cmake_minimum_required(VERSION 2.8.12)
project(terrain_planner)

add_definitions(-std=c++11)

find_package(GDAL)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  grid_map_core
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  grid_map_pcl
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES terrain_planner
  CATKIN_DEPENDS roscpp rospy
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

add_library(${PROJECT_NAME}
  src/profiler.cpp
  src/maneuver_library.cpp
  src/terrain_planner.cpp
  src/terrain_map.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

############
# BINARIES #
############
add_executable(terrain_planner_node
  src/terrain_planner_node.cpp
)
add_dependencies(terrain_planner_node terrain_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(terrain_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})


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
    catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
                                          test/test_example.cpp)

    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                 ${catkin_LIBRARIES}
                                                 ${YAML_CPP_LIBRARIES})
    endif()

endif()

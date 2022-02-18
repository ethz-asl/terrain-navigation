cmake_minimum_required(VERSION 2.8.12)
project(adaptive_viewutility)

add_definitions(-std=c++14)

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
  terrain_navigation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adaptive_viewutility
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
  ${GDAL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/viewutility_map.cpp
  src/viewplanner.cpp
  src/performance_tracker.cpp
  src/adaptive_viewutility.cpp
  src/dubins_planner.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

############
# BINARIES #
############
add_executable(test_viewutility_node
  src/test_viewutility_node.cpp
)
add_dependencies(test_viewutility_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_viewutility_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_visibility_node
  src/test_visibility_node.cpp
)
add_dependencies(test_visibility_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_visibility_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_grid_node
  src/test_grid_node.cpp
)
add_dependencies(test_grid_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_grid_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_mesh_node
  src/test_mesh_node.cpp
)
add_dependencies(test_mesh_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_mesh_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_utility_map_node
  src/test_utility_map_node.cpp
)
add_dependencies(test_utility_map_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_utility_map_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(test_planner_node
  src/test_planner_node.cpp
)
add_dependencies(test_planner_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

add_executable(test_random_node
  src/test_random_node.cpp
)
add_dependencies(test_random_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_random_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

add_executable(test_normals_node
  src/test_normals_node.cpp
)
add_dependencies(test_normals_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_normals_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

add_executable(run_meshtrimmer_node
  src/run_meshtrimmer_node.cpp
)
add_dependencies(run_meshtrimmer_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(run_meshtrimmer_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

add_executable(compare_mesh_node
  src/compare_mesh_node.cpp
)
add_dependencies(compare_mesh_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(compare_mesh_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

add_executable(test_gridmap_loader
  src/test_gridmap_loader.cpp
)
add_dependencies(test_gridmap_loader adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_gridmap_loader ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

add_executable(evaluate_progress_node
  src/evaluate_progress_node.cpp
)
add_dependencies(evaluate_progress_node adaptive_viewutility ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(evaluate_progress_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

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

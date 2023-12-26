cmake_minimum_required(VERSION 2.8.12)
project(terrain_planner_benchmark)

add_definitions(-std=c++17 -Wall)

find_package(GDAL)
find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system filesystem)

find_package(catkin REQUIRED COMPONENTS
    eigen_catkin
    grid_map_core
    grid_map_cv
    grid_map_msgs
    grid_map_ros
    grid_map_pcl
    grid_map_geo
    terrain_navigation
    terrain_planner
    terrain_navigation_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES terrain_planner_benchmark
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/terrain_planner_benchmark.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(terrain_planner_benchmark_node
  src/terrain_planner_benchmark_node.cpp
)
add_dependencies(terrain_planner_benchmark_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(terrain_planner_benchmark_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

add_executable(ompl_benchmark_node
  src/ompl_benchmark_node.cpp
)
add_dependencies(ompl_benchmark_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(ompl_benchmark_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} pthread)

add_executable(test_ics
  src/test_ics.cpp
)
add_dependencies(test_ics ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_ics ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(test_rrt_replanning_node
  src/test_rrt_replanning_node.cpp
)
add_dependencies(test_rrt_replanning_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_rrt_replanning_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(surface_visualization
  src/surface_visualization.cpp
)
add_dependencies(surface_visualization ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(surface_visualization ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(test_dubins_classification
  src/test_dubins_classification.cpp
)
add_dependencies(test_dubins_classification ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_dubins_classification ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(run_replay
  src/rviz_replay.cpp
)
add_dependencies(run_replay ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(run_replay ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(test_rrt_circle_goal
  src/test_rrt_circle_goal.cpp
)
add_dependencies(test_rrt_circle_goal terrain_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_rrt_circle_goal ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(test_rrt_node
  src/test_rrt_node.cpp
)
add_dependencies(test_rrt_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_rrt_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(test_ompl_dubins
  src/test_ompl_dubins.cpp
)
add_dependencies(test_ompl_dubins ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_ompl_dubins ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(test_ompl_dubins_to_circle
  src/test_ompl_dubins_to_circle.cpp
)
add_dependencies(test_ompl_dubins_to_circle ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_ompl_dubins_to_circle ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(test_rrt_hybrid_vtol
  src/test_rrt_hybrid_vtol.cpp
)
add_dependencies(test_rrt_hybrid_vtol ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_rrt_hybrid_vtol ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

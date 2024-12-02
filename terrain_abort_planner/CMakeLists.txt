cmake_minimum_required(VERSION 2.8.12)
project(terrain_abort_planner)

add_definitions(-std=c++17 -Wall)

find_package(GDAL)
find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system filesystem)

include(cmake/CPM.cmake)
CPMAddPackage("gh:nlohmann/json@3.11.3")

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
  # LIBRARIES terrain_planner_benchmark
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
  src/mission_io.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARIES} ${OMPL_LIBRARIES})


add_executable(run_dynamic_rallypoints
  src/run_dynamic_rallypoints.cpp
)
add_dependencies(run_dynamic_rallypoints ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_dynamic_rallypoints ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES} nlohmann_json::nlohmann_json)

if(CATKIN_ENABLE_TESTING)
    # Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
                                          test/test_mission_io.cpp)
                                          # test/test_terrain_map.cpp)

    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                 ${catkin_LIBRARIES}
                                                 ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES})
    endif()

endif()

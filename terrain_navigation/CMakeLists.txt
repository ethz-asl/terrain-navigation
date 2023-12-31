cmake_minimum_required(VERSION 2.8.12)
project(terrain_navigation)
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    eigen_catkin
    grid_map_core
    grid_map_geo
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES terrain_navigation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/data_logger.cpp
  src/profiler.cpp
  src/terrain_map.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    # Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
                                          test/test_trajectory.cpp)
                                          # test/test_terrain_map.cpp)

    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                 ${catkin_LIBRARIES}
                                                 ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES})
    endif()

endif()

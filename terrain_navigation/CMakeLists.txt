cmake_minimum_required(VERSION 2.8.12)
project(terrain_navigation)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    eigen_catkin
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES terrain_navigation
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/profiler.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARIES})

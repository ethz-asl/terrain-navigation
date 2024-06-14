cmake_minimum_required(VERSION 2.8.12)
project(terrain_planner)

add_definitions(-std=c++17 -Wall)

find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system filesystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  grid_map_core
  grid_map_geo
  terrain_navigation
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
  ${OMPL_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/terrain_ompl.cpp
  src/terrain_ompl_rrt.cpp
  src/terrain_mm_prm.cpp
  src/DubinsPath.cpp
  src/DubinsAirplane.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

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
      test/main.cpp)

    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                 ${catkin_LIBRARIES}
                                                 ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES})
    endif()

endif()

cmake_minimum_required(VERSION 3.14.4)
project(terrain_navigation_ros)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set policy for 3.16 for CMP0076 defaulting to ON
cmake_policy(VERSION 3.16)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
#   path/to/file.cmake"
#   other/pathto/file.cmake"
# )

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_geo REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
# find_package(grid_map_pcl REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(ompl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(terrain_navigation REQUIRED)
find_package(terrain_planner REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS serialization system filesystem)
find_package(Eigen3 REQUIRED)

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
  # Workaround for the supplied find script not being on CMake path in Ubuntu.
  # https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173
  if(IS_DIRECTORY /usr/share/cmake/geographiclib)
    list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
  endif()
endif()

find_package(GeographicLib REQUIRED)
# include(CheckGeographicLibDatasets)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ODE REQUIRED ode)

add_library(${PROJECT_NAME}
  src/geo_conversions.cpp
  src/terrain_planner.cpp
  src/visualization.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  tf2_eigen::tf2_eigen
  ${geographic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${grid_map_msgs_TARGETS}
  ${mavros_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${planner_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  grid_map_core
  grid_map_ros
  terrain_navigation
  terrain_planner
)

add_executable(terrain_planner_node
  src/terrain_planner_node.cpp
)

target_link_libraries(terrain_planner_node PUBLIC
  ${PROJECT_NAME}
  ${ODE_LIBRARIES}
)

target_link_directories(terrain_planner_node PUBLIC
  ${ODE_LIBRARY_DIRS}
  ${yaml_cpp_vendor_LIBRARY_DIRS}
)

# Install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  TARGETS
  terrain_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  resources
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

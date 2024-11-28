cmake_minimum_required(VERSION 3.14.4)
project(terrain_planner_benchmark)

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

find_package(ament_cmake REQUIRED)
find_package(GDAL 3.5)
find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)

find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_geo REQUIRED)
find_package(terrain_navigation REQUIRED)
find_package(Boost REQUIRED) # Workaround for terrain_planner not exporting this correctly
find_package(terrain_planner REQUIRED)

add_library(${PROJECT_NAME})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

add_executable(terrain_planner_benchmark_node)
add_executable(ompl_benchmark_node EXCLUDE_FROM_ALL)
add_executable(test_ics EXCLUDE_FROM_ALL)
add_executable(test_rrt_replanning_node EXCLUDE_FROM_ALL)
add_executable(surface_visualization EXCLUDE_FROM_ALL)
add_executable(test_dubins_classification EXCLUDE_FROM_ALL)
add_executable(run_replay EXCLUDE_FROM_ALL)
add_executable(test_rrt_circle_goal EXCLUDE_FROM_ALL)
add_subdirectory(src)

include(GNUInstallDirs)
install(
  TARGETS
    terrain_planner_benchmark_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  resource
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

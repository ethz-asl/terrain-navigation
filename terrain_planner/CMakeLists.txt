cmake_minimum_required(VERSION 3.14.4)
project(terrain_planner)

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
find_package(grid_map_core REQUIRED)
find_package(grid_map_geo REQUIRED)
find_package(grid_map_cv REQUIRED)
# find_package(grid_map_pcl REQUIRED)
find_package(grid_map_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(terrain_navigation REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS serialization system filesystem)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ODE REQUIRED ode)

# Libraries
add_library(${PROJECT_NAME}
  src/common.cpp
  src/DubinsAirplane.cpp
  src/DubinsPath.cpp
  src/terrain_ompl_rrt.cpp
  src/terrain_ompl.cpp
  src/visualization.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${OMPL_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${tf2_geometry_msgs_TARGETS}
  tf2_eigen::tf2_eigen
)

target_compile_options(${PROJECT_NAME} PRIVATE
  "-Wno-gnu-zero-variadic-macro-arguments"
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  grid_map_core
  grid_map_geo
  terrain_navigation
)

# Executables
add_executable(test_rrt_node
  src/test_rrt_node.cpp
)

target_compile_options(test_rrt_node PRIVATE
  "-Wno-gnu-zero-variadic-macro-arguments"
)

target_link_directories(test_rrt_node PUBLIC
  ${ODE_LIBRARY_DIRS}
  ${yaml_cpp_vendor_LIBRARY_DIRS}
)

target_link_libraries(test_rrt_node PUBLIC
  ${PROJECT_NAME}
  ${ODE_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  tf2_eigen::tf2_eigen
)

add_executable(test_ompl_dubins
  src/test_ompl_dubins.cpp
)

target_compile_options(test_ompl_dubins PRIVATE
  "-Wno-gnu-zero-variadic-macro-arguments"
)

target_link_directories(test_ompl_dubins PUBLIC
  ${ODE_LIBRARY_DIRS}
  ${yaml_cpp_vendor_LIBRARY_DIRS}
)

target_link_libraries(test_ompl_dubins PUBLIC
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${ODE_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  tf2_eigen::tf2_eigen
)

add_executable(test_ompl_dubins_to_circle
  src/test_ompl_dubins_to_circle.cpp
)

target_compile_options(test_ompl_dubins_to_circle PRIVATE
  "-Wno-gnu-zero-variadic-macro-arguments"
)

target_link_directories(test_ompl_dubins_to_circle PUBLIC
  ${ODE_LIBRARY_DIRS}
  ${yaml_cpp_vendor_LIBRARY_DIRS}
)

target_link_libraries(test_ompl_dubins_to_circle PUBLIC
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${ODE_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  tf2_eigen::tf2_eigen
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
  test_rrt_node
  test_ompl_dubins
  test_ompl_dubins_to_circle
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test
    test/main.cpp
  )

  target_link_directories(${PROJECT_NAME}-test PUBLIC
    ${ODE_LIBRARY_DIRS}
    ${yaml_cpp_vendor_LIBRARY_DIRS}
  )

  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${ODE_LIBRARIES}
  )
endif()

ament_package()

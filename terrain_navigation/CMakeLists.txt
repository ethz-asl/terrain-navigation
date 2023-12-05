cmake_minimum_required(VERSION 3.14.4)
project(terrain_navigation)

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

find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(action_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(GDAL 3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(yaml_cpp_vendor REQUIRED)
link_directories(${yaml_cpp_vendor_LIBRARY_DIRS})

# Reverse compatability for GDAL<3.5
# https://gdal.org/development/cmake.html
if(NOT TARGET GDAL::GDAL)
  add_library(GDAL::GDAL ALIAS ${GDAL_LIBRARY})
endif()

add_library(${PROJECT_NAME}
  src/data_logger.cpp
  src/profiler.cpp
  src/terrain_map.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  GDAL::GDAL
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  grid_map_core
  grid_map_geo
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
ament_export_dependencies(GDAL)

# Test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test
    test/main.cpp
    test/test_trajectory.cpp
    # test/test_terrain_map.cpp
  )

  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
  )
endif()

ament_package()

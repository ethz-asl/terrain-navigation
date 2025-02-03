/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim, Autonomous Systems Lab,
 *  ETH Zürich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <terrain_navigation/terrain_map.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/point.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner/visualization.h"
#include "terrain_planner_benchmark/terrain_planner_benchmark.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ompl_rrt_planner");

  // Initialize ROS related publishers for visualization
  auto start_pos_pub =
      node->create_publisher<visualization_msgs::msg::Marker>("start_position", rclcpp::QoS(1).transient_local());
  auto goal_pos_pub =
      node->create_publisher<visualization_msgs::msg::Marker>("goal_position", rclcpp::QoS(1).transient_local());
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(1).transient_local());
  auto interpolate_path_pub =
      node->create_publisher<nav_msgs::msg::Path>("interpolated_path", rclcpp::QoS(1).transient_local());
  auto path_segment_pub =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("path_segments", rclcpp::QoS(1).transient_local());
  auto grid_map_pub = node->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", rclcpp::QoS(1).transient_local());
  auto trajectory_pub =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("tree", rclcpp::QoS(1).transient_local());

  auto const map_path = node->declare_parameter("map_path", "");
  auto const location = node->declare_parameter("location", "");
  auto const color_file_path = node->declare_parameter("color_file_path", "");
  auto const output_file_dir = node->declare_parameter("output_directory", "");
  auto const number_of_runs = node->declare_parameter("number_of_runs", 10);

  // Load terrain map from defined tif paths
  auto terrain_map = std::make_shared<TerrainMap>();
  if (!terrain_map->initializeFromGeotiff(map_path)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Unable to load geotiff from '" << map_path << "'!");
    return 1;
  }
  if (!color_file_path.empty()) {  // Load color layer if the color path is nonempty
    terrain_map->addColorFromGeotiff(color_file_path);
  }
  terrain_map->AddLayerDistanceTransform(50.0, "distance_surface");
  terrain_map->AddLayerDistanceTransform(120.0, "max_elevation");
  double radius = 66.667;
  terrain_map->AddLayerHorizontalDistanceTransform(radius, "ics_+", "distance_surface");
  terrain_map->AddLayerHorizontalDistanceTransform(-radius, "ics_-", "max_elevation");

  auto benchmark = std::make_shared<TerrainPlannerBenchmark>();

  benchmark->setMap(terrain_map);

  /// TODO: Configure benchmarking options

  benchmark->runBenchmark(number_of_runs);

  /// TODO: write benchmark results to file
  std::string output_file_path = output_file_dir + "/" + location + "_goal_benchmark.csv";
  benchmark->writeResultstoFile(output_file_path);
  return 0;
}

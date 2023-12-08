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

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <terrain_navigation/terrain_map.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner/visualization.h"
#include "terrain_planner_benchmark/terrain_planner_benchmark.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ompl_rrt_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Initialize ROS related publishers for visualization
  auto start_pos_pub = nh.advertise<visualization_msgs::Marker>("start_position", 1, true);
  auto goal_pos_pub = nh.advertise<visualization_msgs::Marker>("goal_position", 1, true);
  auto path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  auto interpolate_path_pub = nh.advertise<nav_msgs::Path>("interpolated_path", 1, true);
  auto path_segment_pub = nh.advertise<visualization_msgs::MarkerArray>("path_segments", 1, true);
  auto grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  auto trajectory_pub = nh.advertise<visualization_msgs::MarkerArray>("tree", 1, true);
  std::string map_path, color_file_path, output_file_dir, location;
  int number_of_runs;
  nh_private.param<std::string>("map_path", map_path, "");
  nh_private.param<std::string>("location", location, "");
  nh_private.param<std::string>("color_file_path", color_file_path, "");
  nh_private.param<std::string>("output_directory", output_file_dir, "");
  nh_private.param<int>("number_of_runs", number_of_runs, 10);

  // Load terrain map from defined tif paths
  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->initializeFromGeotiff(map_path, false);
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

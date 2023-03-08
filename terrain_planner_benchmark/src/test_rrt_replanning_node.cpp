/****************************************************************************
 *
 *   Copyright (c) 2021-2022 Jaeyoung Lim. All rights reserved.
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
 * 3. Neither the name terrain-navigation nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 * @brief ROS Node to test ompl
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

#include "terrain_navigation/data_logger.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner/visualization.h"

#include <any>
#include <chrono>

void writeToFile(const std::string path, std::vector<Eigen::Vector3d> solution_path) {
  // Write data to files
  std::cout << "[DataLogger] Writing data to file! " << path << std::endl;

  std::string field_seperator{","};
  std::ofstream output_file;
  output_file.open(path, std::ios::trunc);
  std::vector<std::string> keys{"x", "y", "z"};
  for (auto key : keys) {
    output_file << key << field_seperator;
  }
  output_file << "\n";

  for (auto& data : solution_path) {
    output_file << data.x() << field_seperator << data.y() << field_seperator << data.z() << field_seperator;
    output_file << "\n";
  }

  output_file.close();
  return;
}

void publishPathSegments(ros::Publisher& pub, TrajectorySegments& trajectory) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub.publish(msg);

  std::vector<visualization_msgs::Marker> segment_markers;
  int i = 0;
  double max_altitude = -std::numeric_limits<double>::infinity();
  double min_altitude = std::numeric_limits<double>::infinity();

  for (auto& segment : trajectory.segments) {
    for (auto& position : segment.position()) {
      if (position(2) > max_altitude) {
        max_altitude = position(2);
      }
      if (position(2) < min_altitude) {
        min_altitude = position(2);
      }
    }
  }

  for (auto& segment : trajectory.segments) {
    Eigen::Vector3d color = Eigen::Vector3d(0.0, 1.0, 0.0);
    segment_markers.insert(segment_markers.begin(), trajectory2MarkerMsg(segment, i++, min_altitude, max_altitude));
    segment_markers.insert(segment_markers.begin(),
                           vector2ArrowsMsg(segment.position().front(), 5.0 * segment.velocity().front(), i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().back(), i++, color));
  }
  msg.markers = segment_markers;
  pub.publish(msg);
}

void publishCircleSetpoints(const ros::Publisher& pub, const Eigen::Vector3d& position, const double radius, Eigen::Vector3d color=Eigen::Vector3d(0.0, 1.0, 0.0)) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.header.stamp = ros::Time::now();
  std::vector<geometry_msgs::Point> points;
  for (double t = 0.0; t <= 1.0; t += 0.02) {
    geometry_msgs::Point point;
    point.x = position.x() + radius * std::cos(t * 2 * M_PI);
    point.y = position.y() + radius * std::sin(t * 2 * M_PI);
    point.z = position.z();
    points.push_back(point);
  }
  geometry_msgs::Point start_point;
  start_point.x = position.x() + radius * std::cos(0.0);
  start_point.y = position.y() + radius * std::sin(0.0);
  start_point.z = position.z();
  points.push_back(start_point);

  marker.points = points;
  marker.scale.x = 10.0;
  marker.scale.y = 10.0;
  marker.scale.z = 10.0;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  pub.publish(marker);
}

void addErrorLayer(const std::string layer_name, const std::string query_layer, const std::string reference_layer,
                   grid_map::GridMap& map) {
  map.add(layer_name);

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    map.at(layer_name, index) = map.at(query_layer, index) > map.at(reference_layer, index);
  }
}

Eigen::Vector3d getColor(double intensity) {
    const std::vector<std::vector<float>> &ctable = colorMap.at("plasma");

    int idx = int(floor(intensity * 255));
    idx = std::min(idx, 255);
    idx = std::max(idx, 0);

    // Get color from table
    std::vector<float> rgb = ctable.at(idx);
    return Eigen::Vector3d(rgb[0], rgb[1], rgb[2]);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ompl_rrt_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Initialize ROS related publishers for visualization
  auto start_pos_pub = nh.advertise<visualization_msgs::Marker>("start_position", 1, true);
  auto goal_pos_pub = nh.advertise<visualization_msgs::Marker>("goal_position", 1, true);
  auto path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  auto path_segment_pub = nh.advertise<visualization_msgs::MarkerArray>("path_segments", 1, true);
  auto grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  auto trajectory_pub = nh.advertise<visualization_msgs::MarkerArray>("tree", 1, true);

  std::string map_path, color_file_path, location, output_directory;
  nh_private.param<std::string>("location", location, "");
  nh_private.param<std::string>("map_path", map_path, "");
  nh_private.param<std::string>("color_file_path", color_file_path, "");
  nh_private.param<std::string>("output_directory", output_directory, "");

  auto data_logger = std::make_shared<DataLogger>();
  data_logger->setKeys({"time", "path_length"});

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
  terrain_map->getGridMap().add("offset");
  terrain_map->getGridMap()["offset"].setConstant(1000.0);
  addErrorLayer("error", "ics_-", "ics_+", terrain_map->getGridMap());
  // Initialize planner with loaded terrain map
  auto planner = std::make_shared<TerrainOmplRrt>();
  planner->setMap(terrain_map);
  planner->setAltitudeLimits(120.0, 50.0);
  /// TODO: Get bounds from gridmap
  planner->setBoundsFromMap(terrain_map->getGridMap());

  const Eigen::Vector2d map_pos = terrain_map->getGridMap().getPosition();
  const double map_width_x = terrain_map->getGridMap().getLength().x();
  const double map_width_y = terrain_map->getGridMap().getLength().y();

  Eigen::Vector3d start{Eigen::Vector3d(map_pos(0) - 0.35 * map_width_x, map_pos(1) + 0.35 * map_width_y, 0.0)};
  Eigen::Vector3d updated_start;
  if (validatePosition(terrain_map->getGridMap(), start, updated_start)) {
    start = updated_start;
    std::cout << "Specified start position is valid" << std::endl;
  } else {
    throw std::runtime_error("Specified start position is NOT valid");
  }
  Eigen::Vector3d goal{Eigen::Vector3d(map_pos(0) + 0.35 * map_width_x, map_pos(1) - 0.35 * map_width_y, 0.0)};
  Eigen::Vector3d updated_goal;
  if (validatePosition(terrain_map->getGridMap(), goal, updated_goal)) {
    goal = updated_goal;
    std::cout << "Specified goal position is valid" << std::endl;
  } else {
    throw std::runtime_error("Specified goal position is NOT valid");
  }

  // Repeatedly publish results
  planner->setupProblem(start, goal);
  TrajectorySegments reference_primitive_;
  bool new_problem = true;

  int i = 0;

  auto start_time = std::chrono::steady_clock::now();

  double simulation_time{0.0};
  double max_simulation_time{500.0};

  while (simulation_time < max_simulation_time) {
    /// TODO: Time budget based on segment length
    if (reference_primitive_.segments.empty()) {
      // Does not have an initial problem yet
      new_problem = true;
    } else {
      // Check if vehicle traversed the environment
    }

    double traverse_time = 1.0;

    TrajectorySegments planner_solution_path;
    bool found_solution = planner->Solve(traverse_time, planner_solution_path);

    if (found_solution) {
      bool update_solution = false;

      if (new_problem) {
        new_problem = false;  // Since a solution is found, it is not a new problem anymore
        update_solution = true;
      } else {  /// Check if the found solution is a better solution
        /// Get length of the new planner solution path
        double new_solution_path_length = planner_solution_path.getLength();

        /// Get length of the left solution path of the current path segment
        Eigen::Vector3d vehicle_position;
        const int current_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position);
        double current_solution_path_length = reference_primitive_.getLength();
        /// Compare path length between the two path lengths
        update_solution = bool(new_solution_path_length < current_solution_path_length);
        if (update_solution) {
          std::cout << "-------------------" << std::endl;
          std::cout << "  - new_solution_path_length: " << new_solution_path_length << std::endl;
          std::cout << "    - current_solution_path_length: " << current_solution_path_length << std::endl;
          std::cout << "  - Found better solution: " << update_solution << std::endl;
        }
      }

      auto end = std::chrono::steady_clock::now();
      simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time).count() / 1000.0;

      // If a better solution is found, update the path
      if (update_solution) {
        update_solution = false;
        /// TODO: Get segment start velocity and start position
        reference_primitive_ = planner_solution_path;
        /// TODO: Write solution path to file
        // Eigen::Vector3d segment_start;
        // Eigen::Vector3d segment_start_vel;
        // planner->setupProblem(segment_start, segment_start_vel, goal);
        std::string file_path = output_directory + "/solution_" + std::to_string(i) + ".csv";
        writeToFile(file_path, reference_primitive_.position());
        std::unordered_map<std::string, std::any> state;
        state.insert(std::pair<std::string, double>("time", simulation_time));
        state.insert(std::pair<std::string, double>("path_length", reference_primitive_.getLength()));
        data_logger->record(state);
        i++;
      }
    }

    terrain_map->getGridMap().setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), message);
    grid_map_pub.publish(message);
    publishTrajectory(path_pub, reference_primitive_.position());
    publishPathSegments(path_segment_pub, reference_primitive_);
    std::cout << " - start pos: " << start.transpose() << std::endl;
    std::cout << " - goal pos: " << goal.transpose() << std::endl;
    double min_altitude = 1964.41;
    double max_altitude = 2307.01;


    publishCircleSetpoints(start_pos_pub, start, radius, getColor(1.0));
    publishCircleSetpoints(goal_pos_pub, goal, radius, getColor(0.0));
    publishTree(trajectory_pub, planner->getPlannerData(), planner->getProblemSetup());
    ros::Duration(1.0).sleep();
  }
  data_logger->setPrintHeader(true);
  std::string output_file_path = output_directory + "/" + location + "_replanning.csv";
  data_logger->writeToFile(output_file_path);

  ros::spin();
  return 0;
}

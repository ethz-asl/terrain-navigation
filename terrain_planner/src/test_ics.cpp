/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim, All rights reserved.
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
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "terrain_planner/maneuver_library.h"

#include <terrain_navigation/terrain_map.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

void addMaxICSLayer(const std::string layer_name, grid_map::GridMap& map, const double yaw_rate) {
  /// Choose yaw state to calculate ICS state

  auto manuever_library = std::make_shared<ManeuverLibrary>();

  Eigen::Vector3d rate = Eigen::Vector3d(0.0, 0.0, yaw_rate);
  double horizon = 2 * M_PI / std::abs(rate.z());
  /// TODO: Get positon from gridmap
  map.add(layer_name);

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    Eigen::Vector2d pos_2d;
    map.getPosition(index, pos_2d);
    double terrain_elevation = map.at("distance_surface", index);
    Eigen::Vector3d pos = Eigen::Vector3d(pos_2d.x(), pos_2d.y(), terrain_elevation);

    double max_altitude = -std::numeric_limits<double>::infinity();

    /// TODO: Iterate on different yaw states
    for (double yaw = 0.0; yaw < 2 * M_PI; yaw += 0.25 * M_PI) {
      Eigen::Vector3d vel = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
      Trajectory trajectory = manuever_library->generateArcTrajectory(rate, horizon, pos, vel);

      for (auto& position : trajectory.position()) {
        /// TODO: Handle outside states as part of collision surface?
        if (!map.isInside(Eigen::Vector2d(position.x(), position.y()))) continue;
        double altitude = map.atPosition("distance_surface", Eigen::Vector2d(position.x(), position.y()));
        if (altitude > max_altitude) max_altitude = altitude;
      }
    }
    map.at(layer_name, index) = max_altitude;
  }
}

void addMinICSLayer(const std::string layer_name, grid_map::GridMap& map, const double yaw_rate) {
  /// Choose yaw state to calculate ICS state

  auto manuever_library = std::make_shared<ManeuverLibrary>();

  Eigen::Vector3d rate = Eigen::Vector3d(0.0, 0.0, yaw_rate);
  double horizon = 2 * M_PI / std::abs(rate.z());
  /// TODO: Get positon from gridmap
  map.add(layer_name);

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    Eigen::Vector2d pos_2d;
    map.getPosition(index, pos_2d);
    double terrain_elevation = map.at("distance_surface", index);
    Eigen::Vector3d pos = Eigen::Vector3d(pos_2d.x(), pos_2d.y(), terrain_elevation);

    double min_altitude = std::numeric_limits<double>::infinity();

    /// TODO: Iterate on different yaw states
    for (double yaw = 0.0; yaw < 2 * M_PI; yaw += 0.25 * M_PI) {
      Eigen::Vector3d vel = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
      Trajectory trajectory = manuever_library->generateArcTrajectory(rate, horizon, pos, vel);

      for (auto& position : trajectory.position()) {
        /// TODO: Handle outside states as part of collision surface?
        if (!map.isInside(Eigen::Vector2d(position.x(), position.y()))) continue;
        double altitude = map.atPosition("max_elevation", Eigen::Vector2d(position.x(), position.y()));
        if (altitude < min_altitude) min_altitude = altitude;
      }
    }
    map.at(layer_name, index) = min_altitude;
  }
}

void addErrorLayer(const std::string layer_name, grid_map::GridMap& map) {
  map.add(layer_name);

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    map.at(layer_name, index) = std::max(double(map.at("ics_-", index) - map.at("ics_+", index)), 0.0);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  std::string map_path, map_color_path;
  nh_private.param<std::string>("map_path", map_path, "resources/cadastre.tif");
  nh_private.param<std::string>("color_file_path", map_color_path, "resources/cadastre.tif");
  std::shared_ptr<TerrainMap> terrain_map = std::make_shared<TerrainMap>();
  terrain_map->Load(map_path, false, map_color_path);

  /// TODO: Add layer of ics based on the specified maneuver
  std::cout << "Compute ICS Layer yaw resolution 0.25 M_PI" << std::endl;
  addMaxICSLayer("ics_+", terrain_map->getGridMap(), 0.3);
  addMinICSLayer("ics_-", terrain_map->getGridMap(), 0.3);
  addErrorLayer("error", terrain_map->getGridMap());
  std::cout << "Completed ICS Computation!" << std::endl;

  while (true) {
    // Visualize yaw direction
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), message);
    grid_map_pub_.publish(message);
    ros::Duration(1.0).sleep();
  }
  ros::spin();
  return 0;
}

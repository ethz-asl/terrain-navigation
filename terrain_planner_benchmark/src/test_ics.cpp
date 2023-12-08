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

#include <grid_map_msgs/GridMap.h>
#include <terrain_navigation/terrain_map.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include "terrain_navigation/data_logger.h"
#include "terrain_planner/visualization.h"

void addErrorLayer(const std::string layer_name, const std::string query_layer, const std::string reference_layer,
                   grid_map::GridMap& map) {
  map.add(layer_name);

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    map.at(layer_name, index) = map.at(query_layer, index) > map.at(reference_layer, index);
  }
}

void calculateCircleICS(const std::string layer_name, std::shared_ptr<TerrainMap>& terrain_map, double radius) {
  /// Choose yaw state to calculate ICS state
  grid_map::GridMap& map = terrain_map->getGridMap();

  /// TODO: Get positon from gridmap
  terrain_map->AddLayerHorizontalDistanceTransform(radius, "ics_+", "distance_surface");
  terrain_map->AddLayerHorizontalDistanceTransform(-radius, "ics_-", "max_elevation");
  addErrorLayer(layer_name, "ics_-", "ics_+", terrain_map->getGridMap());
}

bool checkCollision(grid_map::GridMap& map, const Eigen::Vector2d pos_2d, const double yaw, const double radius) {
  auto manuever_library = std::make_shared<ManeuverLibrary>();

  double upper_altitude = std::numeric_limits<double>::infinity();
  double lower_altitude = -std::numeric_limits<double>::infinity();
  double yaw_rate = (1 / radius) * 20.0;
  Eigen::Vector3d rate = Eigen::Vector3d(0.0, 0.0, yaw_rate);
  Eigen::Vector3d vel = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
  double horizon = 2 * M_PI / std::abs(rate.z());
  Eigen::Vector3d pos = Eigen::Vector3d(pos_2d.x(), pos_2d.y(), 0.0);

  PathSegment trajectory = manuever_library->generateArcTrajectory(rate, horizon, pos, vel);
  for (auto& position : trajectory.position()) {
    if (!map.isInside(Eigen::Vector2d(position.x(), position.y()))) {
      // Handle outside states as part of collision surface
      // Consider it as a collision when the circle trajectory is outside of the map
      continue;
      // return true;
    };
    double min_collisionaltitude = map.atPosition("distance_surface", Eigen::Vector2d(position.x(), position.y()));
    if (min_collisionaltitude > lower_altitude) lower_altitude = min_collisionaltitude;
    double max_collisionaltitude = map.atPosition("max_elevation", Eigen::Vector2d(position.x(), position.y()));
    if (max_collisionaltitude < upper_altitude) upper_altitude = max_collisionaltitude;
  }

  if (upper_altitude > lower_altitude) {
    return false;
  } else {
    return true;
  }
}

void calculateYawICS(const std::string layer_name, grid_map::GridMap& map, const double yaw, const double radius) {
  /// Choose yaw state to calculate ICS state

  auto manuever_library = std::make_shared<ManeuverLibrary>();

  map.add(layer_name);
  map.add("yaw_error_right");
  map.add("yaw_error_left");

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    Eigen::Vector2d pos_2d;
    map.getPosition(index, pos_2d);

    bool right_hand_circle_in_collision = checkCollision(map, pos_2d, yaw, radius);
    bool left_hand_circle_in_collision = checkCollision(map, pos_2d, yaw, -radius);
    map.at(layer_name, index) = (!right_hand_circle_in_collision || !left_hand_circle_in_collision);
    map.at("yaw_error_right", index) = !right_hand_circle_in_collision;
    map.at("yaw_error_left", index) = !left_hand_circle_in_collision;
  }
}

double getCoverage(const std::string layer_name, const double threshold, const grid_map::GridMap& map) {
  int cell_count{0};
  int valid_count{0};
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    Eigen::Vector2d pos_2d;
    if (map.at(layer_name, index) > threshold) {
      valid_count++;
    }
    cell_count++;
  }

  double coverage = double(valid_count) / double(cell_count);
  return coverage;
}

void publishCirclularPath(const ros::Publisher pub, const Eigen::Vector3d center_position, const double radius) {
  // TODO: Publish circular trajectory
  const int num_discretization = 100;
  std::vector<Eigen::Vector3d> circle_path;
  for (int i = 0; i <= num_discretization; i++) {
    Eigen::Vector3d position;
    position << center_position(0) + radius * std::cos(i * 2 * M_PI / double(num_discretization)),
        center_position(1) + radius * std::sin(i * 2 * M_PI / double(num_discretization)), center_position(2);
    circle_path.push_back(position);
  }
  publishPath(pub, circle_path, Eigen::Vector3d(0.0, 1.0, 0.0));
}

void writeGridmapToImage(const grid_map::GridMap& map, const std::string layer, const std::string& file_path) {
  cv_bridge::CvImage image;
  grid_map::GridMapRosConverter::toCvImage(map, layer, sensor_msgs::image_encodings::MONO8, image);
  if (!cv::imwrite(file_path.c_str(), image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT})) {
    std::cout << "Failed to write map to image: " << file_path << std::endl;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  auto grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  auto reference_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("reference_map", 1, true);
  auto yaw_pub = nh.advertise<visualization_msgs::Marker>("yaw", 1, true);
  auto circle_pub = nh.advertise<visualization_msgs::Marker>("circle", 1, true);

  std::string map_path, map_color_path, output_file_dir, location;
  bool visualize{true};
  nh_private.param<std::string>("location", location, "dischma");
  nh_private.param<std::string>("map_path", map_path, "resources/cadastre.tif");
  nh_private.param<std::string>("color_file_path", map_color_path, "resources/cadastre.tif");
  nh_private.param<std::string>("output_file_dir", output_file_dir, "resources/output.csv");
  nh_private.param<bool>("visualize", visualize, true);

  std::shared_ptr<TerrainMap> reference_map = std::make_shared<TerrainMap>();
  reference_map->Load(map_path, false, map_color_path);
  reference_map->AddLayerDistanceTransform(50.0, "distance_surface");
  reference_map->AddLayerDistanceTransform(120.0, "max_elevation");
  double width = reference_map->getGridMap().getLength().y();
  Eigen::Vector2d reference_map_position = Eigen::Vector2d(0.0, 1.5 * width);
  reference_map->getGridMap().setPosition(reference_map_position);

  const double radius = 66.67;

  calculateCircleICS("circle_error", reference_map, radius);
  double circle_coverage = getCoverage("circle_error", 0.0, reference_map->getGridMap());
  std::cout << "  - coverage: " << circle_coverage << std::endl;
  Eigen::Vector3d marker_position{Eigen::Vector3d(reference_map_position(0), reference_map_position(1), 400.0)};
  publishCirclularPath(circle_pub, marker_position, 200.0);
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(reference_map->getGridMap(), message);
  reference_map_pub_.publish(message);

  std::string reference_map_path = output_file_dir + "/" + location + "_circle_goal.png";
  writeGridmapToImage(reference_map->getGridMap(), "circle_error", reference_map_path);

  std::cout << "Valid yaw terminal state coverage" << std::endl;
  auto data_logger = std::make_shared<DataLogger>();
  data_logger->setKeys({"yaw", "yaw_coverage", "circle_coverage", "yaw_error_left", "yaw_error_right"});
  std::cout << "Valid circlular terminal state coverage" << std::endl;

  std::shared_ptr<TerrainMap> terrain_map = std::make_shared<TerrainMap>();
  terrain_map->Load(map_path, false, map_color_path);
  terrain_map->AddLayerDistanceTransform(50.0, "distance_surface");
  terrain_map->AddLayerDistanceTransform(120.0, "max_elevation");

  for (double yaw = 0.0; yaw < 2 * M_PI; yaw += 0.125 * M_PI) {
    calculateYawICS("yaw_error", terrain_map->getGridMap(), yaw, radius);
    double coverage = getCoverage("yaw_error", 0.0, terrain_map->getGridMap());
    double coverage_left = getCoverage("yaw_error_left", 0.0, terrain_map->getGridMap());
    double coverage_right = getCoverage("yaw_error_right", 0.0, terrain_map->getGridMap());
    std::cout << "  - yaw: " << yaw / M_PI << " pi / coverage: " << coverage << " right: " << coverage_right
              << " left: " << coverage_left << std::endl;

    std::unordered_map<std::string, std::any> state;
    state.insert(std::pair<std::string, double>("yaw", yaw));
    state.insert(std::pair<std::string, double>("yaw_coverage", coverage));
    state.insert(std::pair<std::string, double>("yaw_error_left", coverage_left));
    state.insert(std::pair<std::string, double>("yaw_error_right", coverage_right));
    state.insert(std::pair<std::string, double>("circle_coverage", circle_coverage));
    data_logger->record(state);

    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), message);
    grid_map_pub_.publish(message);
    grid_map_msgs::GridMap message2;
    grid_map::GridMapRosConverter::toMessage(reference_map->getGridMap(), message2);
    reference_map_pub_.publish(message2);

    Eigen::Vector3d pos(0.0, 0.0, 400.0);
    Eigen::Vector3d vel(std::cos(yaw), std::sin(yaw), 0.0);
    publishPositionSetpoints(yaw_pub, pos, vel, Eigen::Vector3d(200.0, 40.0, 40.0));
    std::string terrain_map_path = output_file_dir + "/" + location + "_yaw_" + std::to_string(yaw) + ".png";
    writeGridmapToImage(terrain_map->getGridMap(), "yaw_error", terrain_map_path);
  }

  data_logger->setPrintHeader(true);
  std::string output_file_path = output_file_dir + "/" + location + ".csv";
  data_logger->writeToFile(output_file_path);
  if (visualize) {
    while (true) {
      // Visualize yaw direction
      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), message);
      grid_map_pub_.publish(message);
      grid_map_msgs::GridMap message2;
      grid_map::GridMapRosConverter::toMessage(reference_map->getGridMap(), message2);
      reference_map_pub_.publish(message2);
    }
  }
  return 0;
}

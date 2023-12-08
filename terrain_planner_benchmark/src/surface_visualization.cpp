/****************************************************************************
 *
 *   Copyright (c) 2023 Jaeyoung Lim, Autonomous Systems Lab,
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

void drawCircle(grid_map::GridMap& map, Eigen::Vector2d center, double radius, double height) {
  for (grid_map::CircleIterator submapIterator(map, center, radius); !submapIterator.isPastEnd(); ++submapIterator) {
    const grid_map::Index SubmapIndex = *submapIterator;
    map.at("elevation", SubmapIndex) = height;
  }
}

void generateTestMap(grid_map::GridMap& map) {
  // Create a synthetic map for visualization
  grid_map::Length length = grid_map::Length(500.0, 500.0);
  grid_map::Position position = grid_map::Position(0.0, 0.0);
  double resolution = 5.0;
  map.setGeometry(length, resolution, position);
  map.setFrameId("map");
  map["elevation"].setConstant(0.0);

  Eigen::Vector2d circle_center_2d(0.0, 0.0);
  double radius = 60.0;
  drawCircle(map, Eigen::Vector2d(100.0, 100.0), 60.0, 120.0);
  drawCircle(map, Eigen::Vector2d(-200.0, 0.0), 30.0, 150.0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  auto reference_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  std::string output_file_dir;
  bool visualize{true};
  nh_private.param<std::string>("output_file_dir", output_file_dir, "resources/output.csv");
  nh_private.param<bool>("visualize", visualize, true);

  std::shared_ptr<TerrainMap> reference_map = std::make_shared<TerrainMap>();
  /// TODO: Define a custom map and set it on grid_map
  grid_map::GridMap test_map({"elevation"});
  generateTestMap(test_map);

  reference_map->setGridMap(test_map);

  // reference_map->Load(map_path, false, map_color_path);
  reference_map->AddLayerDistanceTransform(50.0, "distance_surface");
  reference_map->AddLayerDistanceTransform(120.0, "max_elevation");

  calculateCircleICS("circle_error", reference_map, 66.67);
  double circle_coverage = getCoverage("circle_error", 0.0, reference_map->getGridMap());
  std::cout << "  - coverage: " << circle_coverage << std::endl;
  std::string reference_map_path = output_file_dir + "/test" + "_circle_goal.png";
  writeGridmapToImage(reference_map->getGridMap(), "circle_error", reference_map_path);

  std::cout << "Valid yaw terminal state coverage" << std::endl;
  auto data_logger = std::make_shared<DataLogger>();
  data_logger->setKeys({"yaw", "yaw_coverage", "circle_coverage"});
  std::cout << "Valid circlular terminal state coverage" << std::endl;

  if (visualize) {
    while (true) {
      // Visualize yaw direction
      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(reference_map->getGridMap(), message);
      reference_map_pub_.publish(message);
    }
  }
  return 0;
}

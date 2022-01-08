/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include "adaptive_viewutility/adaptive_viewutility.h"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "terrain_navigation/profiler.h"

struct MapData {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  double elevation{NAN};
  double error{NAN};
  double utility{NAN};
};

void MapPublishOnce(ros::Publisher &pub, const std::shared_ptr<ViewUtilityMap> &map) {
  map->getGridMap().setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map->getGridMap(), message);
  pub.publish(message);
}

void printGridmapInfo(std::string name, grid_map::GridMap &map) {
  std::cout << "Map " << name << std::endl;
  std::cout << " - position: " << map.getPosition().transpose() << std::endl;
  std::cout << " - length: " << map.getLength().transpose() << std::endl;
}

void CopyMapLayer(const std::string &layer, const grid_map::GridMap &reference_map, grid_map::GridMap &target_map) {
  target_map.add(layer);
  for (grid_map::GridMapIterator iterator(target_map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    Eigen::Vector2d cell_position;
    target_map.getPosition(index, cell_position);
    if (reference_map.isInside(cell_position)) {
      // Using at position allows us to use different resolution maps
      target_map.at(layer, index) = reference_map.atPosition(layer, cell_position);
    }
  }
  return;
}

void writeMapDataToFile(const std::string path, const std::vector<MapData> &map) {
  std::ofstream output_file;
  output_file.open(path, std::ios::app);
  output_file << "id,x,y,error,utility,padding,\n";
  int id{0};
  for (auto data : map) {
    output_file << id << ",";
    output_file << data.position(0) << ",";
    output_file << data.position(1) << ",";
    output_file << data.error << ",";
    output_file << data.utility << ",";
    output_file << 0 << ",";
    output_file << "\n";
    id++;
  }
  output_file.close();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher gt_map_pub = nh.advertise<grid_map_msgs::GridMap>("groundthruth_map", 1, true);
  ros::Publisher est_map_pub = nh.advertise<grid_map_msgs::GridMap>("estimated_map", 1, true);
  ros::Publisher utility_map_pub = nh.advertise<grid_map_msgs::GridMap>("utility_map", 1, true);

  std::string gt_path, est_path, viewutility_map_path, output_path;

  nh_private.param<std::string>("groundtruth_mesh_path", gt_path, "resources/cadastre.tif");
  nh_private.param<std::string>("estimated_mesh_path", est_path, "resources/cadastre.tif");
  nh_private.param<std::string>("map_data_path", output_path, "");

  if (gt_path.empty() || est_path.empty()) {
    std::cout << "Missing groundtruth mesh or the estimated mesh" << std::endl;
    return 1;
  }

  nh_private.param<std::string>("utility_map_path", viewutility_map_path, "");

  grid_map::GridMap gt_map =
      grid_map::GridMap({"roi", "elevation", "elevation_normal_x", "elevation_normal_y", "elevation_normal_z",
                         "visibility", "geometric_prior", "normalized_prior"});
  std::shared_ptr<ViewUtilityMap> groundtruth_map = std::make_shared<ViewUtilityMap>(gt_map);
  double resolution = 1.0;
  groundtruth_map->initializeFromMesh(gt_path, resolution);

  grid_map::GridMap est_map =
      grid_map::GridMap({"roi", "elevation", "elevation_normal_x", "elevation_normal_y", "elevation_normal_z",
                         "visibility", "geometric_prior", "normalized_prior"});
  std::shared_ptr<ViewUtilityMap> estimated_map = std::make_shared<ViewUtilityMap>(est_map);
  estimated_map->initializeFromMesh(est_path, resolution);

  // Known transform acquired by cloudcompare
  Eigen::Translation3d meshlab_translation(227.044357, -460.073242, -583.809143);
  Eigen::AngleAxisd meshlab_rotation(11.177129 * M_PI / 180.0, Eigen::Vector3d(0.871801, -0.489854, -0.002307));

  Eigen::Isometry3d transform = meshlab_translation * meshlab_rotation;  // Apply affine transformation.
  groundtruth_map->getGridMap() = groundtruth_map->getGridMap().getTransformedMap(
      transform, "elevation", groundtruth_map->getGridMap().getFrameId(), true);

  printGridmapInfo("Groundtruth map (After Transform)", groundtruth_map->getGridMap());
  printGridmapInfo("Estimated map", estimated_map->getGridMap());

  groundtruth_map->CompareMapLayer(estimated_map->getGridMap());

  grid_map::GridMap viewutility_map;
  if (!viewutility_map_path.empty()) {
    std::cout << "[CompareMeshNode ] Loading Utility map: " << viewutility_map_path << std::endl;
    if (grid_map::GridMapRosConverter::loadFromBag(viewutility_map_path, "/grid_map", viewutility_map)) {
      viewutility_map = viewutility_map.getTransformedMap(transform, "elevation", viewutility_map.getFrameId(), true);

      CopyMapLayer("geometric_prior", viewutility_map, groundtruth_map->getGridMap());
    } else {
      std::cout << "  - Failed to load utility map" << std::endl;
    }
  }

  std::vector<MapData> map_data;
  grid_map::GridMap &grid_map = groundtruth_map->getGridMap();
  /// TODO: Iterate through gridmap to save map data in file
  for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    MapData data;
    grid_map.getPosition(index, data.position);
    data.elevation = grid_map.at("elevation", index);
    data.error = grid_map.at("elevation_difference", index);
    data.utility = grid_map.at("geometric_prior", index);
    map_data.push_back(data);
  }
  writeMapDataToFile(output_path, map_data);
  while (true) {
    MapPublishOnce(gt_map_pub, groundtruth_map);
    MapPublishOnce(est_map_pub, estimated_map);
    if (!viewutility_map_path.empty()) {
      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(viewutility_map, message);
      utility_map_pub.publish(message);
    }
    ros::Duration(5.0).sleep();
  }

  ros::spin();
  return 0;
}

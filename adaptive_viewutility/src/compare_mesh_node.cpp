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
  double ground_sample_distance{NAN};
  double triangulation_prior{NAN};
  double incident_prior{NAN};
  double visibility{NAN};
  double min_eigen_value{NAN};
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
  output_file << "id,x,y,error,utility,ground_sample_distance,incident_prior,triangulation_prior,visibility,min_eigen_"
                 "value,padding,\n";
  int id{0};
  for (auto data : map) {
    output_file << id << ",";
    output_file << data.position(0) << ",";
    output_file << data.position(1) << ",";
    output_file << data.error << ",";
    output_file << data.utility << ",";
    output_file << data.ground_sample_distance << ",";
    output_file << data.incident_prior << ",";
    output_file << data.triangulation_prior << ",";
    output_file << data.visibility << ",";
    output_file << data.min_eigen_value << ",";
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
  bool visualization_enabled{true};
  nh_private.param<std::string>("groundtruth_mesh_path", gt_path, "resources/cadastre.tif");
  nh_private.param<std::string>("estimated_mesh_path", est_path, "resources/cadastre.tif");
  nh_private.param<std::string>("map_data_path", output_path, "");
  nh_private.param<bool>("visualize", visualization_enabled, true);

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
  // Qmax05 200steps
  // Eigen::Translation3d meshlab_translation(218.003296, -428.974365, -377.713348);
  // Eigen::AngleAxisd meshlab_rotation(1.082817 * M_PI / 180.0, Eigen::Vector3d(0.050463, 0.056066, -0.997151));
  // Qmax05 400steps
  // Eigen::Translation3d meshlab_translation(235.992828, -424.721649, -361.758606);
  // Eigen::AngleAxisd meshlab_rotation(1.074108 * M_PI / 180.0, Eigen::Vector3d(0.057445, 0.054471, -0.996862));
  // Triangulation Prior fix
  // Eigen::Translation3d meshlab_translation(182.640350,-459.849915,-242.304398);
  // Eigen::AngleAxisd meshlab_rotation(1.136428 * M_PI / 180.0, Eigen::Vector3d(0.003528,-0.001497,-0.999993));
  // Fisher information fix
  // Eigen::Translation3d meshlab_translation(179.645706,-468.833771,-246.334869);
  // Eigen::AngleAxisd meshlab_rotation(1.139866 * M_PI / 180.0, Eigen::Vector3d(0.004730, -0.000685, -0.999989));
  // Fisher information render fix
  Eigen::Translation3d meshlab_translation(194.932800, -427.770630, -313.790955);
  Eigen::AngleAxisd meshlab_rotation(1.084983 * M_PI / 180.0, Eigen::Vector3d(0.053948, 0.048623, -0.997359));

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
      Eigen::Translation3d airsim_start_pos(-374.47859375, 723.12984375, 286.77371094);
      Eigen::AngleAxisd airsim_start_rot(0.0 * M_PI / 180.0, Eigen::Vector3d(0.0, 0.0, 1.0));
      Eigen::Isometry3d airsim_transform = airsim_start_pos * airsim_start_rot;

      viewutility_map =
          viewutility_map.getTransformedMap(airsim_transform, "elevation", viewutility_map.getFrameId(), true);
      viewutility_map = viewutility_map.getTransformedMap(transform, "elevation", viewutility_map.getFrameId(), true);

      CopyMapLayer("geometric_prior", viewutility_map, groundtruth_map->getGridMap());
      CopyMapLayer("ground_sample_distance", viewutility_map, groundtruth_map->getGridMap());
      CopyMapLayer("incident_prior", viewutility_map, groundtruth_map->getGridMap());
      CopyMapLayer("triangulation_prior", viewutility_map, groundtruth_map->getGridMap());
      CopyMapLayer("visibility", viewutility_map, groundtruth_map->getGridMap());
      CopyMapLayer("min_eigen_value", viewutility_map, groundtruth_map->getGridMap());
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
    data.incident_prior = grid_map.at("incident_prior", index);
    data.triangulation_prior = grid_map.at("triangulation_prior", index);
    data.ground_sample_distance = grid_map.at("ground_sample_distance", index);
    data.visibility = grid_map.at("visibility", index);
    data.min_eigen_value = grid_map.at("min_eigen_value", index);
    map_data.push_back(data);
  }
  writeMapDataToFile(output_path, map_data);
  if (visualization_enabled) {
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
  }
  return 0;
}

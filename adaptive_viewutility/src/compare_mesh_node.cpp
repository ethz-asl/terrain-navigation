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

#include <gdal/cpl_string.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_p.h>
#include <gdal/ogr_spatialref.h>

enum class ESPG { ECEF = 4978, WGS84 = 4326, WGS84_32N = 32632, CH1903_LV03 = 21781 };
static constexpr const double kDefaultHomeX = 683565.21;     // LV03/CH1903
static constexpr const double kDefaultHomeY = 250246.85;     // rad
static constexpr const double kDefaultHomeAltitude = 488.0;  // meters

static Eigen::Vector3d transformCoordinates(ESPG src_coord, ESPG tgt_coord, const Eigen::Vector3d source_coordinates) {
  OGRSpatialReference source, target;
  source.importFromEPSG(static_cast<int>(src_coord));
  target.importFromEPSG(static_cast<int>(tgt_coord));

  OGRPoint p;
  p.setX(source_coordinates(0));
  p.setY(source_coordinates(1));
  p.setZ(source_coordinates(2));
  p.assignSpatialReference(&source);

  p.transformTo(&target);
  Eigen::Vector3d target_coordinates(p.getX(), p.getY(), p.getZ());
  return target_coordinates;
}

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
  output_file.open(path);
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

Eigen::Vector3d readOffsetFile(const std::string path) {
  Eigen::Vector3d offset;
  std::ifstream file(path);
  std::string data = "";
  int i = 0;
  while (getline(file, data, ' ')) {
    offset(i) = std::stod(data);
    i++;
  }
  std::cout << "offset" << offset.transpose() << std::endl;
  return offset;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher gt_map_pub = nh.advertise<grid_map_msgs::GridMap>("groundthruth_map", 1, true);
  ros::Publisher est_map_pub = nh.advertise<grid_map_msgs::GridMap>("estimated_map", 1, true);
  ros::Publisher utility_map_pub = nh.advertise<grid_map_msgs::GridMap>("utility_map", 1, true);

  std::string gt_path, est_path, viewutility_map_path, output_path, offset_file_path;
  bool visualization_enabled{true};
  nh_private.param<std::string>("groundtruth_mesh_path", gt_path, "resources/cadastre.tif");
  nh_private.param<std::string>("estimated_mesh_path", est_path, "resources/cadastre.tif");
  nh_private.param<std::string>("map_data_path", output_path, "");
  nh_private.param<std::string>("offset_file_path", offset_file_path, "");
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

  // Offset data acquired from Pix4D
  // Eigen::Vector3d position_wgs8432n(465873.000, 5249166.000, 554.000);
  Eigen::Vector3d position_wgs8432n = readOffsetFile(offset_file_path);
  Eigen::Vector3d position_lv03 = transformCoordinates(ESPG::WGS84_32N, ESPG::CH1903_LV03, position_wgs8432n);
  Eigen::Vector3d home(kDefaultHomeX, kDefaultHomeY, kDefaultHomeAltitude);
  Eigen::Vector3d local_offset = position_lv03 - home;
  Eigen::Vector3d player_start{Eigen::Vector3d(374.47859375, -723.12984375, -286.77371094)};
  Eigen::Vector3d adjusted_offset = player_start - local_offset;
  Eigen::Translation3d meshlab_translation(adjusted_offset(0), adjusted_offset(1), adjusted_offset(2));
  Eigen::AngleAxisd meshlab_rotation(0.0 * M_PI / 180.0, Eigen::Vector3d(0.0, 0.0, 1.0));

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

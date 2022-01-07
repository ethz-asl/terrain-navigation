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
#include "terrain_navigation/profiler.h"
#include "grid_map_ros/GridMapRosConverter.hpp"

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher original_map_pub = nh.advertise<grid_map_msgs::GridMap>("groundthruth_map", 1, true);
  ros::Publisher loaded_map_pub = nh.advertise<grid_map_msgs::GridMap>("loaded_map", 1, true);

  double resolution = 1.0;

  std::string original_map_path{};
  std::string saved_map_path{};

  nh_private.param<std::string>("original_map_path", original_map_path, "resources/cadastre.tif");
  nh_private.param<std::string>("saved_map_path", saved_map_path, "resources/cadastre.bag");

  grid_map::GridMap gt_map =
      grid_map::GridMap({"roi", "elevation", "elevation_normal_x", "elevation_normal_y", "elevation_normal_z",
                         "visibility", "geometric_prior", "normalized_prior"});
  std::shared_ptr<ViewUtilityMap> groundtruth_map = std::make_shared<ViewUtilityMap>(gt_map);
  groundtruth_map->initializeFromMesh(original_map_path, resolution);

  grid_map::GridMapRosConverter::saveToBag(groundtruth_map->getGridMap(), saved_map_path, "/grid_map");

  grid_map::GridMap loaded_map;
  grid_map::GridMapRosConverter::loadFromBag(saved_map_path, "/grid_map", loaded_map);


  while (true) {
    MapPublishOnce(original_map_pub, groundtruth_map);
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(loaded_map, message);
    loaded_map_pub.publish(message);
    ros::Duration(5.0).sleep();
  }

  ros::spin();
  return 0;
}

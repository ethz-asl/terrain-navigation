/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim. All rights reserved.
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
 * @brief Performance Evaluation Helper Functions
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef EVALUATION_H
#define EVALUATION_H

#include "adaptive_viewutility/adaptive_viewutility.h"

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

void printGridmapInfo(std::string name, grid_map::GridMap &map) {
  std::cout << "Map " << name << std::endl;
  std::cout << " - position: " << map.getPosition().transpose() << std::endl;
  std::cout << " - length: " << map.getLength().transpose() << std::endl;
}

void MapPublishOnce(ros::Publisher &pub, const std::shared_ptr<ViewUtilityMap> &map) {
  map->getGridMap().setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map->getGridMap(), message);
  pub.publish(message);
}

#endif

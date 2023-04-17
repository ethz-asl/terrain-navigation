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
 * @brief Terain map representation
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "terrain_navigation/terrain_map.h"
#include <grid_map_core/GridMapMath.hpp>

TerrainMap::TerrainMap() : GridMapGeo() {}

TerrainMap::~TerrainMap() {}

void TerrainMap::addLayerSafety(const std::string &layer, const std::string &layer_name_lowerbound,
                                const std::string &layer_name_upperbound) {
  /// TODO: Iterate over map to find the distance surface and validation
  grid_map_.add(layer);
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    const double value_lowerbound = grid_map_.at(layer_name_lowerbound, index);
    const double value_upperbound = grid_map_.at(layer_name_upperbound, index);
    if (value_lowerbound < value_upperbound) {
      grid_map_.at(layer, index) = (value_lowerbound + value_upperbound) / 2.0;
    } else {
      grid_map_.at(layer, index) = NAN;
    }
  }
}

bool TerrainMap::isInCollision(const std::string &layer, const Eigen::Vector3d &position, const bool is_above) {
  const Eigen::Vector2d position_2d(position(0), position(1));
  if (grid_map_.isInside(position_2d)) {
    const double elevation = grid_map_.atPosition(layer, position_2d);
    if (is_above) {
      if (elevation > position(2)) {
        return true;
      } else {
        return false;
      }
    } else {
      if (elevation < position(2)) {
        return true;
      } else {
        return false;
      }
    }
  } else {
    return true;  // Do not allow vehicle to go outside the map
  }
}

double TerrainMap::getCollisionDepth(const std::string &layer, const Eigen::Vector3d &position, bool is_above) {
  double collision_depth{0.0};
  Eigen::Vector2d position_2d(position(0), position(1));
  if (!grid_map_.isInside(position_2d)) return true;  // Do not allow vehicle to go outside the map
  double elevation = grid_map_.atPosition(layer, position_2d);
  if (is_above) {
    if (elevation > position(2)) {
      collision_depth = elevation - position(2);
    }
  } else {
    if (elevation < position(2)) {
      collision_depth = position(2) - elevation;
    }
  }
  return collision_depth;
}

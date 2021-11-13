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

#ifndef TERRAIN_MAP_H
#define TERRAIN_MAP_H

enum class ESPG { ECEF = 4978, WGS84 = 4326, CH1903_LV03 = 21781 };

// #include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <gdal/cpl_string.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_p.h>
#include <gdal/ogr_spatialref.h>
#include <iostream>

class TerrainMap {
 public:
  TerrainMap();
  virtual ~TerrainMap();
  bool initializeFromGeotiff(const std::string& path);
  bool AddLayerDistanceTransform(const std::string& string);
  grid_map::GridMap& getGridMap() { return grid_map_; }
  bool isInCollision(const std::string& layer, const Eigen::Vector3d& position, bool is_above = true);
  double getCollisionDepth(const std::string& layer, const Eigen::Vector3d& position, bool is_above = true);
  void setGlobalOrigin(ESPG src_coord, const Eigen::Vector2d origin);
  static Eigen::Vector2d transformCoordinates(ESPG src_coord, ESPG tgt_coord,
                                                  const Eigen::Vector2d source_coordinates) {
    OGRSpatialReference source, target;
    source.importFromEPSG(static_cast<int>(src_coord));
    target.importFromEPSG(static_cast<int>(tgt_coord));

    OGRPoint p;
    p.setX(source_coordinates(0));
    p.setY(source_coordinates(1));
    p.assignSpatialReference(&source);

    p.transformTo(&target);
    Eigen::Vector2d target_coordinates(p.getX(), p.getY());
    return target_coordinates;
  }
 private:
  grid_map::GridMap grid_map_;
  double localorigin_e_{789823.93};  // duerrboden berghaus
  double localorigin_n_{177416.56};
};
#endif

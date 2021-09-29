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

#include "terrain_planner/terrain_map.h"
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

TerrainMap::TerrainMap() {}

TerrainMap::~TerrainMap() {}

bool TerrainMap::isInCollision(const std::string &layer, const Eigen::Vector3d &position) {
  Eigen::Vector2d position_2d(position(0), position(1));
  if (grid_map_.isInside(position_2d)) {
    double elevation = grid_map_.atPosition(layer, position_2d);
    if (elevation > position(2)) {
      return true;
    } else {
      return false;
    }
  } else {
    return true;  // Do not allow vehicle to go outside the map
  }
}

bool TerrainMap::initializeFromGeotiff(const std::string &path) {
  GDALAllRegister();
  GDALDataset *dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_ReadOnly);
  if (!dataset) {
    std::cout << "Failed to open" << std::endl;
    return false;
  }
  std::cout << std::endl << "Loading GeoTIFF file for gridmap" << std::endl;

  double originX, originY, pixelSizeX, pixelSizeY;

  double geoTransform[6];
  if (dataset->GetGeoTransform(geoTransform) == CE_None) {
    originX = geoTransform[0];
    originY = geoTransform[3];
    pixelSizeX = geoTransform[1];
    pixelSizeY = geoTransform[5];
  } else {
    std::cout << "Failed read geotransform" << std::endl;
    return false;
  }

  const char *pszProjection = dataset->GetProjectionRef();
  std::cout << std::endl << "Wkt ProjectionRef: " << pszProjection << std::endl;
  /// TODO: Get proper projection references using gDal
  // double center_latitude = 46.7240653;
  // double center_longitude = 9.912185;
  double mapcenter_e = 789738.500;
  double mapcenter_n = 177105.500;

  double center_altitude = 2010.0;  /// TODO: Get center altitude as minimum altitude

  // duerrboden berghaus
  double localorigin_e = 789823.93;
  double localorigin_n = 177416.56;

  double map_position_x = mapcenter_e - localorigin_e;
  double map_position_y = mapcenter_n - localorigin_n;

  // Get image metadata
  unsigned width = dataset->GetRasterXSize();
  unsigned height = dataset->GetRasterYSize();
  double resolution = pixelSizeX;
  std::cout << "Width: " << width << " Height: " << height << " Resolution: " << resolution << std::endl;

  // pixelSizeY is negative because the origin of the image is the north-east corner and positive
  // Y pixel coordinates go towards the south
  const double lengthX = resolution * width;
  const double lengthY = resolution * height;
  grid_map::Length length(lengthX, lengthY);
  Eigen::Vector2d position = Eigen::Vector2d(map_position_x, map_position_y);
  std::cout << "map position: " << position.transpose() << std::endl;
  // Eigen::Vector2d position = Eigen::Vector2d::Zero();
  grid_map_.setGeometry(length, resolution, position);
  grid_map_.setFrameId("world");
  grid_map_.add("elevation");
  grid_map_.add("max_elevation");
  GDALRasterBand *elevationBand = dataset->GetRasterBand(1);

  std::vector<float> data(width * height, 0.0f);
  elevationBand->RasterIO(GF_Read, 0, 0, width, height, &data[0], width, height, GDT_Float32, 0, 0);

  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  grid_map::Matrix &layer_max_elevation = grid_map_["max_elevation"];
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    // TODO: This may be wrong if the pixelSizeY > 0
    int x = width - 1 - gridMapIndex(0);
    int y = gridMapIndex(1);

    layer_elevation(x, y) = data[gridMapIndex(0) + width * gridMapIndex(1)] - center_altitude;
    layer_max_elevation(x, y) = layer_elevation(x, y) + 150.0;
  }
  return true;
}

bool TerrainMap::AddLayerDistanceTransform(const std::string &string) {
  grid_map_.add("distance_surface");

  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index MapIndex = *iterator;
    /// TODO: Add circle iterator
    Eigen::Vector3d center_pos;
    grid_map_.getPosition3("elevation", MapIndex, center_pos);
    Eigen::Vector2d center_pos_2d(center_pos(0), center_pos(1));
    double surface_distance = 50.0;
    for (grid_map::CircleIterator submapIterator(grid_map_, center_pos_2d, surface_distance); !submapIterator.isPastEnd();
         ++submapIterator) {
      const grid_map::Index SubmapIndex = *submapIterator;
      Eigen::Vector3d cell_position;
      grid_map_.getPosition3("elevation", SubmapIndex, cell_position);
      double distance = (cell_position - center_pos).norm();
      if (distance < surface_distance) {
        grid_map_.at("distance_surface", MapIndex) = std::sqrt(std::pow(surface_distance, 2) - std::pow(distance, 2));
      }
    }
  }
  return true;
}

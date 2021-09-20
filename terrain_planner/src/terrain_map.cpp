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

TerrainMap::TerrainMap() {}

TerrainMap::~TerrainMap() {}

bool TerrainMap::isInCollision(const Eigen::Vector3d &position) {
  Eigen::Vector2d position_2d(position(0), position(1));
  double elevation = grid_map_.atPosition("elevation", position_2d);
  if (elevation > position(3)) {
    return true;
  } else {
    return false;
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
  // Fluela
  double center_latitude = 46.95240555555556;
  double center_longitude = 7.439583333333333;
  double center_altitude = 2500.0;  /// TODO: Get center altitude as minimum altitude

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
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  grid_map_.setGeometry(length, resolution, position);
  grid_map_.setFrameId("world");
  grid_map_.add("elevation");
  GDALRasterBand *elevationBand = dataset->GetRasterBand(1);

  std::vector<float> data(width * height, 0.0f);
  elevationBand->RasterIO(GF_Read, 0, 0, width, height, &data[0], width, height, GDT_Float32, 0, 0);

  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    int x = gridMapIndex(0);
    int y = height - 1 - gridMapIndex(1);

    layer_elevation(x, y) = data[gridMapIndex(0) + width * gridMapIndex(1)] - center_altitude;
  }
  return true;
}

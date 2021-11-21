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
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

TerrainMap::TerrainMap() {}

TerrainMap::~TerrainMap() {}

bool TerrainMap::isInCollision(const std::string &layer, const Eigen::Vector3d &position, bool is_above) {
  Eigen::Vector2d position_2d(position(0), position(1));
  if (grid_map_.isInside(position_2d)) {
    double elevation = grid_map_.atPosition(layer, position_2d);
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

  /// TODO: Align coordinates vertically
  double center_altitude = localorigin_altitude_;

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

  double mapcenter_e = originX + pixelSizeX * width * 0.5;
  double mapcenter_n = originY + pixelSizeY * height * 0.5;

  Eigen::Vector3d origin_lv03 =
      transformCoordinates(ESPG::WGS84, std::string(pszProjection), localorigin_wgs84_.position);
  localorigin_e_ = origin_lv03(0);
  localorigin_n_ = origin_lv03(1);
  localorigin_altitude_ = origin_lv03(2);

  double map_position_x = mapcenter_e - localorigin_e_;
  double map_position_y = mapcenter_n - localorigin_n_;

  Eigen::Vector2d position = Eigen::Vector2d(map_position_x, map_position_y);
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

  /// TODO: This is a workaround with the problem of gdal 3 not translating altitude correctly.
  /// This section just levels the current position to the ground
  Eigen::Translation3d meshlab_translation(
      0.0, 0.0, -grid_map_.atPosition("elevation", Eigen::Vector2d(position(0), position(1))));
  Eigen::AngleAxisd meshlab_rotation(Eigen::AngleAxisd::Identity());
  Eigen::Isometry3d transform = meshlab_translation * meshlab_rotation;  // Apply affine transformation.
  grid_map_ = grid_map_.getTransformedMap(transform, "elevation", grid_map_.getFrameId(), true);
  grid_map_ = grid_map_.getTransformedMap(transform, "max_elevation", grid_map_.getFrameId(), true);

  return true;
}

bool TerrainMap::addColorFromGeotiff(const std::string &path) {
  GDALAllRegister();
  GDALDataset *dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_ReadOnly);
  if (!dataset) {
    std::cout << "Failed to open" << std::endl;
    return false;
  }
  std::cout << std::endl << "Loading color layer from GeoTIFF file for gridmap" << std::endl;

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

  grid_map_.add("color");
  GDALRasterBand *raster_red = dataset->GetRasterBand(1);
  GDALRasterBand *raster_green = dataset->GetRasterBand(2);
  GDALRasterBand *raster_blue = dataset->GetRasterBand(3);

  std::vector<uint16_t> data_red(width * height, 0.0f);
  std::vector<uint16_t> data_green(width * height, 0.0f);
  std::vector<uint16_t> data_blue(width * height, 0.0f);

  raster_red->RasterIO(GF_Read, 0, 0, width, height, &data_red[0], width, height, GDT_UInt16, 0, 0);
  raster_green->RasterIO(GF_Read, 0, 0, width, height, &data_green[0], width, height, GDT_UInt16, 0, 0);
  raster_blue->RasterIO(GF_Read, 0, 0, width, height, &data_blue[0], width, height, GDT_UInt16, 0, 0);

  grid_map::Matrix &layer_color = grid_map_["color"];
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    /// TODO: This may be wrong if the pixelSizeY > 0
    int x = width - 1 - gridMapIndex(0);
    int y = gridMapIndex(1);
    Eigen::Vector3i colorVector;
    colorVector(2) = data_red[gridMapIndex(0) + width * gridMapIndex(1)];
    colorVector(1) = data_green[gridMapIndex(0) + width * gridMapIndex(1)];
    colorVector(0) = data_blue[gridMapIndex(0) + width * gridMapIndex(1)];
    grid_map::colorVectorToValue(colorVector, layer_color(x, y));
  }
  return true;
}

bool TerrainMap::AddLayerDistanceTransform(const std::string &string) {
  double surface_distance = 50.0;
  grid_map_.add("distance_surface");
  grid_map_.add("offset_surface");
  grid_map_.add("error_surface");

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index MapIndex = *iterator;
    Eigen::Vector3d center_pos;
    grid_map_.getPosition3("elevation", MapIndex, center_pos);
    center_pos(2) = center_pos(2) + surface_distance;
    Eigen::Vector2d center_pos_2d(center_pos(0), center_pos(1));
    grid_map_.at("distance_surface", MapIndex) = center_pos(2);  // elevation
    grid_map_.at("offset_surface", MapIndex) = center_pos(2);    // elevation
    for (grid_map::CircleIterator submapIterator(grid_map_, center_pos_2d, surface_distance);
         !submapIterator.isPastEnd(); ++submapIterator) {
      const grid_map::Index SubmapIndex = *submapIterator;
      Eigen::Vector3d cell_position;
      grid_map_.getPosition3("elevation", SubmapIndex, cell_position);
      double distance = (cell_position - center_pos).norm();
      if (distance < surface_distance) {
        double distance_2d = (Eigen::Vector2d(cell_position(0), cell_position(1)) - center_pos_2d).norm();
        grid_map_.at("distance_surface", MapIndex) =
            cell_position(2) + std::sqrt(std::pow(surface_distance, 2) - std::pow(distance_2d, 2));
      }
    }
    grid_map_.at("error_surface", MapIndex) =
        grid_map_.at("distance_surface", MapIndex) - grid_map_.at("offset_surface", MapIndex);  // elevation
  }
  return true;
}

void TerrainMap::setGlobalOrigin(ESPG src_coord, const Eigen::Vector3d origin) {
  // Transform global origin into CH1903 / LV03 coordinates
  localorigin_wgs84_.espg = src_coord;
  localorigin_wgs84_.position = origin;
}

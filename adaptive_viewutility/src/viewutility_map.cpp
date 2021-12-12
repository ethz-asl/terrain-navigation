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
 * @brief Adaptive view utility estimation node
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "adaptive_viewutility/viewutility_map.h"
#include "terrain_navigation/profiler.h"

#include "grid_map_cv/grid_map_cv.hpp"
#include "grid_map_pcl/GridMapPclConverter.hpp"

#include <pcl/io/obj_io.h>
#include <fstream>
#include <iostream>

ViewUtilityMap::ViewUtilityMap(grid_map::GridMap &grid_map) : grid_map_(grid_map) {
  // Initialize gridmap layers
  grid_map_.add("visibility");
  grid_map_.add("geometric_prior");
  grid_map_.add("normalized_prior");
  grid_map_.add("roi");
  grid_map_.add("elevation_normal_x");
  grid_map_.add("elevation_normal_y");
  grid_map_.add("elevation_normal_z");

  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(0);
  grid_map_["normalized_prior"].setConstant(0);
  grid_map_["roi"].setConstant(1.0);
  grid_map_["elevation_normal_x"].setConstant(0);
  grid_map_["elevation_normal_y"].setConstant(0);
  grid_map_["elevation_normal_z"].setConstant(1);
}

ViewUtilityMap::~ViewUtilityMap() {}

double ViewUtilityMap::getBestJointPrior(const std::vector<GeometricPrior> &prior_list) {
  double best_prior{0.0};

  // Iterate through the list of priors to get the maximum geometric_prior
  for (auto prior : prior_list) {
    if (prior.joint > best_prior) best_prior = prior.joint;
  }
  return best_prior;
}

double ViewUtilityMap::CalculateViewUtility(std::vector<ViewPoint> &viewpoint_set, bool update_utility_map) {
  double view_utility = 0.0;
  // Copy cell information to append list with candidate views
  /// TODO: making deep copies of this map is not a good idea, but works for now
  std::vector<CellInfo> candidate_cell_information = cell_information_;
  grid_map::GridMap candidate_grid_map = grid_map_;
  for (auto &viewpoint : viewpoint_set) {
    double utility = CalculateViewUtility(viewpoint, true, candidate_cell_information, candidate_grid_map);
    view_utility += utility;
  }
  return view_utility;
}

grid_map::Polygon ViewUtilityMap::getVisibilityPolygon(ViewPoint &viewpoint, grid_map::GridMap &grid_map) {
  grid_map::Polygon polygon;
  double max_distance = 1000.0;

  Eigen::Vector3d viewpoint_center_local = viewpoint.getCenterLocal();

  polygon.setFrameId(grid_map.getFrameId());
  for (auto ray : viewpoint.getCornerRayVectors()) {
    /// TODO: Do something about ray tracing steps
    ray.normalize();
    for (double t = 0.0; t < max_distance; t += 1.0) {
      Eigen::Vector3d ray_point = viewpoint_center_local + t * ray;
      Eigen::Vector2d ray_point_2d(ray_point(0), ray_point(1));
      double elevation{0.0};
      if (grid_map.isInside(ray_point_2d)) {
        elevation = grid_map.atPosition("elevation", ray_point_2d);
      }
      if (elevation > ray_point(2)) {
        Eigen::Vector3d cell_pos;
        grid_map::Index idx;
        if (grid_map_.isInside(ray_point_2d)) {
          grid_map.getIndex(ray_point_2d, idx);
          grid_map.getPosition3("elevation", idx, cell_pos);
          polygon.addVertex(grid_map::Position(cell_pos(0), cell_pos(1)));
        } else {
          polygon.addVertex(grid_map::Position(ray_point_2d(0), ray_point_2d(1)));
        }
        break;
      }
    }
  }

  return polygon;
}

double ViewUtilityMap::CalculateViewUtility(ViewPoint &viewpoint, bool update_utility_map,
                                            std::vector<CellInfo> &cell_information, grid_map::GridMap &grid_map) {
  double view_utility = 0.0;

  grid_map::Matrix &terrain_data = grid_map["elevation"];

  grid_map::Polygon polygon = getVisibilityPolygon(viewpoint, grid_map);
  if (polygon.nVertices() == 4) {
    Eigen::Vector3d viewpoint_center_local = viewpoint.getCenterLocal();
    int width = grid_map.getSize()(0);
    int view_index = viewpoint.getIndex();
    grid_map::Matrix &layer_visibility = grid_map["visibility"];
    grid_map::Matrix &layer_geometricprior = grid_map["geometric_prior"];
    grid_map::Matrix &layer_utility = grid_map["normalized_prior"];
    for (grid_map::PolygonIterator iterator(grid_map, polygon); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);

      /// No need to run any calculations if is not part of ROI
      if (grid_map.at("roi", index) < 0.5) continue;

      int idx = index(0) + width * index(1);  // Index number of the gridcell

      // Get position of the gridcell
      /// TODO: Treat terrain height as a distribution
      Eigen::Vector3d cell_pos;
      grid_map.getPosition3("elevation", index, cell_pos);

      Eigen::Vector3d cell_normal{Eigen::Vector3d(grid_map.at("elevation_normal_x", index),
                                                  grid_map.at("elevation_normal_y", index),
                                                  grid_map.at("elevation_normal_z", index))};

      // Calculate and store view vectors so that we don't need to repeat the calculation
      Eigen::Vector3d view_vector = (viewpoint_center_local - cell_pos).normalized();
      double view_distance = (viewpoint_center_local - cell_pos).norm();
      std::vector<GeometricPrior> geometric_prior =
          getGeometricPrior(settings_, view_vector, view_distance, cell_normal, cell_information[idx]);

      double best_prior = getBestJointPrior(geometric_prior);
      bool prior_updated = false;
      double area = std::pow(grid_map_.getResolution(), 2);
      /// TODO: This results in over estimation of view utility since it does not account for view
      if (best_prior > layer_geometricprior(index(0), index(1))) {
        prior_updated = true;
        if (best_prior < max_prior_) {
          // TODO: Why are we dividing to 100.0? where does 100 come from?
          view_utility += best_prior - layer_geometricprior(index(0), index(1)) * area / 100.0;
        }
      }

      // Register view into the view utility map
      ViewInfo viewinfo;
      viewinfo.view_vector = view_vector;
      viewinfo.view_distance = view_distance;
      viewinfo.view_index = view_index;
      cell_information[idx].view_info.push_back(viewinfo);

      if (update_utility_map) {
        // Update Visibility count
        layer_visibility(index(0), index(1)) += 1.0;  // Increase visibility count
        // Update Geometric Prior
        if (prior_updated) {
          layer_geometricprior(index(0), index(1)) = best_prior;
          layer_utility(index(0), index(1)) = std::min(best_prior, max_prior_) / max_prior_;
        }
      }
    }

    if (update_utility_map) viewpoint.setUtility(view_utility);
  }

  return view_utility;
}

void ViewUtilityMap::UpdateUtility(ViewPoint &viewpoint) {
  CalculateViewUtility(viewpoint, true, cell_information_, grid_map_);
}

std::vector<GeometricPrior> ViewUtilityMap::getGeometricPrior(const GeometricPriorSettings &settings,
                                                              const Eigen::Vector3d &view_vector_query,
                                                              const double &view_distance,
                                                              const Eigen::Vector3d &cell_normal, CellInfo &cell_info) {
  std::vector<GeometricPrior> prior;
  /// Calculate incidence prior
  double sigma_k = settings.sigma_k;  /// TODO: Parmaeterize
  double reference_view_distance = settings.reference_view_distance;
  double min_angle = settings.min_triangulation_angle;

  double incident_angle = std::acos(view_vector_query.dot(cell_normal));
  double incident_prior = std::exp(-std::pow(incident_angle, 2) / (2 * std::pow(sigma_k, 2)));

  /// Calculate ground sample distance prior
  double relative_gsd = std::pow(reference_view_distance / view_distance, 2);
  double gsd_prior = std::min(relative_gsd, 1 / relative_gsd);

  // Calculate pairwise priors
  int i = 0;
  for (auto view : cell_info.view_info) {
    /// Calculate resolution prior
    /// TODO: Experiment with pairwise resolution prior and resolution vs GSD
    double relative_area = std::pow(view_distance / view.view_distance, 2);
    double resolution_prior = std::min(relative_area, 1 / relative_area);

    // Calculate triangulation prior
    double angle = std::abs(
        std::acos(view_vector_query.dot(view.view_vector)));  /// TODO: Make sure the vectors are all normalized
    double triangulation_prior = 1 - (std::pow((std::min(angle, min_angle) - min_angle), 2) / (min_angle * min_angle));

    GeometricPrior geometric_prior;
    geometric_prior.incident = incident_prior;
    geometric_prior.resolution = resolution_prior;
    geometric_prior.triangulation = triangulation_prior;
    geometric_prior.sample_distance = gsd_prior;
    double joint_geometric_prior = incident_prior * resolution_prior * gsd_prior * triangulation_prior;
    geometric_prior.joint = joint_geometric_prior;

    prior.push_back(geometric_prior);
    i++;
  }
  return prior;
}

void ViewUtilityMap::initializeFromGridmap() {
  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  grid_map::Matrix &layer_normal_x = grid_map_["elevation_normal_x"];
  grid_map::Matrix &layer_normal_y = grid_map_["elevation_normal_y"];
  grid_map::Matrix &layer_normal_z = grid_map_["elevation_normal_z"];

  unsigned width = grid_map_.getSize()(0);
  unsigned height = grid_map_.getSize()(1);
  double resolution = grid_map_.getResolution();
  setCellInformation(width * height);
  // Compute normals from elevation map
  // Surface normal calculation from: https://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    /// TODO: Verify normal by visualization
    int x = gridMapIndex(0);
    int y = height - 1 - gridMapIndex(1);

    float sx = layer_elevation(x < width - 1 ? x + 1 : x, y) - layer_elevation(x > 0 ? x - 1 : x, y);
    if (x == 0 || x == width - 1) sx *= 2;

    float sy = layer_elevation(x, y < height - 1 ? y + 1 : y) - layer_elevation(x, y > 0 ? y - 1 : y);
    if (y == 0 || y == height - 1) sy *= 2;

    Eigen::Vector3d normal(Eigen::Vector3d(sx, sy, 2 * resolution));
    normal.normalize();

    layer_normal_x(x, y) = normal(0);
    layer_normal_y(x, y) = normal(1);
    layer_normal_z(x, y) = normal(2);
  }
  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(0);
  grid_map_["roi"].setConstant(1.0);
  grid_map_["normalized_prior"].setConstant(0);
}

bool ViewUtilityMap::initializeFromGeotiff(GDALDataset *dataset) {
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
  double center_altitude = 1000.0;  /// TODO: Get center altitude as minimum altitude

  // Cadastre
  // double center_latitude = 46.553215;
  // double center_longitude = 6.682505;
  // double center_altitude = 800; /// TODO: Get center altitude as minimum altitude

  // this->setOrigin(center_latitude, center_longitude, center_altitude);

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
  grid_map_.setFrameId("map");
  grid_map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_.add("elevation");

  GDALRasterBand *elevationBand = dataset->GetRasterBand(1);

  std::vector<float> data(width * height, 0.0f);
  elevationBand->RasterIO(GF_Read, 0, 0, width, height, &data[0], width, height, GDT_Float32, 0, 0);

  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  grid_map::Matrix &layer_normal_x = grid_map_["elevation_normal_x"];
  grid_map::Matrix &layer_normal_y = grid_map_["elevation_normal_y"];
  grid_map::Matrix &layer_normal_z = grid_map_["elevation_normal_z"];

  setCellInformation(lengthX * lengthY);

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    int x = gridMapIndex(0);
    int y = height - 1 - gridMapIndex(1);

    layer_elevation(x, y) = data[gridMapIndex(0) + width * gridMapIndex(1)] - center_altitude;
  }

  // Compute normals from elevation map
  // Surface normal calculation from: https://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    /// TODO: Verify normal by visualization
    int x = gridMapIndex(0);
    int y = height - 1 - gridMapIndex(1);

    float sx = layer_elevation(x < width - 1 ? x + 1 : x, y) - layer_elevation(x > 0 ? x - 1 : x, y);
    if (x == 0 || x == width - 1) sx *= 2;

    float sy = layer_elevation(x, y < height - 1 ? y + 1 : y) - layer_elevation(x, y > 0 ? y - 1 : y);
    if (y == 0 || y == height - 1) sy *= 2;

    Eigen::Vector3d normal(Eigen::Vector3d(sx, sy, 2 * resolution));
    normal.normalize();

    layer_normal_x(x, y) = normal(0);
    layer_normal_y(x, y) = normal(1);
    layer_normal_z(x, y) = normal(2);
  }
  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(0);
  grid_map_["roi"].setConstant(0);
  grid_map_["normalized_prior"].setConstant(0);

  return true;
}

bool ViewUtilityMap::initializeFromMesh(const std::string &path, const double res) {
  pcl::PolygonMesh mesh;
  pcl::io::loadOBJFile(path, mesh);
  grid_map::GridMapPclConverter::initializeFromPolygonMesh(mesh, res, grid_map_);
  grid_map::GridMapPclConverter::addLayerFromPolygonMesh(mesh, "elevation", grid_map_);

  double width = grid_map_.getSize()(0);
  double height = grid_map_.getSize()(1);
  double resolution = grid_map_.getResolution();

  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  grid_map::Matrix &layer_normal_x = grid_map_["elevation_normal_x"];
  grid_map::Matrix &layer_normal_y = grid_map_["elevation_normal_y"];
  grid_map::Matrix &layer_normal_z = grid_map_["elevation_normal_z"];

  setCellInformation(width * height);

  // Compute normals from elevation map
  // Surface normal calculation from: https://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    /// TODO: Verify normal by visualization
    int x = gridMapIndex(0);
    int y = grid_map_.getSize()(1) - 1 - gridMapIndex(1);

    float sx = layer_elevation(x < width - 1 ? x + 1 : x, y) - layer_elevation(x > 0 ? x - 1 : x, y);
    if (x == 0 || x == width - 1) sx *= 2;

    float sy = layer_elevation(x, y < height - 1 ? y + 1 : y) - layer_elevation(x, y > 0 ? y - 1 : y);
    if (y == 0 || y == height - 1) sy *= 2;

    Eigen::Vector3d normal(Eigen::Vector3d(sx, sy, 2 * resolution));
    normal.normalize();

    layer_normal_x(x, y) = normal(0);
    layer_normal_y(x, y) = normal(1);
    layer_normal_z(x, y) = normal(2);
  }
  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(0);
  grid_map_["roi"].setConstant(0);
  grid_map_["normalized_prior"].setConstant(0);
}

bool ViewUtilityMap::initializeEmptyMap() {
  grid_map_.setGeometry(grid_map::Length(100.0, 100.0), 10.0, grid_map::Position(0.0, 0.0));
  // Initialize gridmap layers
  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(0);
  grid_map_["elevation"].setConstant(1);
  grid_map_["roi"].setConstant(0);
  grid_map_["elevation_normal_x"].setConstant(0);
  grid_map_["elevation_normal_y"].setConstant(0);
  grid_map_["elevation_normal_z"].setConstant(1);

  // Initialize cell information
  double width = grid_map_.getSize()(0);
  double height = grid_map_.getSize()(1);
  setCellInformation(width * height);
}

void ViewUtilityMap::OutputMapData(const std::string path) {
  // Write data to files

  std::ofstream output_file;
  output_file.open(path);
  output_file << "geometric_prior,visibility,\n";

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);

    output_file << grid_map_.at("geometric_prior", index) << ",";
    output_file << grid_map_.at("visibility", index) << ",";
    output_file << "\n";
  }

  output_file.close();
  return;
}

void ViewUtilityMap::SetRegionOfInterest(const grid_map::Polygon &polygon) {
  for (grid_map::PolygonIterator iterator(grid_map_, polygon); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map_.at("roi", index) = 1.0;
  }
  return;
}

void ViewUtilityMap::CompareMapLayer(const std::string &layer, const grid_map::GridMap &reference_map) {
  grid_map_.add("elevation_difference");
  grid_map_["elevation_difference"].setConstant(0.0);
  std::vector<double> error_vector;
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    Eigen::Vector3d cell_pos_3d;
    grid_map_.getPosition3("elevation", index, cell_pos_3d);
    double cell_elevation = cell_pos_3d(2);
    const Eigen::Vector2d cell_pos = Eigen::Vector2d(cell_pos_3d(0), cell_pos_3d(1));
    if (reference_map.isInside(cell_pos) && (cell_pos.norm() > 0.0001)) {
      double ref_elevation = reference_map.atPosition("elevation", cell_pos);
      double error = std::abs(ref_elevation - cell_elevation);
      grid_map_.at("elevation_difference", index) = error;
      error_vector.push_back(error);
    }
  }
  double cumulative_error{0.0};
  for (auto error_point : error_vector) {
    cumulative_error += error_point;
  }
  double mean = cumulative_error / error_vector.size();
  double cumulative_squared_error{0.0};
  for (auto error_point : error_vector) {
    cumulative_squared_error += std::pow(error_point - mean, 2);
  }
  double stdev = std::sqrt(cumulative_squared_error / error_vector.size());

  std::cout << "- Average Error: " << mean << std::endl;
  std::cout << "- Average STDEV: " << stdev << std::endl;
}

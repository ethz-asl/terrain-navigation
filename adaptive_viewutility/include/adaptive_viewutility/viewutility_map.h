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

#ifndef VIEWUTILITY_MAP_H
#define VIEWUTILITY_MAP_H

#include "terrain_navigation/viewpoint.h"

#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <Eigen/Dense>

struct ViewInfo {
  int view_index{0};
  Eigen::Vector3d view_vector{Eigen::Vector3d::Zero()};
  double view_distance{-1.0};
};
constexpr double limit_cramerrao_bounds{1.0};
struct CellInfo {
  std::vector<ViewInfo> view_info;
  Eigen::Matrix3d fisher_information{Eigen::Matrix3d::Zero()};
  double min_eigen_value{0.0};
  double max_cramerrao_bounds{limit_cramerrao_bounds};
};

struct GeometricPriorSettings {
  double reference_view_distance{100.0};
  double sigma_k{45.0 / 180.0 * M_PI};
  double min_triangulation_angle{0.083 * M_PI};
};

class ViewUtilityMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ViewUtilityMap(grid_map::GridMap &grid_map);
  virtual ~ViewUtilityMap();
  void setGridMap(grid_map::GridMap &map) { grid_map_ = map; };
  void setCellInformation(int map_size) { cell_information_.resize(map_size); };
  void setGeometricPriorSettings(const GeometricPriorSettings &settings) { settings_ = settings; }
  grid_map::GridMap &getGridMap() { return grid_map_; };
  std::vector<CellInfo> getCellInfo() { return cell_information_; };
  void UpdateUtility(ViewPoint &viewpoint) { CalculateViewUtility(viewpoint, true, cell_information_, grid_map_); };
  void OutputMapData(const std::string path);
  double CalculateViewUtility(std::vector<ViewPoint> &viewpoint_set, bool update_utility_map);

  /**
   * @brief Calculate Incident Prior
   *
   * @param unit_view_vector unit vector of camera viewpoint from cell position
   * @param cell_normal cell normal unit vector
   * @param sigma_k constant factor defining incident angle sensitivity
   * @todo This function is not robust against non-unit vectors being used as parameters
   * @return double incident prior
   */
  static double getIncidentPrior(const Eigen::Vector3d &unit_view_vector, const Eigen::Vector3d &cell_normal,
                                 double sigma_k);

  /**
   * @brief Calculate Fisher information matrix of a single view
   *
   * @param bearing_vector Bearing vector of a landmark from camera
   * @param view_distance Distance to the landmark from camera
   * @param sigma Standard Deviation of the bearing vector measurements
   * @return Eigen::Matrix3d Fisher information matrix of a viewpoint
   */
  static Eigen::Matrix3d getFisherInformationMatrix(const Eigen::Vector3d &bearing_vector, const double &view_distance,
                                                    const double sigma);
  void initializeFromGridmap();

  /**
   * @brief Initialize View Utiltiy Map from Mesh
   *
   * @param path path to the mesh file
   * @param res [m] resoultion of the map
   * @return true successfully loaded meshfile
   * @return false mesh loading was unsuccessful
   */
  bool initializeFromMesh(const std::string &path, const double res = 10.0);

  /**
   * @brief Initialize Empty ViewUtility Map
   *
   * @return true successfully loaded meshfile
   * @return false mesh loading was unsuccessful
   */
  bool initializeEmptyMap();
  void SetRegionOfInterest(const grid_map::Polygon &polygon);
  void CompareMapLayer(grid_map::GridMap &reference_map);
  double CalculatePrecision(const std::vector<double> &error_vector, const double threshold);
  void TransformMap(const Eigen::Vector3d &translation);

 protected:
  double CalculateViewUtility(ViewPoint &viewpoint, bool update_utility_map, std::vector<CellInfo> &cell_information,
                              grid_map::GridMap &grid_map);
  grid_map::Polygon getVisibilityPolygon(ViewPoint &viewpoint, grid_map::GridMap &grid_map);

  grid_map::GridMap &grid_map_;
  double max_prior_{0.5};
  GeometricPriorSettings settings_;

 private:
  std::vector<double> calculateErrors(grid_map::GridMap &groundtruth_map, const grid_map::GridMap &reference_map);
  std::vector<CellInfo> cell_information_;
};
#endif

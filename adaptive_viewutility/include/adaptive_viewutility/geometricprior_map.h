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

/**
 * @brief Map using spherical coverage
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */
#ifndef GEOMETRICPRIOR_MAP_H
#define GEOMETRICPRIOR_MAP_H

#include "adaptive_viewutility/viewutility_map.h"

struct GeometricPrior {
  double triangulation{0.0};
  double resolution{0.0};
  double incident{0.0};
  double sample_distance{0.0};
  double joint{0.0};
};

class GeometricPriorMap : public ViewUtilityMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GeometricPriorMap(grid_map::GridMap &grid_map);
  virtual ~GeometricPriorMap();

  /**
   * @brief Calculate view utility of the viewpoint
   *
   * @param viewpoint
   * @param update_utility_map
   * @param cell_information
   * @param grid_map
   * @return double view utility
   */
  double CalculateViewUtility(ViewPoint &viewpoint, bool update_utility_map, std::vector<CellInfo> &cell_information,
                              grid_map::GridMap &grid_map);

 private:
  /**
   * @brief Get the Geometric Prior object
   *
   * @param settings Geometric prior configuration
   * @param view_vector_query Unit vector that is queried for calculating the geometric prior
   * @param view_distance Distance of the viewpoint
   * @param cell_normal Unit vector of cell normal
   * @param cell_info List containing view information of each cell
   * @return std::vector<GeometricPrior>
   */
  static std::vector<GeometricPrior> getGeometricPrior(const GeometricPriorSettings &settings,
                                                       const Eigen::Vector3d &view_vector_query,
                                                       const double &view_distance, const Eigen::Vector3d &cell_normal,
                                                       const Eigen::Vector3d &center_ray, CellInfo &cell_info);

  /**
   * @brief Get the Best Geometric Prior object
   *
   * @param prior_list
   * @return GeometricPrior
   */
  static GeometricPrior getBestGeometricPrior(const std::vector<GeometricPrior> &prior_list);

  /**
   * @brief
   *
   * @param prior_list
   * @return double
   */
  static double getBestJointPrior(const std::vector<GeometricPrior> &prior_list);

  /**
   * @brief Calculate Groundsample prior
   *
   * @param bearing_vector
   * @param optical_center
   * @param reference_view_distance
   * @return double
   */
  static double getGroundSamplePrior(const Eigen::Vector3d &bearing_vector, const Eigen::Vector3d &optical_center,
                                     const double reference_view_distance);

  /**
   * @brief Get the Best Ground Sample Distance object
   *
   * @param prior_list
   * @return double
   */
  static double getBestGroundSampleDistance(const std::vector<GeometricPrior> &prior_list);
};
#endif

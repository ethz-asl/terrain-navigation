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
#ifndef SPHERICALCOVERAGE_MAP_H
#define SPHERICALCOVERAGE_MAP_H

#include "adaptive_viewutility/viewutility_map.h"

class SphericalCoverageMap : public ViewUtilityMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SphericalCoverageMap(grid_map::GridMap &grid_map);
  virtual ~SphericalCoverageMap();

  /**
   * @brief Helper function to evaluate coverage with hemisphere radials
   *
   * @param view unit vector of view point
   * @param sample unit vector of hemisphere sample
   * @param distance distance to view point
   * @return true inside hemisphere cover
   * @return false not inside hemisphere cover
   */
  static bool hemisphereInside(const Eigen::Vector3d &view, const Eigen::Vector3d &sample, const double distance) {
    double theta_max{0.5 * 0.25 * M_PI};
    double t0 = 100.0;
    double t_half = 30.0;
    double angle = std::acos(sample.dot(view));  // Angle between view sample and sample
    double radius = theta_max * std::pow(2.0, -std::max(distance - t0, 0.0) / t_half);
    return bool(angle < radius);  // Sample is inside the radius of a view
  }

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
};
#endif

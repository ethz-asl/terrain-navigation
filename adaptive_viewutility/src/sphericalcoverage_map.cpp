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
#include "adaptive_viewutility/sphericalcoverage_map.h"

SphericalCoverageMap::SphericalCoverageMap(grid_map::GridMap &grid_map) : ViewUtilityMap(grid_map) {}

SphericalCoverageMap::~SphericalCoverageMap() {}

double SphericalCoverageMap::CalculateViewUtility(ViewPoint &viewpoint, bool update_utility_map,
                                                  std::vector<CellInfo> &cell_information,
                                                  grid_map::GridMap &grid_map) {
  double view_utility = 0.0;

  grid_map::Matrix &terrain_data = grid_map["elevation"];

  grid_map::Polygon polygon = getVisibilityPolygon(viewpoint, grid_map);
  if (polygon.nVertices() == 4) {
    Eigen::Vector3d viewpoint_center_local = viewpoint.getCenterLocal();
    int width = grid_map.getSize()(0);
    int view_index = viewpoint.getIndex();
    grid_map::Matrix &layer_visibility = grid_map["visibility"];
    grid_map::Matrix &layer_geometricprior = grid_map["geometric_prior"];
    grid_map::Matrix &layer_sample_distance = grid_map["ground_sample_distance"];
    grid_map::Matrix &layer_incident_prior = grid_map["incident_prior"];
    grid_map::Matrix &layer_triangulation_prior = grid_map["triangulation_prior"];
    grid_map::Matrix &layer_utility = grid_map["normalized_prior"];
    grid_map::Matrix &layer_min_eigen_value = grid_map["min_eigen_value"];
    for (grid_map::PolygonIterator iterator(grid_map, polygon); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      int idx = index(0) + width * index(1);  // Index number of the gridcell
      if (cell_information.size() <= idx) {
        /// TODO: This is a workaround since the polygon iterator is not checking for map bounds
        continue;
      }
      /// No need to run any calculations if is not part of ROI
      if (grid_map.at("roi", index) < 0.5) continue;

      // Get position of the gridcell
      /// TODO: Treat terrain height as a distribution
      Eigen::Vector3d cell_pos;
      grid_map.getPosition3("elevation", index, cell_pos);

      Eigen::Vector3d cell_normal{Eigen::Vector3d(grid_map.at("elevation_normal_x", index),
                                                  grid_map.at("elevation_normal_y", index),
                                                  grid_map.at("elevation_normal_z", index))};

      // Calculate and store view vectors so that we don't need to repeat the calculation
      Eigen::Vector3d view_vector = viewpoint_center_local - cell_pos;
      Eigen::Vector3d view_vector_u = view_vector.normalized();
      double view_distance = view_vector.norm();
      Eigen::Vector3d optical_center = viewpoint.getCenterRayVector();
      double cell_area = std::pow(grid_map_.getResolution(), 2);

      /// Generate discrete samples
      /// Roberts 2017 generate samples through http://blog.marmakoide.org/?p=1
      const int K = 256;
      std::vector<Eigen::Vector3d> hemisphere_samples(K);
      double golden_angle = M_PI * (3.0 - std::sqrt(5.0));
      for (int i = 0; i < hemisphere_samples.size(); i++) {
        double theta = golden_angle * i;
        double z = std::abs((1.0 - 1.0 / double(K)) * (1.0 - 2.0 * i / (double(K) - 1.0)));
        double radius = sqrt(1.0 - std::pow(z, 2));
        hemisphere_samples[i](0) = radius * std::cos(theta);
        hemisphere_samples[i](1) = radius * std::sin(theta);
        hemisphere_samples[i](2) = z;
      }

      double F_c{0.0};
      for (auto &sample : hemisphere_samples) {
        double w_h = sample.dot(cell_normal);
        double vj = 0.0;  // Coverage indicator function
        for (auto view : cell_information[idx].view_info) {
          if (hemisphereInside(view.view_vector, sample, view.view_distance)) {
            vj = 1.0;
            break;  // If the sample is in the covered region, no need to iterate
          }
        }
        vj = hemisphereInside(view_vector_u, sample, view_distance) ? 1.0 : vj;
        vj = (w_h < 0.0) ? 0.0 : vj;
        F_c += (2 * M_PI / static_cast<double>(K)) * w_h * vj;
      }
      if (F_c > layer_geometricprior(index(0), index(1))) {
        // This condition should be redundant, but numerically unstable
        /// TODO: Investigate why this is numerically unstable
        view_utility += F_c - layer_geometricprior(index(0), index(1));
      }

      ViewInfo viewinfo;
      viewinfo.view_vector = view_vector_u;
      viewinfo.view_distance = view_distance;
      viewinfo.view_index = view_index;
      cell_information[idx].view_info.push_back(viewinfo);

      if (update_utility_map) {
        layer_geometricprior(index(0), index(1)) = F_c;
        layer_utility(index(0), index(1)) = F_c;
      }

      if (update_utility_map) {
        // Update Visibility count
        /// TODO: This increases the visibility count even though it is very far away
        layer_visibility(index(0), index(1)) += 1.0;  // Increase visibility count
      }
    }

    if (update_utility_map) viewpoint.setUtility(view_utility);
  }
}

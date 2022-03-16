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
#include "adaptive_viewutility/geometricprior_map.h"

GeometricPriorMap::GeometricPriorMap(grid_map::GridMap &grid_map) : ViewUtilityMap(grid_map) {}

GeometricPriorMap::~GeometricPriorMap() {}

double GeometricPriorMap::CalculateViewUtility(ViewPoint &viewpoint, bool update_utility_map,
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

      std::vector<GeometricPrior> geometric_prior = getGeometricPrior(
          settings_, view_vector_u, view_distance, cell_normal, optical_center, cell_information[idx]);

      double best_prior = getBestJointPrior(geometric_prior);
      GeometricPrior best_geometric_prior = getBestGeometricPrior(geometric_prior);
      double best_sample_distance = getBestGroundSampleDistance(geometric_prior);
      bool prior_updated = false;
      /// TODO: This results in over estimation of view utility since it does not account for view
      if (best_prior > layer_geometricprior(index(0), index(1))) {
        prior_updated = true;
        if (best_prior < max_prior_) {
          // TODO: Why are we dividing to 100.0? where does 100 come from?
          view_utility += best_prior - layer_geometricprior(index(0), index(1)) * cell_area / 100.0;
        }
      }

      // Register view into the view utility map
      ViewInfo viewinfo;
      viewinfo.view_vector = view_vector_u;
      viewinfo.view_distance = view_distance;
      viewinfo.view_index = view_index;
      cell_information[idx].view_info.push_back(viewinfo);

      if (update_utility_map) {
        // Update Geometric Prior
        if (prior_updated) {
          layer_geometricprior(index(0), index(1)) = best_prior;
          layer_sample_distance(index(0), index(1)) = best_sample_distance;
          layer_utility(index(0), index(1)) = std::min(best_prior, max_prior_) / max_prior_;
          layer_sample_distance(index(0), index(1)) = best_geometric_prior.resolution;
          layer_incident_prior(index(0), index(1)) = best_geometric_prior.incident;
          layer_triangulation_prior(index(0), index(1)) = best_geometric_prior.triangulation;
        }
      }
    }

    if (update_utility_map) viewpoint.setUtility(view_utility);
  }
}

std::vector<GeometricPrior> GeometricPriorMap::getGeometricPrior(
    const GeometricPriorSettings &settings, const Eigen::Vector3d &view_vector_query, const double &view_distance,
    const Eigen::Vector3d &cell_normal, const Eigen::Vector3d &center_ray, CellInfo &cell_info) {
  std::vector<GeometricPrior> prior;
  /// Calculate incidence prior
  double min_angle = settings.min_triangulation_angle;

  double sigma_k = settings.sigma_k;
  double incident_prior = getIncidentPrior(view_vector_query, cell_normal, sigma_k);

  /// Calculate ground sample distance prior
  double reference_view_distance = settings.reference_view_distance;
  Eigen::Vector3d bearing_vector = -view_distance * view_vector_query;
  double gsd_prior = getGroundSamplePrior(bearing_vector, center_ray, reference_view_distance);

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
    double triangulation_prior =
        1.0 - (std::pow((std::min(angle, min_angle) - min_angle), 2) / (min_angle * min_angle));

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

GeometricPrior GeometricPriorMap::getBestGeometricPrior(const std::vector<GeometricPrior> &prior_list) {
  double best_prior{0.0};
  GeometricPrior best_prior_info;

  // Iterate through the list of priors to get the maximum geometric_prior
  for (auto &prior : prior_list) {
    if (prior.joint > best_prior) {
      best_prior = prior.joint;
      best_prior_info = prior;
    }
  }
  return best_prior_info;
}

double GeometricPriorMap::getBestJointPrior(const std::vector<GeometricPrior> &prior_list) {
  double best_prior{0.0};

  // Iterate through the list of priors to get the maximum geometric_prior
  for (auto prior : prior_list) {
    if (prior.joint > best_prior) best_prior = prior.joint;
  }
  return best_prior;
}

double GeometricPriorMap::getGroundSamplePrior(const Eigen::Vector3d &bearing_vector,
                                               const Eigen::Vector3d &optical_center,
                                               const double reference_view_distance) {
  double projected_distance = (bearing_vector).dot(optical_center);
  double relative_gsd = std::pow(reference_view_distance / projected_distance, 2);
  double gsd_prior = std::min(relative_gsd, 1 / relative_gsd);
  return gsd_prior;
}

double GeometricPriorMap::getBestGroundSampleDistance(const std::vector<GeometricPrior> &prior_list) {
  double best_sample_distance{std::numeric_limits<double>::infinity()};

  // Iterate through the list of priors to get the maximum geometric_prior
  for (auto prior : prior_list) {
    if ((prior.sample_distance < best_sample_distance) && (best_sample_distance > 0))
      best_sample_distance = prior.sample_distance;
  }
  return best_sample_distance;
}

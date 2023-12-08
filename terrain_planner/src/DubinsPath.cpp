/*!
 * \file DubinsPath.cpp
 *
 * \brief Describes a path between two states.
 *
 *  Created on: Nov 4, 2016
 *      Author: Daniel Schneider, ASL
 *              Florian Achermann, ASL
 */

#include "terrain_planner/DubinsPath.hpp"

#include <assert.h>

#include <boost/math/constants/constants.hpp>
#include <cmath>

namespace fw_planning {

namespace spaces {

const double TWO_PI = boost::math::constants::two_pi<double>();

const DubinsPath::DubinsPathSegmentType DubinsPath::dubinsPathType[6][3] = {
    {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT},  {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT},
    {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT}, {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT},
    {DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT},    {DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}};

DubinsPath::DubinsPath(Index type, double t, double p, double q, double gam, unsigned int ks, unsigned int ke, double r)
    : length_{{0, t, 0, p, q, 0}},
      length_2D_(t + p + q),
      radiusRatio_{{r, r, r, r, r, r}},
      radiusRatioInverse_{{1.0 / r, 1.0 / r, 1.0 / r, 1.0 / r, 1.0 / r, 1.0 / r}},
      gamma_(gam),
      one_div_cos_abs_gamma_(1.0 / cosf(fabs(gamma_))),
      k_end_(ke),
      k_start_(ks),
      classification_(NOT_ASSIGNED),
      idx_(type),
      lmh_(DubinsPath::ALT_CASE_LOW),
      additionalManeuver_(false),
      foundOptimalPath_(true) {
  // only 6 different types available
  assert(type < 6u);
  type_ = dubinsPathType[type];

  /* by minimum radius (rho_) normalized length of projection of helix at the start on x-y plane
   * by minimum radius (rho_) normalized length of projection of first segment on x-y plane (first Dubins car path
   * segment) by minimum radius (rho_) normalized length of projection of intermediate segment on x-y plane by minimum
   * radius (rho_) normalized length of projection of second segment on x-y plane (second Dubins car path segment) by
   * minimum radius (rho_) normalized length of projection of third segment on x-y plane (third Dubins car path segment)
   * by minimum radius (rho_) normalized length of projection of helix at the end on x-y plane */
  assert(t >= 0.0);
  assert(p >= 0.0 || isnan(p));
  assert(q >= 0.0);
}

double DubinsPath::length_2D() const { return length_2D_; }

double DubinsPath::length_3D() const { return length_2D() * one_div_cos_abs_gamma_; }

bool DubinsPath::getFoundOptimalPath() const { return foundOptimalPath_; }

void DubinsPath::setFoundOptimalPath(bool found_optimal_path) { foundOptimalPath_ = found_optimal_path; }

bool DubinsPath::getAdditionalManeuver() const { return additionalManeuver_; }

void DubinsPath::setAdditionalManeuver(bool additional_maneuver) { additionalManeuver_ = additional_maneuver; }

DubinsPath::AltitudeCase DubinsPath::getAltitudeCase() const { return lmh_; }

void DubinsPath::setAltitudeCase(DubinsPath::AltitudeCase altitude_case) { lmh_ = altitude_case; }

DubinsPath::Index DubinsPath::getIdx() const { return idx_; }

void DubinsPath::setClassification(DubinsPath::Classification classification) { classification_ = classification; }

DubinsPath::Classification DubinsPath::getClassification() const { return classification_; }

void DubinsPath::setStartHelix(unsigned int num_helix, double radius_ratio) {
  k_start_ = num_helix;
  radiusRatio_[0] = radius_ratio;
  radiusRatioInverse_[0] = 1.0 / radius_ratio;
  length_[0] = num_helix * TWO_PI * radius_ratio;
  length_2D_ = length_[0] + length_[1] + length_[2] + length_[3] + length_[4] + length_[5];
}

void DubinsPath::setEndHelix(unsigned int num_helix, double radius_ratio) {
  k_end_ = num_helix;
  radiusRatio_[5] = radius_ratio;
  radiusRatioInverse_[5] = 1.0 / radius_ratio;
  length_[5] = num_helix * TWO_PI * radius_ratio;
  length_2D_ = length_[0] + length_[1] + length_[2] + length_[3] + length_[4] + length_[5];
}

void DubinsPath::setGamma(double gamma) {
  gamma_ = gamma;
  one_div_cos_abs_gamma_ = 1.0 / cosf(fabs(gamma_));
}

double DubinsPath::getGamma() const { return gamma_; }

double DubinsPath::getRadiusRatio(unsigned int idx) const { return radiusRatio_[idx]; }

double DubinsPath::getInverseRadiusRatio(unsigned int idx) const { return radiusRatioInverse_[idx]; }

double DubinsPath::getSegmentLength(unsigned int idx) const { return length_[idx]; }

void DubinsPath::setSegmentLength(double length, unsigned int idx) {
  length_[idx] = length;
  length_2D_ = length_[0] + length_[1] + length_[2] + length_[3] + length_[4] + length_[5];
}

const DubinsPath::DubinsPathSegmentType* DubinsPath::getType() const { return type_; }

}  // namespace spaces

}  // namespace fw_planning

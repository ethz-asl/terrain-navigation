/*
 * DubinsAirplane.cpp
 *
 *  Created on: Jun 4, 2015
 *      Author: Daniel Schneider, ASL
 *              Florian Achermann, ASL
 *              Philipp Oetthershagen, ASL
 *
 *      Note: See header file for more information on definitions and states.
 */

#include "terrain_planner/DubinsAirplane.hpp"

#include <assert.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <boost/math/constants/constants.hpp>

namespace ob = ompl::base;

namespace fw_planning {

namespace spaces {

// constants
const double half_pi = boost::math::constants::half_pi<double>();
const double pi = boost::math::constants::pi<double>();
const double twopi = boost::math::constants::two_pi<double>();
const double one_div_twopi = boost::math::constants::one_div_two_pi<double>();

const double DUBINS_EPS = 1e-2;
const double DUBINS_ZERO = -1e-4;

/** \brief mod2pi
 * Sets the input angle to the corresponding angle between 0 and 2pi.
 * TODO Move it to a general file as this function can be used in many different functions/classes
 */
inline double mod2pi(double x) {
  if (x < 0 && x > DUBINS_ZERO) return 0;

  return x - twopi * floor(x * one_div_twopi);
}

/** \brief sgn
 * Returns +1 for positive sign, -1 for negative sign and 0 if val=0
 * TODO Move it to a general file as this function can be used in many different functions/classes
 */
template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

/*-----------------------------------------------------------------------------------------------*/
/*- StateType: Public Functions------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------*/

DubinsAirplaneStateSpace::StateType::StateType() : ob::CompoundStateSpace::StateType() {}

double DubinsAirplaneStateSpace::StateType::getX() const {
  return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
}

double DubinsAirplaneStateSpace::StateType::getY() const {
  return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
}

double DubinsAirplaneStateSpace::StateType::getZ() const {
  return as<ob::RealVectorStateSpace::StateType>(0)->values[2];
}

double DubinsAirplaneStateSpace::StateType::getYaw() const { return as<ob::SO2StateSpace::StateType>(1)->value; }

void DubinsAirplaneStateSpace::StateType::setX(double x) { as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x; }

void DubinsAirplaneStateSpace::StateType::setY(double y) { as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y; }

void DubinsAirplaneStateSpace::StateType::setZ(double z) { as<ob::RealVectorStateSpace::StateType>(0)->values[2] = z; }

void DubinsAirplaneStateSpace::StateType::setYaw(double yaw) { as<ob::SO2StateSpace::StateType>(1)->value = yaw; }

void DubinsAirplaneStateSpace::StateType::setXYZ(double x, double y, double z) {
  setX(x);
  setY(y);
  setZ(z);
}

void DubinsAirplaneStateSpace::StateType::setXYZYaw(double x, double y, double z, double yaw) {
  setX(x);
  setY(y);
  setZ(z);
  setYaw(yaw);
}

void DubinsAirplaneStateSpace::StateType::addToX(double val) {
  as<ob::RealVectorStateSpace::StateType>(0)->values[0] += val;
}

void DubinsAirplaneStateSpace::StateType::addToY(double val) {
  as<ob::RealVectorStateSpace::StateType>(0)->values[1] += val;
}

void DubinsAirplaneStateSpace::StateType::addToZ(double val) {
  as<ob::RealVectorStateSpace::StateType>(0)->values[2] += val;
}

const double* DubinsAirplaneStateSpace::StateType::getPosValuePointer() const {
  return as<ob::RealVectorStateSpace::StateType>(0)->values;
}

void DubinsAirplaneStateSpace::StateType::printState(const std::string& msg) const {
  std::cout << msg << "state (x,y,z,yaw): " << this->getX() << " " << this->getY() << " " << this->getZ() << " "
            << this->getYaw() << std::endl
            << std::endl;
}

/*-----------------------------------------------------------------------------------------------*/
/*- DubinsAirplaneStateSpace: Protected Functions-----------------------------------------------*/
/*-----------------------------------------------------------------------------------------------*/

DubinsAirplaneStateSpace::DubinsAirplaneStateSpace(double turningRadius, double gam, bool useEuclDist)
    : ob::CompoundStateSpace(),
      rho_(turningRadius),
      curvature_(1.0 / turningRadius),
      gammaMax_(gam),
      tanGammaMax_(tan(gam)),
      tanGammaMaxInv_(1.0 / tan(gam)),
      sin_gammaMax_(sin(gam)),
      one_div_sin_gammaMax_(1.0 / sin(gam)),
      optimalStSp_(false),
      dubinsWindPrintXthError_(1000000),
      dp_(),
      useEuclideanDistance_(useEuclDist),
      csc_ctr_(0),
      ccc_ctr_(0),
      long_ctr_(0),
      short_ctr_(0),
      dp_failed_ctr_(0),
      dp_failed_xy_wind_ctr_(0),
      dp_failed_z_wind_ctr_(0),
      dp_success_ctr_(0),
      duration_distance_(0.0),
      duration_interpolate_(0.0),
      duration_interpolate_motionValidator_(0.0),
      duration_get_wind_drift_(0.0),
      interpol_seg_(0.0),
      interpol_tanGamma_(0.0),
      interpol_phiStart_(0.0),
      interpol_dPhi_(0.0),
      interpol_v_(0.0),
      interpol_iter_(0),
      interpol_tmp_(0.0) {
  setName("DubinsAirplane" + getName());
  type_ = ob::STATE_SPACE_SE3;
  addSubspace(ob::StateSpacePtr(new ob::DubinsStateSpace()), 1.);
  addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 1.);
  lock();

  stateInterpolation_ = allocState()->as<StateType>();
}

DubinsAirplaneStateSpace::~DubinsAirplaneStateSpace() { freeState(stateInterpolation_); }

double DubinsAirplaneStateSpace::getMaximumExtent() const {
  /* For the DubinsAirplaneStateSpace this computes:
   * R3_max_extent + SO2_max_extent = sqrtf(bound_x^2 + bound_y^2 + bound_z^2) + pi */
  double e = 0.0;
  for (unsigned int i = 0; i < componentCount_; ++i)
    if (weights_[i] >= std::numeric_limits<double>::epsilon())  // avoid possible multiplication of 0 times infinity
      e += weights_[i] * components_[i]->getMaximumExtent();
  return e;
}

double DubinsAirplaneStateSpace::getEuclideanExtent() const {
  /* For the DubinsAirplaneStateSpace this computes:
   * R3_max_extent = sqrtf(bound_x^2 + bound_y^2 + bound_z^2) */
  return components_[0]->getMaximumExtent();
}

unsigned int DubinsAirplaneStateSpace::validSegmentCount(const ob::State* state1, const ob::State* state2) const {
  if (!useEuclideanDistance_) {
    double dist = distance(state1, state2);
    if (std::isnan(dist))
      return 0u;
    else
      return longestValidSegmentCountFactor_ * (unsigned int)ceil(dist / longestValidSegment_);
  } else {
    // still compute the dubins airplane distance.
    useEuclideanDistance_ = false;
    unsigned int nd =
        longestValidSegmentCountFactor_ * (unsigned int)ceil(distance(state1, state2) / longestValidSegment_);
    useEuclideanDistance_ = true;
    return nd;
  }
}

double DubinsAirplaneStateSpace::distance(const ob::State* state1, const ob::State* state2) const {
  if (useEuclideanDistance_) {
    return euclidean_distance(state1, state2);
  } else {
    dubins(state1, state2, dp_);

    const double dist = rho_ * dp_.length_3D();

    return dist;
  }
}

double DubinsAirplaneStateSpace::euclidean_distance(const ob::State* state1, const ob::State* state2) const {
  const DubinsAirplaneStateSpace::StateType* dubinsAirplane2State1 = state1->as<DubinsAirplaneStateSpace::StateType>();
  const DubinsAirplaneStateSpace::StateType* dubinsAirplane2State2 = state2->as<DubinsAirplaneStateSpace::StateType>();

  double eucl_dist = ((dubinsAirplane2State1->getX() - dubinsAirplane2State2->getX()) *
                      (dubinsAirplane2State1->getX() - dubinsAirplane2State2->getX())) +
                     ((dubinsAirplane2State1->getY() - dubinsAirplane2State2->getY()) *
                      (dubinsAirplane2State1->getY() - dubinsAirplane2State2->getY())) +
                     ((dubinsAirplane2State1->getZ() - dubinsAirplane2State2->getZ()) *
                      (dubinsAirplane2State1->getZ() - dubinsAirplane2State2->getZ()));

  eucl_dist = sqrtf(eucl_dist);

  const double dub_dist = fabs(dubinsAirplane2State1->getZ() - dubinsAirplane2State2->getZ()) * one_div_sin_gammaMax_;

  return std::max(eucl_dist, dub_dist);
}

void DubinsAirplaneStateSpace::interpolate(const ob::State* from, const ob::State* to, const double t,
                                           ob::State* state) const {
  bool firstTime = true;
  DubinsPath path;
  SegmentStarts segmentStarts;
  interpolate(from, to, t, firstTime, path, segmentStarts, state);
}

void DubinsAirplaneStateSpace::calculateSegments(const ob::State* from, const ob::State* to, DubinsPath& path,
                                                 SegmentStarts& segmentStarts) const {
  // compute the path if interpolate is called the first time.
  dubins(from, to, path);
  // compute the segment starts
  calculateSegmentStarts(from, path, segmentStarts);
}

void DubinsAirplaneStateSpace::interpolate(const ob::State* from, const ob::State* to, double t, bool& firstTime,
                                           DubinsPath& path, SegmentStarts& segmentStarts, ob::State* state) const {
  // compute the path if interpolate is called the first time.
  if (firstTime) {
    dubins(from, to, path);

    // compute the segment starts
    calculateSegmentStarts(from, path, segmentStarts);

    // this must be after the computation of the dubins path (otherwise a state of an invalid path is returned.
    if (t >= 1.) {
      if (to != state) copyState(state, to);
      return;
    }
    if (t <= 0.) {
      if (from != state) copyState(state, from);
      return;
    }
    firstTime = false;
  }

  if (std::isnan(path.length_3D())) {
    state->as<StateType>()->setXYZ(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::max());
  } else {
    interpolate(path, segmentStarts, t, state);
  }
}

void DubinsAirplaneStateSpace::enforceBounds(ob::State* state) const {
  // RE3 part
  StateType* rstate = static_cast<StateType*>(state);
  for (unsigned int i = 0; i < getSubspace(0)->getDimension(); ++i) {
    if (rstate->as<ob::RealVectorStateSpace::StateType>(0)->values[i] >
        as<ob::RealVectorStateSpace>(0)->getBounds().high[i])
      rstate->as<ob::RealVectorStateSpace::StateType>(0)->values[i] =
          as<ob::RealVectorStateSpace>(0)->getBounds().high[i];
    else if (rstate->as<ob::RealVectorStateSpace::StateType>(0)->values[i] <
             as<ob::RealVectorStateSpace>(0)->getBounds().low[i])
      rstate->as<ob::RealVectorStateSpace::StateType>(0)->values[i] =
          as<ob::RealVectorStateSpace>(0)->getBounds().low[i];
  }

  // Orientation (S02 part)
  getSubspace(1)->enforceBounds(rstate->as<ob::SO2StateSpace::StateType>(1));
}

void DubinsAirplaneStateSpace::setMaxClimbingAngle(double maxClimb) {
  gammaMax_ = maxClimb;
  tanGammaMax_ = tan(gammaMax_);
  tanGammaMaxInv_ = 1.0 / tanGammaMax_;
  sin_gammaMax_ = sin(gammaMax_);
  one_div_sin_gammaMax_ = 1.0 / sin_gammaMax_;
}

double DubinsAirplaneStateSpace::getMaxClimbingAngle() const { return gammaMax_; }

double DubinsAirplaneStateSpace::getOneDivSinGammaMax() const { return one_div_sin_gammaMax_; }

void DubinsAirplaneStateSpace::setMinTurningRadius(double r_min) {
  rho_ = r_min;
  curvature_ = 1 / rho_;
}

double DubinsAirplaneStateSpace::getMinTurningRadius() const { return rho_; }

double DubinsAirplaneStateSpace::getCurvature() const { return curvature_; }

void DubinsAirplaneStateSpace::setUseOptStSp(bool useOptStSp) { optimalStSp_ = useOptStSp; }

void DubinsAirplaneStateSpace::setUseEuclideanDistance(bool useEuclDist) { useEuclideanDistance_ = useEuclDist; }

void DubinsAirplaneStateSpace::setDubinsWindPrintXthError(int print_xth_error) {
  dubinsWindPrintXthError_ = print_xth_error;
}

bool DubinsAirplaneStateSpace::getUseEuclideanDistance() const { return useEuclideanDistance_; }

bool DubinsAirplaneStateSpace::isMetricSpace() const { return false; }

bool DubinsAirplaneStateSpace::hasSymmetricDistance() const { return false; }

bool DubinsAirplaneStateSpace::hasSymmetricInterpolate() const { return false; }

void DubinsAirplaneStateSpace::sanityChecks() const {
  double zero = std::numeric_limits<double>::epsilon();
  double eps = std::numeric_limits<float>::epsilon();
  int flags = ~(STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND);
  flags &= ~STATESPACE_DISTANCE_SYMMETRIC;
  // don't do the sanity check in case of wind as it takes a long time and therefore blocks the benchmarking
  // we know that for our purpose the functionality is given
  StateSpace::sanityChecks(zero, eps, flags);
}

void DubinsAirplaneStateSpace::setBounds(const ob::RealVectorBounds& bounds) {
  as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}

const ob::RealVectorBounds& DubinsAirplaneStateSpace::getBounds() const {
  return as<ob::RealVectorStateSpace>(0)->getBounds();
}

void DubinsAirplaneStateSpace::printStateSpaceProperties() const {
  std::cout << "DubinsAirplaneStateSpace [" << getName() << "]" << std::endl;
  std::cout << "  !!! This state space is asymmetric !!!" << std::endl;
  std::cout << "  Airplane speed relative to ground:  9 m/s" << std::endl;
  std::cout << "  Euclidean Distance: " << useEuclideanDistance_ << std::endl;
  std::cout << "  Minimum turning radius: " << rho_ << " m" << std::endl;
  std::cout << "  Maximum climbing angle: " << gammaMax_ << " radian" << std::endl;
  std::cout << "  Using optimal Dubins airplane paths (not working properly): " << optimalStSp_ << std::endl;
  std::cout << "  State space bounds (in meter and radian): [" << this->getBounds().low[0] << " "
            << this->getBounds().high[0] << "], [" << this->getBounds().low[1] << " " << this->getBounds().high[1]
            << "], [" << this->getBounds().low[2] << " " << this->getBounds().high[2] << "], ["
            << "-pi pi"
            << ")";
  std::cout << std::endl << std::endl;
}

void DubinsAirplaneStateSpace::printCtrs() const {
  std::cout << "Number samples resulting in a CSC path: " << csc_ctr_
            << " (in %: " << double(csc_ctr_) / double(csc_ctr_ + ccc_ctr_) << " )" << std::endl;
  std::cout << "Number samples resulting in a CCC path: " << ccc_ctr_
            << " (in %: " << double(ccc_ctr_) / double(csc_ctr_ + ccc_ctr_) << " )" << std::endl
            << std::endl;
  std::cout << "Number samples resulting in a long path: " << long_ctr_
            << " (in %: " << double(long_ctr_) / double(long_ctr_ + short_ctr_) << " )" << std::endl;
  std::cout << "Number samples resulting in a short  path: " << short_ctr_
            << " (in %: " << double(short_ctr_) / double(long_ctr_ + short_ctr_) << " )" << std::endl
            << std::endl;

  if ((dp_failed_ctr_ != 0) || (dp_success_ctr_ != 0)) {
    std::cout << "Number of times failed to compute a path in wind:  " << dp_failed_ctr_
              << " (in %: " << double(dp_failed_ctr_) / double(dp_failed_ctr_ + dp_success_ctr_) << " )" << std::endl;
    if (dp_failed_ctr_) {
      std::cout << "    Not converged:                   "
                << dp_failed_ctr_ - dp_failed_xy_wind_ctr_ - dp_failed_z_wind_ctr_ << " (in %: "
                << double(dp_failed_ctr_ - dp_failed_xy_wind_ctr_ - dp_failed_z_wind_ctr_) / double(dp_failed_ctr_)
                << " )" << std::endl;
      std::cout << "    Too strong wind in xy-direction: " << dp_failed_xy_wind_ctr_
                << " (in %: " << double(dp_failed_xy_wind_ctr_) / double(dp_failed_ctr_) << " )" << std::endl;
      std::cout << "    Too strong wind in z-direction:  " << dp_failed_z_wind_ctr_
                << " (in %: " << double(dp_failed_z_wind_ctr_) / double(dp_failed_ctr_) << " )" << std::endl;
    }
    std::cout << "Number of times successfully computed a path in wind:  " << dp_success_ctr_
              << " (in %: " << double(dp_success_ctr_) / double(dp_failed_ctr_ + dp_success_ctr_) << " )" << std::endl
              << std::endl;
  }
}

void DubinsAirplaneStateSpace::printDurations() {
  duration_interpolate_motionValidator_ -= duration_interpolate_;
  std::cout << "DubinsAirplaneStateSpace" << std::endl;
  std::cout << "  total time in secs in distance function: " << duration_distance_ << " s" << std::endl;
  std::cout << "  total time in secs in interpolate function: " << duration_interpolate_ << " s" << std::endl;
  std::cout
      << "  total time in secs in interpolate function called in checkMotion function of the DubinsMotionValidator "
         "class (for DubinsAirplaneStateSpace, this  time is contained in the time of checkMotion): "
      << duration_interpolate_motionValidator_ << " s" << std::endl;
  std::cout << "  total time in secs in calculate wind drift function: " << duration_get_wind_drift_ << " s"
            << std::endl;
  std::cout << std::endl << std::endl;
}

void DubinsAirplaneStateSpace::resetCtrs() {
  csc_ctr_ = 0;
  ccc_ctr_ = 0;
  long_ctr_ = 0;
  short_ctr_ = 0;
  dp_failed_ctr_ = 0;
  dp_failed_xy_wind_ctr_ = 0;
  dp_failed_z_wind_ctr_ = 0;
  dp_success_ctr_ = 0;
}

void DubinsAirplaneStateSpace::resetDurations() {
  duration_distance_ = 0.0;
  duration_interpolate_ = 0.0;
  duration_interpolate_motionValidator_ = 0.0;
  duration_get_wind_drift_ = 0.0;
}

void DubinsAirplaneStateSpace::printDurationsAndCtrs() {
  printDurations();
  printCtrs();
}

void DubinsAirplaneStateSpace::resetDurationsAndCtrs() {
  resetDurations();
  resetCtrs();
}

/*-----------------------------------------------------------------------------------------------*/
/*- DubinsAirplaneStateSpace: Protected Functions-----------------------------------------------*/
/*-----------------------------------------------------------------------------------------------*/

void DubinsAirplaneStateSpace::dubins(const ob::State* state1, const ob::State* state2, DubinsPath& dp) const {
  // extract state 1
  const double x1 = state1->as<DubinsAirplaneStateSpace::StateType>()->getX();
  const double y1 = state1->as<DubinsAirplaneStateSpace::StateType>()->getY();
  const double z1 = state1->as<DubinsAirplaneStateSpace::StateType>()->getZ();
  const double th1 = state1->as<DubinsAirplaneStateSpace::StateType>()->getYaw();

  // extract state 2
  const double x2 = state2->as<DubinsAirplaneStateSpace::StateType>()->getX();
  const double y2 = state2->as<DubinsAirplaneStateSpace::StateType>()->getY();
  const double z2 = state2->as<DubinsAirplaneStateSpace::StateType>()->getZ();
  const double th2 = state2->as<DubinsAirplaneStateSpace::StateType>()->getYaw();

  const double dx = (x2 - x1) * curvature_;
  const double dy = (y2 - y1) * curvature_;
  const double dz = (z2 - z1) * curvature_;
  const double fabs_dz = fabs(dz);

  // compute the 2D path
  auto dubins_path_2d = getSubspace(0)->as<ob::DubinsStateSpace>()->dubins(state1, state2);
  double L = dubins_path_2d.length();

  // set the climbing angle
  if (fabs_dz * tanGammaMaxInv_ <= L) {  // low altitude

    dp.setAltitudeCase(DubinsPath::ALT_CASE_LOW);
    dp.setGamma(atan2f(dz, L));

  } else if (fabs_dz * tanGammaMaxInv_ >= (L + twopi)) {  // high altitude

    dp.setAltitudeCase(DubinsPath::ALT_CASE_HIGH);
    const int k = std::floor((fabs_dz * tanGammaMaxInv_ - L) * one_div_twopi);

    if (dz >= 0) {
      dp.setGamma(gammaMax_);
      dp.setStartHelix(k, computeOptRratio(fabs_dz, L, tanf(fabs(dp.getGamma())), k));
    } else {
      dp.setGamma(-gammaMax_);
      dp.setEndHelix(k, computeOptRratio(fabs_dz, L, tanf(fabs(dp.getGamma())), k));
    }

  } else {  // medium altitude

    dp.setAltitudeCase(DubinsPath::ALT_CASE_MEDIUM);

    // fly at most one circle too much
    const int k = std::floor((fabs_dz * tanGammaMaxInv_ - L) * one_div_twopi) + 1;

    if (dz >= 0) {
      dp.setStartHelix(k, 1.0);
      dp.setGamma(
          atan2f(dz, L + dp.getSegmentLength(0)));  // need to use dp.length_2D since length changed in line before!
    } else {
      dp.setEndHelix(k, 1.0);
      dp.setGamma(
          atan2f(dz, L + dp.getSegmentLength(5)));  // need to use dp.length_2D since length changed in line before!
    }
}

  if (dp.getIdx() < 4) {  // CSC cases
    csc_ctr_ += 1;
  } else {
    ccc_ctr_ += 1;
  }
}

DubinsPath::Classification DubinsAirplaneStateSpace::classifyPath(double alpha, double beta) const {
  int row(0), column(0);
  // TODO: Check if here something with integer division can be done + switch
  if (0 <= alpha && alpha <= half_pi) {
    row = 1;
  } else if (half_pi < alpha && alpha <= pi) {
    row = 2;
  } else if (pi < alpha && alpha <= 3 * half_pi) {
    row = 3;
  } else if (3 * half_pi < alpha && alpha <= twopi) {
    row = 4;
  }

  if (0 <= beta && beta <= half_pi) {
    column = 1;
  } else if (half_pi < beta && beta <= pi) {
    column = 2;
  } else if (pi < beta && beta <= 3 * half_pi) {
    column = 3;
  } else if (3 * half_pi < beta && beta <= twopi) {
    column = 4;
  }

  assert(row >= 1 && row <= 4 && "alpha is not in the range of [0,2pi] in classifyPath(double alpha, double beta).");
  assert(column >= 1 && column <= 4 &&
         "beta is not in the range of [0,2pi] in classifyPath(double alpha, double beta).");
  assert((column - 1) + 4 * (row - 1) >= 0 && (column - 1) + 4 * (row - 1) <= 15 && "class is not in range [0,15].");
  return (DubinsPath::Classification)((column - 1) + 4 * (row - 1));
}

double DubinsAirplaneStateSpace::computeOptRratio(double fabsHdist, double L, double fabsTanGamma, int k) const {
  return (fabsHdist - L * fabsTanGamma) / (twopi * fabsTanGamma * k);
}

void DubinsAirplaneStateSpace::interpolate(const DubinsPath& path, const SegmentStarts& segmentStarts, double t,
                                           ob::State* state) const {
  interpol_seg_ = t * path.length_2D();
  if (path.getGamma() == gammaMax_)
    interpol_tanGamma_ = tanGammaMax_;
  else if (path.getGamma() == -gammaMax_)
    interpol_tanGamma_ = -tanGammaMax_;
  else
    interpol_tanGamma_ = tanf(path.getGamma());

  for (interpol_iter_ = 0u; interpol_iter_ < 6u; ++interpol_iter_) {
    if ((interpol_seg_ < path.getSegmentLength(interpol_iter_) || (interpol_iter_ == 5u))) {
      stateInterpolation_->setXYZYaw(0.0, 0.0, 0.0, segmentStarts.segmentStarts[interpol_iter_].yaw);
      interpol_phiStart_ = stateInterpolation_->getYaw();

      switch (path.getType()[convert_idx(interpol_iter_)]) {
        case DubinsPath::DUBINS_LEFT:
          interpol_dPhi_ = interpol_seg_ * path.getInverseRadiusRatio(interpol_iter_);
          if (interpol_iter_ != 2u) {
            interpol_tmp_ = 2.0 * path.getRadiusRatio(interpol_iter_) * sinf(0.5 * interpol_dPhi_);
            stateInterpolation_->addToX(interpol_tmp_ * cosf(interpol_phiStart_ + 0.5 * interpol_dPhi_));
            stateInterpolation_->addToY(interpol_tmp_ * sinf(interpol_phiStart_ + 0.5 * interpol_dPhi_));
            stateInterpolation_->addToZ(interpol_seg_ * interpol_tanGamma_);
            stateInterpolation_->setYaw(interpol_phiStart_ + interpol_dPhi_);
            break;
          } else {  // DUBINS_RIGHT case for intermediate spiral
            interpol_tmp_ = 2.0 * path.getRadiusRatio(interpol_iter_) * sinf(0.5 * interpol_dPhi_);
            stateInterpolation_->addToX(interpol_tmp_ * cosf(interpol_phiStart_ - 0.5 * interpol_dPhi_));
            stateInterpolation_->addToY(interpol_tmp_ * sinf(interpol_phiStart_ - 0.5 * interpol_dPhi_));
            stateInterpolation_->addToZ(interpol_seg_ * interpol_tanGamma_);
            stateInterpolation_->setYaw(interpol_phiStart_ - interpol_dPhi_);
            break;
          }
        case DubinsPath::DUBINS_RIGHT:
          interpol_dPhi_ = interpol_seg_ * path.getInverseRadiusRatio(interpol_iter_);
          if (interpol_iter_ != 2u) {
            interpol_tmp_ = 2.0 * path.getRadiusRatio(interpol_iter_) * sinf(0.5 * interpol_dPhi_);
            stateInterpolation_->addToX(interpol_tmp_ * cosf(interpol_phiStart_ - 0.5 * interpol_dPhi_));
            stateInterpolation_->addToY(interpol_tmp_ * sinf(interpol_phiStart_ - 0.5 * interpol_dPhi_));
            stateInterpolation_->addToZ(interpol_seg_ * interpol_tanGamma_);
            stateInterpolation_->setYaw(interpol_phiStart_ - interpol_dPhi_);
            break;
          } else {  // DUBINS_LEFT case for intermediate spiral
            interpol_tmp_ = 2.0 * path.getRadiusRatio(interpol_iter_) * sinf(0.5 * interpol_dPhi_);
            stateInterpolation_->addToX(interpol_tmp_ * cosf(interpol_phiStart_ + 0.5 * interpol_dPhi_));
            stateInterpolation_->addToY(interpol_tmp_ * sinf(interpol_phiStart_ + 0.5 * interpol_dPhi_));
            stateInterpolation_->addToZ(interpol_seg_ * interpol_tanGamma_);
            stateInterpolation_->setYaw(interpol_phiStart_ + interpol_dPhi_);
            break;
          }
        case DubinsPath::DUBINS_STRAIGHT:
          if (interpol_iter_ != 2u) {
            stateInterpolation_->addToX(interpol_seg_ * cosf(interpol_phiStart_));
            stateInterpolation_->addToY(interpol_seg_ * sinf(interpol_phiStart_));
            stateInterpolation_->addToZ(interpol_seg_ * interpol_tanGamma_);
            break;
          } else {
            std::cout
                << "This should never happen, otherwise something wrong in the DubinsAirplaneStateSpace::interpolate"
                   "(const ob::State *from, const DubinsPath &path, double t, ob::State *state) const function.";
            break;
          }
      }
      state->as<StateType>()->setX(stateInterpolation_->getX() * rho_ + segmentStarts.segmentStarts[interpol_iter_].x);
      state->as<StateType>()->setY(stateInterpolation_->getY() * rho_ + segmentStarts.segmentStarts[interpol_iter_].y);
      state->as<StateType>()->setZ(stateInterpolation_->getZ() * rho_ + segmentStarts.segmentStarts[interpol_iter_].z);
      getSubspace(1)->enforceBounds(stateInterpolation_->as<ob::SO2StateSpace::StateType>(1));
      state->as<StateType>()->setYaw(stateInterpolation_->getYaw());
      interpol_seg_ = 0.0;
      return;

    } else {
      interpol_seg_ -= path.getSegmentLength(interpol_iter_);
    }
  }
  std::cout << "This should never happen, otherwise something wrong in the DubinsAirplaneStateSpace::interpolate"
               "(const ob::State *from, const DubinsPath &path, double t, ob::State *state) const function.";
  return;
}

void DubinsAirplaneStateSpace::interpolateWithWind(const ob::State* from, const DubinsPath& path,
                                                   const SegmentStarts& segmentStarts, double t,
                                                   ob::State* state) const {
  interpolate(path, segmentStarts, t, state);
}

void DubinsAirplaneStateSpace::calculateSegmentStarts(const ob::State* from, const DubinsPath& path,
                                                      SegmentStarts& segmentStarts) const {
  if (std::isnan(path.length_2D())) return;

  interpol_seg_ = path.length_2D();
  if (path.getGamma() == gammaMax_)
    interpol_tanGamma_ = tanGammaMax_;
  else if (path.getGamma() == -gammaMax_)
    interpol_tanGamma_ = -tanGammaMax_;
  else
    interpol_tanGamma_ = tanf(path.getGamma());

  stateInterpolation_->setXYZYaw(0.0, 0.0, 0.0, from->as<StateType>()->getYaw());
  for (interpol_iter_ = 0u; interpol_iter_ < 6u && interpol_seg_ > 0.0; ++interpol_iter_) {
    interpol_v_ = std::min(interpol_seg_, path.getSegmentLength(interpol_iter_));
    interpol_phiStart_ = stateInterpolation_->getYaw();
    interpol_seg_ -= interpol_v_;

    segmentStarts.segmentStarts[interpol_iter_].x = stateInterpolation_->getX() * rho_ + from->as<StateType>()->getX();
    segmentStarts.segmentStarts[interpol_iter_].y = stateInterpolation_->getY() * rho_ + from->as<StateType>()->getY();
    segmentStarts.segmentStarts[interpol_iter_].z = stateInterpolation_->getZ() * rho_ + from->as<StateType>()->getZ();
    getSubspace(1)->enforceBounds(stateInterpolation_->as<ob::SO2StateSpace::StateType>(1));
    segmentStarts.segmentStarts[interpol_iter_].yaw = stateInterpolation_->getYaw();

    switch (path.getType()[convert_idx(interpol_iter_)]) {
      case DubinsPath::DUBINS_LEFT:
        interpol_dPhi_ = interpol_v_ * path.getInverseRadiusRatio(interpol_iter_);
        if (interpol_iter_ != 2u) {
          interpol_tmp_ = 2.0 * path.getRadiusRatio(interpol_iter_) * sinf(0.5 * interpol_dPhi_);
          stateInterpolation_->addToX(interpol_tmp_ * cosf(interpol_phiStart_ + 0.5 * interpol_dPhi_));
          stateInterpolation_->addToY(interpol_tmp_ * sinf(interpol_phiStart_ + 0.5 * interpol_dPhi_));
          stateInterpolation_->addToZ(interpol_v_ * interpol_tanGamma_);
          stateInterpolation_->setYaw(interpol_phiStart_ + interpol_dPhi_);
          break;
        } else {  // DUBINS_RIGHT case for intermediate spiral
          interpol_tmp_ = 2.0 * path.getRadiusRatio(interpol_iter_) * sinf(0.5 * interpol_dPhi_);
          stateInterpolation_->addToX(interpol_tmp_ * cosf(interpol_phiStart_ - 0.5 * interpol_dPhi_));
          stateInterpolation_->addToY(interpol_tmp_ * sinf(interpol_phiStart_ - 0.5 * interpol_dPhi_));
          stateInterpolation_->addToZ(interpol_v_ * interpol_tanGamma_);
          stateInterpolation_->setYaw(interpol_phiStart_ - interpol_dPhi_);
          break;
        }
      case DubinsPath::DUBINS_RIGHT:
        interpol_dPhi_ = interpol_v_ * path.getInverseRadiusRatio(interpol_iter_);
        if (interpol_iter_ != 2u) {
          interpol_tmp_ = 2.0 * path.getRadiusRatio(interpol_iter_) * sinf(0.5 * interpol_dPhi_);
          stateInterpolation_->addToX(interpol_tmp_ * cosf(interpol_phiStart_ - 0.5 * interpol_dPhi_));
          stateInterpolation_->addToY(interpol_tmp_ * sinf(interpol_phiStart_ - 0.5 * interpol_dPhi_));
          stateInterpolation_->addToZ(interpol_v_ * interpol_tanGamma_);
          stateInterpolation_->setYaw(interpol_phiStart_ - interpol_dPhi_);
          break;
        } else {  // DUBINS_LEFT case for intermediate spiral
          interpol_tmp_ = 2.0 * path.getRadiusRatio(interpol_iter_) * sinf(0.5 * interpol_dPhi_);
          stateInterpolation_->addToX(interpol_tmp_ * cosf(interpol_phiStart_ + 0.5 * interpol_dPhi_));
          stateInterpolation_->addToY(interpol_tmp_ * sinf(interpol_phiStart_ + 0.5 * interpol_dPhi_));
          stateInterpolation_->addToZ(interpol_v_ * interpol_tanGamma_);
          stateInterpolation_->setYaw(interpol_phiStart_ + interpol_dPhi_);
          break;
        }
      case DubinsPath::DUBINS_STRAIGHT:
        if (interpol_iter_ != 2u) {
          stateInterpolation_->addToX(interpol_v_ * cosf(interpol_phiStart_));
          stateInterpolation_->addToY(interpol_v_ * sinf(interpol_phiStart_));
          stateInterpolation_->addToZ(interpol_v_ * interpol_tanGamma_);
          break;
        } else {
          std::cout
              << "This should never happen, otherwise something wrong in the DubinsAirplaneStateSpace::interpolate"
                 "(const ob::State *from, const DubinsPath &path, double t, ob::State *state) const function.";
          break;
        }
    }
  }
}

void DubinsAirplaneStateSpace::getStateOnCircle(const ob::State* from, int rl /* right (0), left (1)*/,
                                                int ud /* up(0), down(1) */, double t, ob::State* state) const {
  // assuming flying with rho_ and gammaMax_
  StateType* s = allocState()->as<StateType>();

  s->setXYZ(0.0, 0.0, 0.0);
  s->setYaw(from->as<StateType>()->getYaw());
  const double phi = s->getYaw();

  switch (rl) {
    case 1:  // left
      // TODO: precompute tanf(gammaMax_)
      s->setXYZ(s->getX() + sinf(phi + t) - sinf(phi), s->getY() - cosf(phi + t) + cosf(phi),
                s->getZ() + t * tanf(ud * gammaMax_));
      s->setYaw(phi + t);
      break;

    case 0:  // right
      // TODO: precompute tanf(gammaMax_)
      s->setXYZ(s->getX() - sinf(phi - t) + sinf(phi), s->getY() + cosf(phi - t) - cosf(phi),
                s->getZ() + t * tanf(ud * gammaMax_));
      s->setYaw(phi - t);
      break;
  }

  state->as<StateType>()->setX(s->getX() * rho_ + from->as<StateType>()->getX());
  state->as<StateType>()->setY(s->getY() * rho_ + from->as<StateType>()->getY());
  state->as<StateType>()->setZ(s->getZ() * rho_ + from->as<StateType>()->getZ());
  getSubspace(1)->enforceBounds(s->as<ob::SO2StateSpace::StateType>(1));
  state->as<StateType>()->setYaw(s->getYaw());
  freeState(s);
}

unsigned int DubinsAirplaneStateSpace::convert_idx(unsigned int i) const {
  assert(i >= 0u && "In convert_idx, i < 0");
  assert(i < 6u && "In convert_idx, i > 5");
  switch (i) {
    case 0:  // start helix
    case 1:  // first dubins segment
      /* intermediate maneuver, return same direction as first dubins segment before
       * In interpolate function, will handle this and turn the other way. */
    case 2: {
      return 0u;
      break;
    }
    case 3:  // second dubins segment
    {
      return 1u;
      break;
    }
    default:  // third dubins segment and end helix
    {
      return 2u;
      break;
    }
  }
}

}  // namespace spaces

}  // namespace fw_planning

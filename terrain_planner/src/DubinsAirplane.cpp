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
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

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

  addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.);
  addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.);
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
  const double d = sqrtf(dx * dx + dy * dy);
  const double th = atan2f(dy, dx);
  const double alpha = mod2pi(th1 - th);
  const double beta = mod2pi(th2 - th);

  // compute the 2D path
  dubins(d, alpha, beta, dp);
  double L = dp.length_2D();

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

    if (optimalStSp_) {
      auto tuple = additionalManeuver(dp, L, state1, state2);

      if (dp.getIdx() < 4) {  // CSC cases
        // double phi_i = std::get<2>(tuple);
        dp.setFoundOptimalPath(std::get<1>(tuple));
        dp.setAdditionalManeuver(true);
        dp.setSegmentLength(std::get<0>(tuple), 1);
        dp.setSegmentLength(std::get<3>(tuple), 3);
        dp.setSegmentLength(std::get<4>(tuple), 4);
        if (dz >= 0) {
          dp.setGamma(gammaMax_);
          dp.setSegmentLength(0.0, 0);
          dp.setSegmentLength(std::get<2>(tuple), 2);
        } else {
          dp.setGamma(-gammaMax_);
          dp.setSegmentLength(0.0, 5);
          dp.setSegmentLength(std::get<2>(tuple), 2);
        }
      } else {  // CCC cases (does not find any optimal path yet. Flies 2D Dubins car path with adequate climbing rate
                // gamma)

        // double rad = std::get<0>(tuple);
        dp.setFoundOptimalPath(std::get<1>(tuple));
        dp.setSegmentLength(std::get<2>(tuple), 1);
        dp.setSegmentLength(std::get<3>(tuple), 3);
        dp.setSegmentLength(std::get<4>(tuple), 4);

        if (dz >= 0) {
          dp.setStartHelix(1, 1.0);
          dp.setGamma(atan2f(dz, L + twopi));
        } else {
          dp.setEndHelix(1, 1.0);
          dp.setGamma(atan2f(dz, L + twopi));
        }
      }

    } else {
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
  }

  if (dp.getIdx() < 4) {  // CSC cases
    csc_ctr_ += 1;
  } else {
    ccc_ctr_ += 1;
  }
}

void DubinsAirplaneStateSpace::dubins(double d, double alpha, double beta, DubinsPath& path) const {
  if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS) {
    path = DubinsPath(DubinsPath::TYPE_LSL, 0.0, d, 0.0);

  } else {
    const double sa = sinf(alpha);
    const double sb = sinf(beta);
    const double ca = cosf(alpha);
    const double cb = cosf(beta);

    // TODO: Check if that is necessary for the short path case or if it can be moved inside the bracket of the long
    // distance cases.
    path.setClassification(classifyPath(alpha, beta));

    if (enable_classification_) {
      bool long_path_case = d > (sqrtf(4.0 - pow(ca + cb, 2.0)) + fabs(sa) + fabs(sb));
      if (long_path_case) {  // sufficient condition for optimality of CSC path type
        ++long_ctr_;
        calcDubPathWithClassification(path, d, alpha, beta, sa, sb, ca, cb);
        return;
      }
    }
    ++short_ctr_;
    calcDubPathWithoutClassification(path, d, alpha, beta, sa, sb, ca, cb);
  }
}

void DubinsAirplaneStateSpace::calcDubPathWithClassification(DubinsPath& path, double d, double alpha, double beta,
                                                             double sa, double sb, double ca, double cb) const {
  // TODO: Computational speed up could be achieved here by ordering the if-cases according to their relative
  // probability (if known a priori)
  switch (path.getClassification()) {
    case DubinsPath::CLASS_A11: {  // class a_11: optimal path is RSL
      path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      break;
    }
    case DubinsPath::CLASS_A12: {  // class a_12: depending on S_12, optimal path is either RSR (S_12<0) or RSL (S_12>0)
      if (t_rsr(d, alpha, beta, sa, sb, ca, cb) - pi < 0.0) {
        if (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb) -
                2.0 * (q_rsl(d, alpha, beta, sa, sb, ca, cb) - pi) >
            0.0) {  // S_12>0: RSL is optimal
          path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
        } else {  // S_12<0: RSR is optimal
          path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
        }
      } else {
        if (p_lsr(d, alpha, beta, sa, sb, ca, cb) + t_lsr(d, alpha, beta, sa, sb, ca, cb) +
                q_lsr(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb) -
                q_rsl(d, alpha, beta, sa, sb, ca, cb) - t_rsl(d, alpha, beta, sa, sb, ca, cb) <
            0.0) {  // S_12>0: RSL is optimal
          // LSR is optimal
          path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
        } else {  // RSL is optimal
          path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
        }
      }
      break;
    }
    case DubinsPath::CLASS_A13: {  // class a_13: depending on S_13, optimal path is either RSR (S_13<0) or LSR (S_13>0)
      if (t_rsr(d, alpha, beta, sa, sb, ca, cb) - pi > 0.0) {  // S_13>0: LSR is optimal
        path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
      } else {  // S_13<0: RSR is optimal
        path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A14: {  // class a_14: depending on S^{1,2}_14,
      // optimal path is LSR (S^1_14>0) or RSL (S^2_14>0) or RSR otherwise
      if (t_rsr(d, alpha, beta, sa, sb, ca, cb) - pi > 0.0) {  // S^1_14>0: LSR is optimal
        path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
      } else if (q_rsr(d, alpha, beta, sa, sb, ca, cb) - pi > 0.0) {  // S^2_14>0: RSL is optimal
        path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      } else {  // RSR is optimal
        path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A21: {  // class a_21 (top. equiv. a_12): depending on S_21, optimal path is either LSL
                                   // (S_21<0) or RSL (S_12>0)
      if (q_lsl(d, alpha, beta, sa, sb, ca, cb) - pi < 0.0) {
        if (p_lsl(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb) -
                2.0 * (t_rsl(d, alpha, beta, sa, sb, ca, cb) - pi) <
            0.0) {  // S_21<0: LSL is optimal
          path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
        } else {  // S_21>0: RSL is optimal
          path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
        }
      } else {
        if (p_lsr(d, alpha, beta, sa, sb, ca, cb) + t_lsr(d, alpha, beta, sa, sb, ca, cb) +
                q_lsr(d, alpha, beta, sa, sb, ca, cb) - p_lsl(d, alpha, beta, sa, sb, ca, cb) -
                q_lsl(d, alpha, beta, sa, sb, ca, cb) - t_lsl(d, alpha, beta, sa, sb, ca, cb) <
            0.0) {  // S_21<0: LSL is optimal
          path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
        } else {  // S_21>0: RSL is optimal
          path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
        }
      }
      break;
    }
    case DubinsPath::CLASS_A22: {  // class a_22 (top. equiv. a_33): depending on alpha, beta, S^{1,2}_22,
      // optimal path is  LSL (alpha>beta && S^1_22<0) or RSL (alpha>beta && S^1_22>0)
      //                  RSR (alpha<beta && S^2_22<0) or RSL (alpha<beta && S^2_22>0)
      if (alpha >= beta && (p_lsl(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb) -
                            2.0 * (t_rsl(d, alpha, beta, sa, sb, ca, cb) - pi)) < 0.0) {  // LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      } else if (alpha < beta && (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb) -
                                  2.0 * (q_rsl(d, alpha, beta, sa, sb, ca, cb) - pi)) < 0.0) {  // RSR is optimal
        path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
      } else {  // if( alpha < beta && (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb) -
                // 2*(q_rsl(d, alpha, beta, sa, sb, ca, cb) - pi)) > 0 ) { // RSL is optimal
        path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A23: {  // class a_23: RSR is optimal
      path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
      break;
    }
    case DubinsPath::CLASS_A24: {  // class a_24: depending on S_24, optimal path is RSR (S_24<0) or RSL (S_24>0)
      if (q_rsr(d, alpha, beta, sa, sb, ca, cb) - pi < 0.0) {  // S_24<0: RSR is optimal
        path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
      } else {  // S_24>0: RSL is optimal
        path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A31: {  // class a_31 (top. equiv. to a_13): depending on S_31, optimal path is LSL (S_31<0)
                                   // or LSR (S_31>0)
      if (q_lsl(d, alpha, beta, sa, sb, ca, cb) - pi < 0.0) {  // S_31<0: LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      } else {  // S_31>0: LSR is optimal
        path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A32: {  // class a_32 (top. equiv. to a_32): optimal path is LSL
      path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      break;
    }
    case DubinsPath::CLASS_A33: {  // class a_33 (top. equiv. to a_33): depending on a, b, S^{1,2}_33,
      // optimal path is  RSR (alpha<beta && S^1_33<0) or LSR (alpha>beta && S^1_33>0)
      //                  LSL (alpha<beta && S^2_33<0) or LSR (alpha>beta && S^2_33>0)
      if (alpha <= beta &&
          (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_lsr(d, alpha, beta, sa, sb, ca, cb) -
           2.0 * (t_lsr(d, alpha, beta, sa, sb, ca, cb) - pi)) < 0.0) {  // alpha<beta && S^1_33<0: RSR is optimal
        path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
      } else if (alpha > beta && (p_lsl(d, alpha, beta, sa, sb, ca, cb) - p_lsr(d, alpha, beta, sa, sb, ca, cb) -
                                  2.0 * (q_lsr(d, alpha, beta, sa, sb, ca, cb) - pi)) <
                                     0.0) {  // alpha<beta && S^2_33<0: LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      } else {  // LSR is optimal
        path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A34: {  // class a_34 (top. equiv. to a_12): depending on S_34, optimal path is RSR
                                   // (S_34<0) or LSR (S_34>0)
      if (q_rsr(d, alpha, beta, sa, sb, ca, cb) - pi < 0.0) {
        if (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_lsr(d, alpha, beta, sa, sb, ca, cb) -
                2.0 * (t_lsr(d, alpha, beta, sa, sb, ca, cb) - pi) <
            0) {  // S_34<0: RSR is optimal
          path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
        } else {  // S_34>0: LSR is optimal
          path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
        }
      } else {
        if (p_lsr(d, alpha, beta, sa, sb, ca, cb) + t_lsr(d, alpha, beta, sa, sb, ca, cb) +
                q_lsr(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb) -
                q_rsl(d, alpha, beta, sa, sb, ca, cb) - t_rsl(d, alpha, beta, sa, sb, ca, cb) <
            0.0) {  // S_34<0: RSR is optimal
          path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
        } else {  // S_34>0: LSR is optimal
          path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
        }
      }
      break;
    }
    case DubinsPath::CLASS_A41: {  // class a_41 (top. equiv. to a_14): depending on S^{1,2}_41,
      // optimal path is RSL (S^1_41>0) or LSR (S^2_41>0) or LSL otherwise
      if (t_lsl(d, alpha, beta, sa, sb, ca, cb) - pi > 0.0) {  // S^1_41>0: RSL is optimal
        path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      } else if (q_lsl(d, alpha, beta, sa, sb, ca, cb) - pi > 0.0) {  // S^2_41>0: LSR is optimal
        path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
      } else {  // LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A42: {  // class a_42 (top. equiv. to a_13): depending on S_42, optimal path is LSL (S_42<0)
                                   // or RSL (S_42>0)
      if (t_lsl(d, alpha, beta, sa, sb, ca, cb) - pi < 0.0) {  // S_42 < 0: LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      } else {  // S_42 > 0: RSL is optimal
        path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A43: {  // class a_43 (top. equiv. to a_34): depending on S_43, optimal path is LSL
                                   // (S_43<0) or LSR (S_43>0)
      if (t_lsl(d, alpha, beta, sa, sb, ca, cb) - pi < 0.0) {
        if (p_lsl(d, alpha, beta, sa, sb, ca, cb) - p_lsr(d, alpha, beta, sa, sb, ca, cb) -
                2.0 * (q_lsr(d, alpha, beta, sa, sb, ca, cb) - pi) <
            0.0) {  // S_43<0: LSL is optimal
          path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
        } else {  // S_43>0: LSR is optimal
          path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
        }
      } else {
        if (p_lsr(d, alpha, beta, sa, sb, ca, cb) + t_lsr(d, alpha, beta, sa, sb, ca, cb) +
                q_lsr(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb) -
                q_rsl(d, alpha, beta, sa, sb, ca, cb) - t_rsl(d, alpha, beta, sa, sb, ca, cb) <
            0.0) {  // S_12>0: RSL is optimal
          // LSR is optimal
          path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
        } else {  // RSL is optimal
          path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
        }
      }
      break;
    }
    case DubinsPath::CLASS_A44: {  // class a_44: optimal path is LSR
      path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
      break;
    }
    default: {
      std::cout << "default (a not in set{0,1,...,15}), path.a: " << path.getClassification() << " ,d: " << d
                << " ,alpha: " << alpha << ",beta: " << beta;
      assert(false && "class of path (path.a) was not assigned to an integer in the set {0,1,...,15}.");
    }
  }
}

void DubinsAirplaneStateSpace::calcDubPathWithoutClassification(DubinsPath& path, double d, double alpha, double beta,
                                                                double sa, double sb, double ca, double cb) const {
  path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
  DubinsPath tmp2(dubinsRSR(d, alpha, beta, sa, sb, ca, cb));
  double len;
  double minLength = path.length_2D();

  if ((len = tmp2.length_2D()) < minLength) {
    minLength = len;
    path = tmp2;
  }
  tmp2 = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
  if ((len = tmp2.length_2D()) < minLength) {
    minLength = len;
    path = tmp2;
  }
  tmp2 = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
  if ((len = tmp2.length_2D()) < minLength) {
    minLength = len;
    path = tmp2;
  }
  tmp2 = dubinsRLR(d, alpha, beta, sa, sb, ca, cb);
  if ((len = tmp2.length_2D()) < minLength) {
    minLength = len;
    path = tmp2;
  }
  tmp2 = dubinsLRL(d, alpha, beta, sa, sb, ca, cb);
  if ((len = tmp2.length_2D()) < minLength) {
    path = tmp2;
  }
}

std::tuple<double, bool, double, double, double> DubinsAirplaneStateSpace::additionalManeuver(
    const DubinsPath& dp, double& L_2D, const ob::State* state1, const ob::State* state2) const {
  bool foundSol = false;

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

  const double dx = (x2 - x1) * curvature_;   // dx scaled by the radius
  const double dy = (y2 - y1) * curvature_;   // dy scaled by the radius
  const double dz = (z2 - z1) * curvature_;   // dz scaled by the radius
  const double d = sqrtf(dx * dx + dy * dy);  // 2D euclidean distance
  const double th = atan2f(dy, dx);
  const double alpha = mod2pi(th1 - th);
  const double beta = mod2pi(th2 - th);

  const double step =
      0.26;  // 0.35 = 20 / 180 * pi, 0.26 = 15. / 180. * pi, 0.17 = 10. / 180. * pi, 0.09 = 5. / 180. * pi

  double L_desired2D = fabs(dz) * tanGammaMaxInv_;

  double error_abs = fabs(L_desired2D - L_2D);
  double error_min_abs = error_abs;

  // allocate variables outside the loop.
  double phi_min = dp.getSegmentLength(1);
  double t_min = 0.0;
  double p_min = dp.getSegmentLength(3);
  double q_min = dp.getSegmentLength(4);
  double x1_c, y1_c, /*z1_c,*/ dx_c, dy_c, /*dz_c,*/ d_c, th1_c, th_c, alpha_c, beta_c = 0.0;

  ob::State* si = allocState();
  DubinsPath dp_tmp;

  // seperate by path case
  switch (dp.getIdx()) {
    case DubinsPath::TYPE_LSL: {
      // The sub-optimal 2D dubins path is LSL so the optimal path is L + RSL
      for (double phi = 0.; phi < twopi; phi += step) {
        // get a state on the circle with the angle phi
        getStateOnCircle(state1, 1 /* right (0), left (1)*/, sgn(dz), phi, si);

        // extract state
        x1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getX();
        y1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getY();
        // z1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getZ();
        th1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getYaw();
        dx_c = (x2 - x1_c) * curvature_;
        dy_c = (y2 - y1_c) * curvature_;
        // dz_c = (z2 - z1_c) * curvature_;
        d_c = sqrtf(dx_c * dx_c + dy_c * dy_c);
        th_c = atan2f(dy_c, dx_c);
        alpha_c = mod2pi(th1_c - th_c);
        beta_c = mod2pi(th2 - th_c);

        dp_tmp = dubinsRSL(d_c, alpha_c, beta_c);
        L_2D = dp_tmp.length_2D() + phi;

        error_abs = fabs(L_desired2D - L_2D);
        if (error_abs < error_min_abs) {
          error_min_abs = error_abs;
          phi_min = phi;
          foundSol = true;
          t_min = dp_tmp.getSegmentLength(1);
          p_min = dp_tmp.getSegmentLength(3);
          q_min = dp_tmp.getSegmentLength(4);
        }
      }
      freeState(si);
      return std::make_tuple(phi_min, foundSol, t_min, p_min, q_min);
    }
    case DubinsPath::TYPE_RSR: {
      // The 2D dubins path is RSR so the optimal 3D path is R + LSR
      for (double phi = 0.; phi < twopi; phi += step) {
        // get a state on the circle with the angle phi
        getStateOnCircle(state1, 0 /* right (0), left (1)*/, sgn(dz), phi, si);

        // extract state
        x1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getX();
        y1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getY();
        // z1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getZ();
        th1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getYaw();
        dx_c = (x2 - x1_c) * curvature_;
        dy_c = (y2 - y1_c) * curvature_;
        // dz_c = (z2 - z1_c) * curvature_;
        d_c = sqrtf(dx_c * dx_c + dy_c * dy_c);
        th_c = atan2f(dy_c, dx_c);
        alpha_c = mod2pi(th1_c - th_c);
        beta_c = mod2pi(th2 - th_c);

        dp_tmp = dubinsLSR(d_c, alpha_c, beta_c);
        L_2D = dp_tmp.length_2D() + phi;

        error_abs = fabs(L_desired2D - L_2D);
        if (error_abs < error_min_abs) {
          error_min_abs = error_abs;
          phi_min = phi;
          foundSol = true;
          t_min = dp_tmp.getSegmentLength(1);
          p_min = dp_tmp.getSegmentLength(3);
          q_min = dp_tmp.getSegmentLength(4);
        }
      }
      freeState(si);
      return std::make_tuple(phi_min, foundSol, t_min, p_min, q_min);
    }
    case DubinsPath::TYPE_RSL: {
      // The 2D dubins path is RSL so the optimal 3D path is R + LSL
      for (double phi = 0.; phi < twopi; phi += step) {
        // get a state on the circle with the angle phi
        getStateOnCircle(state1, 0 /* right (0), left (1)*/, sgn(dz), phi, si);

        // extract state
        x1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getX();
        y1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getY();
        // z1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getZ();
        th1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getYaw();
        dx_c = (x2 - x1_c) * curvature_;
        dy_c = (y2 - y1_c) * curvature_;
        // dz_c = (z2 - z1_c) * curvature_;
        d_c = sqrtf(dx_c * dx_c + dy_c * dy_c);
        th_c = atan2f(dy_c, dx_c);
        alpha_c = mod2pi(th1_c - th_c);
        beta_c = mod2pi(th2 - th_c);

        dp_tmp = dubinsLSL(d_c, alpha_c, beta_c);
        L_2D = dp_tmp.length_2D() + phi;

        error_abs = fabs(L_desired2D - L_2D);

        if (error_abs < error_min_abs) {
          error_min_abs = error_abs;
          phi_min = phi;
          foundSol = true;
          t_min = dp_tmp.getSegmentLength(1);
          p_min = dp_tmp.getSegmentLength(3);
          q_min = dp_tmp.getSegmentLength(4);
        }
      }
      freeState(si);
      return std::make_tuple(phi_min, foundSol, t_min, p_min, q_min);
    }
    case DubinsPath::TYPE_LSR: {
      // The 2D dubins path is LSR so the optimal 3D path is L + RSR
      for (double phi = 0.; phi < twopi; phi += step) {
        // get a state on the circle with the angle phi
        getStateOnCircle(state1, 1 /* right (0), left (1)*/, sgn(dz), phi, si);

        // extract state
        x1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getX();
        y1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getY();
        // z1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getZ();
        th1_c = si->as<DubinsAirplaneStateSpace::StateType>()->getYaw();
        dx_c = (x2 - x1_c) * curvature_;
        dy_c = (y2 - y1_c) * curvature_;
        // dz_c = (z2 - z1_c) * curvature_;
        d_c = sqrtf(dx_c * dx_c + dy_c * dy_c);
        th_c = atan2f(dy_c, dx_c);
        alpha_c = mod2pi(th1_c - th_c);
        beta_c = mod2pi(th2 - th_c);

        dp_tmp = dubinsRSR(d_c, alpha_c, beta_c);
        L_2D = dp_tmp.length_2D() + phi;

        error_abs = fabs(L_desired2D - L_2D);
        if (error_abs < error_min_abs) {
          error_min_abs = error_abs;
          phi_min = phi;
          foundSol = true;
          t_min = dp_tmp.getSegmentLength(1);
          p_min = dp_tmp.getSegmentLength(3);
          q_min = dp_tmp.getSegmentLength(4);
        }
      }
      freeState(si);
      return std::make_tuple(phi_min, foundSol, t_min, p_min, q_min);
    }
    case DubinsPath::TYPE_RLR: {  // RLR
      // does not find any adequate solution so far. Returns 2D Dubins car path and flies with adequate climbing rate
      dp_tmp =
          dubinsRLR(d, alpha, beta);  // TODO: Check if that is necessary, isn't that the same path as the input path?
      t_min = dp_tmp.getSegmentLength(1);
      p_min = dp_tmp.getSegmentLength(3);
      q_min = dp_tmp.getSegmentLength(4);
      foundSol = false;
      return std::make_tuple(rho_, foundSol, t_min, p_min, q_min);
    }
    case DubinsPath::TYPE_LRL: {  // LRL
      // does not find any adqeuate solution so far. Returns 2D Dubins car path and flies with adequate climbing rate
      dp_tmp =
          dubinsLRL(d, alpha, beta);  // TODO: Check if that is necessary, isn't that the same path as the input path?
      t_min = dp_tmp.getSegmentLength(1);
      p_min = dp_tmp.getSegmentLength(3);
      q_min = dp_tmp.getSegmentLength(4);
      foundSol = false;
      return std::make_tuple(rho_, foundSol, t_min, p_min, q_min);
    }
    default: {
      std::cout << "DubinsAirplane::additionalManeuver: Invalid path index: " << dp.getIdx();
      return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0);
    }
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

  for (interpol_iter_ = 0; interpol_iter_ < 6; ++interpol_iter_) {
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

void DubinsAirplaneStateSpace::interpolateWithWind(const ob::State* /*from*/, const DubinsPath& path,
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
  for (interpol_iter_ = 0; interpol_iter_ < 6 && interpol_seg_ > 0.0; ++interpol_iter_) {
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

double DubinsAirplaneStateSpace::t_lsr(double d, double alpha, double /*beta*/, double sa, double sb, double ca,
                                       double cb) const {
  const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
  const double p = sqrtf(std::max(tmp, 0.0));
  const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
  return mod2pi(-alpha + theta);  // t
}

double DubinsAirplaneStateSpace::p_lsr(double d, double /*alpha*/, double /*beta*/, double sa, double sb, double ca,
                                       double cb) const {
  const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
  return sqrtf(std::max(tmp, 0.0));  // p
}

double DubinsAirplaneStateSpace::q_lsr(double d, double /*alpha*/, double beta, double sa, double sb, double ca,
                                       double cb) const {
  const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
  const double p = sqrtf(std::max(tmp, 0.0));
  const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
  return mod2pi(-beta + theta);  // q
}

double DubinsAirplaneStateSpace::t_rsl(double d, double alpha, double /*beta*/, double sa, double sb, double ca,
                                       double cb) const {
  const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
  const double p = sqrtf(std::max(tmp, 0.0));
  const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
  return mod2pi(alpha - theta);  // t
}

double DubinsAirplaneStateSpace::p_rsl(double d, double /*alpha*/, double /*beta*/, double sa, double sb, double ca,
                                       double cb) const {
  const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
  return sqrtf(std::max(tmp, 0.0));  // p
}

double DubinsAirplaneStateSpace::q_rsl(double d, double /*alpha*/, double beta, double sa, double sb, double ca,
                                       double cb) const {
  const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
  const double p = sqrtf(std::max(tmp, 0.));
  const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
  return mod2pi(beta - theta);  // q
}

double DubinsAirplaneStateSpace::t_rsr(double d, double alpha, double /*beta*/, double sa, double sb, double ca,
                                       double cb) const {
  const double theta = atan2f(ca - cb, d - sa + sb);
  return mod2pi(alpha - theta);  // t
}

double DubinsAirplaneStateSpace::p_rsr(double d, double /*alpha*/, double /*beta*/, double sa, double sb, double ca,
                                       double cb) const {
  const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
  return sqrtf(std::max(tmp, 0.0));  // p
}

double DubinsAirplaneStateSpace::q_rsr(double d, double /*alpha*/, double beta, double sa, double sb, double ca,
                                       double cb) const {
  const double theta = atan2f(ca - cb, d - sa + sb);
  return mod2pi(-beta + theta);  // q
}

double DubinsAirplaneStateSpace::t_lsl(double d, double alpha, double /*beta*/, double sa, double sb, double ca,
                                       double cb) const {
  const double theta = atan2f(cb - ca, d + sa - sb);
  return mod2pi(-alpha + theta);  // t
}

double DubinsAirplaneStateSpace::p_lsl(double d, double /*alpha*/, double /*beta*/, double sa, double sb, double ca,
                                       double cb) const {
  const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
  return sqrtf(std::max(tmp, 0.0));  // p
}

double DubinsAirplaneStateSpace::q_lsl(double d, double /*alpha*/, double beta, double sa, double sb, double ca,
                                       double cb) const {
  const double theta = atan2f(cb - ca, d + sa - sb);
  return mod2pi(beta - theta);  // q
}

// ------------------------- LSL ------------------------- //
DubinsPath DubinsAirplaneStateSpace::dubinsLSL(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
}

DubinsPath DubinsAirplaneStateSpace::dubinsLSL(double d, double alpha, double beta, double sa, double sb, double ca,
                                               double cb) const {
  double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
  if (tmp >= DUBINS_ZERO) {  // TODO Check if fabs is missing.
    const double theta = atan2f(cb - ca, d + sa - sb);
    const double t = mod2pi(-alpha + theta);
    const double p = sqrtf(std::max(tmp, 0.0));
    const double q = mod2pi(beta - theta);
    assert(fabs(p * cosf(alpha + t) - sa + sb - d) < DUBINS_EPS);
    assert(fabs(p * sinf(alpha + t) + ca - cb) < DUBINS_EPS);
    assert(mod2pi(alpha + t + q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS);
    return DubinsPath(DubinsPath::TYPE_LSL, t, p, q);
  }
  return DubinsPath();
}
// ------------------------- LSL ------------------------- //

// ------------------------- RSR ------------------------- //
DubinsPath DubinsAirplaneStateSpace::dubinsRSR(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
}

DubinsPath DubinsAirplaneStateSpace::dubinsRSR(double d, double alpha, double beta, double sa, double sb, double ca,
                                               double cb) const {
  const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
  if (tmp >= DUBINS_ZERO) {  // TODO Check if fabs is missing.
    const double theta = atan2f(ca - cb, d - sa + sb);
    const double t = mod2pi(alpha - theta);
    const double p = sqrtf(std::max(tmp, 0.));
    const double q = mod2pi(-beta + theta);
    assert(fabs(p * cosf(alpha - t) + sa - sb - d) < DUBINS_EPS);
    assert(fabs(p * sinf(alpha - t) - ca + cb) < DUBINS_EPS);
    assert(mod2pi(alpha - t - q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS);
    return DubinsPath(DubinsPath::TYPE_RSR, t, p, q);
  }
  return DubinsPath();
}
// ------------------------- RSR ------------------------- //

// ------------------------- RSL ------------------------- //
DubinsPath DubinsAirplaneStateSpace::dubinsRSL(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
}

DubinsPath DubinsAirplaneStateSpace::dubinsRSL(double d, double alpha, double beta, double sa, double sb, double ca,
                                               double cb) const {
  const double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
  if (tmp >= DUBINS_ZERO) {  // TODO Check if here fabs is missing.
    const double p = sqrtf(std::max(tmp, 0.));
    const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2., p);
    const double t = mod2pi(alpha - theta);
    const double q = mod2pi(beta - theta);
    assert(fabs(p * cosf(alpha - t) - 2.0 * sinf(alpha - t) + sa + sb - d) < DUBINS_EPS);
    assert(fabs(p * sinf(alpha - t) + 2.0 * cosf(alpha - t) - ca - cb) < DUBINS_EPS);
    assert(mod2pi(alpha - t + q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS);
    return DubinsPath(DubinsPath::TYPE_RSL, t, p, q);
  }
  return DubinsPath();
}
// ------------------------- RSL ------------------------- //

// ------------------------- LSR ------------------------- //
DubinsPath DubinsAirplaneStateSpace::dubinsLSR(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
}

DubinsPath DubinsAirplaneStateSpace::dubinsLSR(double d, double alpha, double beta, double sa, double sb, double ca,
                                               double cb) const {
  const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
  if (tmp >= DUBINS_ZERO) {  // TODO Check if here fabs is missing.
    const double p = sqrtf(std::max(tmp, 0.0));
    const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2., p);
    const double t = mod2pi(-alpha + theta);
    const double q = mod2pi(-beta + theta);
    assert(fabs(p * cosf(alpha + t) + 2.0 * sinf(alpha + t) - sa - sb - d) < DUBINS_EPS);
    assert(fabs(p * sinf(alpha + t) - 2.0 * cosf(alpha + t) + ca + cb) < DUBINS_EPS);
    assert(mod2pi(alpha + t - q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS);
    return DubinsPath(DubinsPath::TYPE_LSR, t, p, q);
  }
  return DubinsPath();
}
// ------------------------- LSR ------------------------- //

// ------------------------- RLR ------------------------- //
DubinsPath DubinsAirplaneStateSpace::dubinsRLR(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsRLR(d, alpha, beta, sa, sb, ca, cb);
}

DubinsPath DubinsAirplaneStateSpace::dubinsRLR(double d, double alpha, double beta, double sa, double sb, double ca,
                                               double cb) const {
  const double tmp = 0.125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
  if (fabs(tmp) < 1.0) {
    const double p = twopi - acosf(tmp);
    const double theta = atan2f(ca - cb, d - sa + sb);
    const double t = mod2pi(alpha - theta + 0.5 * p);
    const double q = mod2pi(alpha - beta - t + p);
    assert(fabs(2.0 * sinf(alpha - t + p) - 2.0 * sinf(alpha - t) - d + sa - sb) < DUBINS_EPS);
    assert(fabs(-2.0 * cosf(alpha - t + p) + 2.0 * cosf(alpha - t) - ca + cb) < DUBINS_EPS);
    assert(mod2pi(alpha - t + p - q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS);
    return DubinsPath(DubinsPath::TYPE_RLR, t, p, q);
  }
  return DubinsPath();
}
// ------------------------- RLR ------------------------- //

// ------------------------- LRL ------------------------- //
DubinsPath DubinsAirplaneStateSpace::dubinsLRL(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsLRL(d, alpha, beta, sa, sb, ca, cb);
}

DubinsPath DubinsAirplaneStateSpace::dubinsLRL(double d, double alpha, double beta, double sa, double sb, double ca,
                                               double cb) const {
  const double tmp = 0.125 * (6.0 - d * d + 2.0 * (ca * cb + sa * sb - d * (sa - sb)));
  if (fabs(tmp) < 1.0) {
    const double p = twopi - acosf(tmp);
    const double theta = atan2f(-ca + cb, d + sa - sb);
    const double t = mod2pi(-alpha + theta + 0.5 * p);
    const double q = mod2pi(beta - alpha - t + p);
    assert(fabs(-2.0 * sinf(alpha + t - p) + 2.0 * sinf(alpha + t) - d - sa + sb) < DUBINS_EPS);
    assert(fabs(2.0 * cosf(alpha + t - p) - 2.0 * cosf(alpha + t) + ca - cb) < DUBINS_EPS);
    assert(mod2pi(alpha + t - p + q - beta + 0.5 * DUBINS_EPS) < DUBINS_EPS);
    return DubinsPath(DubinsPath::TYPE_LRL, t, p, q);
  }
  return DubinsPath();
}
// ------------------------- LRL ------------------------- //

}  // namespace spaces

}  // namespace fw_planning

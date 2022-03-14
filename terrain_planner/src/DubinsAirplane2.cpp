/*
 * DubinsAirplane2.cpp
 *
 *  Created on: Jun 4, 2015
 *      Author: Daniel Schneider, ASL
 *              Florian Achermann, ASL
 *              Philipp Oetthershagen, ASL
 *
 *      Note: See header file for more information on definitions and states.
 */


#include "fw_planning_planning/spaces/DubinsAirplane2.hpp"

#include <assert.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <limits>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/math/constants/constants.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include "fw_planning_planning/base/params.hpp"

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
  if (x < 0 && x > DUBINS_ZERO)
    return 0;

  return x - twopi * floor(x * one_div_twopi);
}


/** \brief sgn
 * Returns +1 for positive sign, -1 for negative sign and 0 if val=0
 * TODO Move it to a general file as this function can be used in many different functions/classes
 */
template<typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}


/*-----------------------------------------------------------------------------------------------*/
/*- StateType: Public Functions------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------*/

DubinsAirplane2StateSpace::StateType::StateType()
    : ob::CompoundStateSpace::StateType() {
}


double DubinsAirplane2StateSpace::StateType::getX() const {
  return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
}


double DubinsAirplane2StateSpace::StateType::getY() const {
  return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
}


double DubinsAirplane2StateSpace::StateType::getZ() const {
  return as<ob::RealVectorStateSpace::StateType>(0)->values[2];
}


double DubinsAirplane2StateSpace::StateType::getYaw() const {
  return as<ob::SO2StateSpace::StateType>(1)->value;
}


void DubinsAirplane2StateSpace::StateType::setX(double x) {
  as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
}


void DubinsAirplane2StateSpace::StateType::setY(double y) {
  as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
}


void DubinsAirplane2StateSpace::StateType::setZ(double z) {
  as<ob::RealVectorStateSpace::StateType>(0)->values[2] = z;
}


void DubinsAirplane2StateSpace::StateType::setYaw(double yaw) {
  as<ob::SO2StateSpace::StateType>(1)->value = yaw;
}


void DubinsAirplane2StateSpace::StateType::setXYZ(double x, double y, double z) {
  setX(x);
  setY(y);
  setZ(z);
}


void DubinsAirplane2StateSpace::StateType::setXYZYaw(double x, double y, double z, double yaw) {
  setX(x);
  setY(y);
  setZ(z);
  setYaw(yaw);
}


void DubinsAirplane2StateSpace::StateType::addToX(double val) {
  as<ob::RealVectorStateSpace::StateType>(0)->values[0] += val;
}


void DubinsAirplane2StateSpace::StateType::addToY(double val) {
  as<ob::RealVectorStateSpace::StateType>(0)->values[1] += val;

}


void DubinsAirplane2StateSpace::StateType::addToZ(double val) {
  as<ob::RealVectorStateSpace::StateType>(0)->values[2] += val;
}


const double* DubinsAirplane2StateSpace::StateType::getPosValuePointer() const {
  return as<ob::RealVectorStateSpace::StateType>(0)->values;
}


void DubinsAirplane2StateSpace::StateType::printState(const std::string& msg) const {
  std::cout << msg << "state (x,y,z,yaw): " << this->getX() << " " << this->getY() << " " << this->getZ() << " "
            << this->getYaw() << std::endl << std::endl;
}


/*-----------------------------------------------------------------------------------------------*/
/*- DubinsAirplane2StateSpace: Protected Functions-----------------------------------------------*/
/*-----------------------------------------------------------------------------------------------*/

DubinsAirplane2StateSpace::DubinsAirplane2StateSpace(double turningRadius,
                                                     double gam,
                                                     bool useEuclDist)
    : ob::CompoundStateSpace(),
      rho_(turningRadius),
      curvature_(1.0 / turningRadius),
      gammaMax_(gam),
      tanGammaMax_(tan(gam)),
      tanGammaMaxInv_(1.0 / tan(gam)),
      sin_gammaMax_(sin(gam)),
      one_div_sin_gammaMax_(1.0 / sin(gam)),
      optimalStSp_(false),
      useWind_(false),
      vAirInv_(1.0 / params::v_air_param),
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
      CWD_windDrift_(),
      CWD_meteoData_(),
      duration_distance_(0.0),
      duration_interpolate_(0.0),
      duration_interpolate_motionValidator_(0.0),
      duration_get_wind_drift_(0.0),
      meteoGrid_(new base::MeteoGridClass()),
      interpol_seg_(0.0),
      interpol_tanGamma_(0.0),
      interpol_phiStart_(0.0),
      interpol_dPhi_(0.0),
      interpol_v_(0.0),
      interpol_iter_(0),
      interpol_tmp_(0.0) {
  setName("DubinsAirplane2" + getName());
  type_ = ob::STATE_SPACE_SE3;

  addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.);
  addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.);
  lock();

  stateInterpolation_ = allocState()->as<StateType>();
  stateComputeWindDrift_ = allocState();
}


DubinsAirplane2StateSpace::~DubinsAirplane2StateSpace() {
  freeState(stateInterpolation_);
  freeState(stateComputeWindDrift_);
}


double DubinsAirplane2StateSpace::getMaximumExtent() const {
  /* For the DubinsAirplane2StateSpace this computes:
   * R3_max_extent + SO2_max_extent = sqrtf(bound_x^2 + bound_y^2 + bound_z^2) + pi */
  double e = 0.0;
  for (unsigned int i = 0; i < componentCount_; ++i)
    if (weights_[i] >= std::numeric_limits<double>::epsilon())  // avoid possible multiplication of 0 times infinity
      e += weights_[i] * components_[i]->getMaximumExtent();
  return e;
}


double DubinsAirplane2StateSpace::getEuclideanExtent() const {
  /* For the DubinsAirplane2StateSpace this computes:
   * R3_max_extent = sqrtf(bound_x^2 + bound_y^2 + bound_z^2) */
  return components_[0]->getMaximumExtent();
}


unsigned int DubinsAirplane2StateSpace::validSegmentCount(const ob::State* state1, const ob::State* state2) const {
  if (!useEuclideanDistance_) {
    double dist = distance(state1, state2);
    if (isnan(dist))
      return 0u;
    else
      return longestValidSegmentCountFactor_ * (unsigned int) ceil(dist / longestValidSegment_);
  } else {
    // still compute the dubins airplane distance.
    useEuclideanDistance_ = false;
    unsigned int nd = longestValidSegmentCountFactor_ * (unsigned int) ceil(distance(state1, state2) / longestValidSegment_);
    useEuclideanDistance_ = true;
    return nd;
  }
}


double DubinsAirplane2StateSpace::distance(const ob::State* state1, const ob::State* state2) const {
  if (useEuclideanDistance_) {
    return euclidean_distance(state1, state2);
  } else {
#if(TIME_ENABLED)
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
    if (useWind_)
      dubinsWithWind(state1, state2, dp_);
    else
      dubins(state1, state2, dp_);

    const double dist = rho_ * dp_.length_3D();

#if(TIME_ENABLED)
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    duration_distance_ += std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() * 1e-9;
#endif
    return dist;
  }
}


double DubinsAirplane2StateSpace::euclidean_distance(const ob::State* state1, const ob::State* state2) const {
#if(TIME_ENABLED)
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
  const DubinsAirplane2StateSpace::StateType *dubinsAirplane2State1 = state1->as<DubinsAirplane2StateSpace::StateType>();
  const DubinsAirplane2StateSpace::StateType *dubinsAirplane2State2 = state2->as<DubinsAirplane2StateSpace::StateType>();

  double eucl_dist =
      ((dubinsAirplane2State1->getX() - dubinsAirplane2State2->getX()) * (dubinsAirplane2State1->getX() - dubinsAirplane2State2->getX())) +
      ((dubinsAirplane2State1->getY() - dubinsAirplane2State2->getY()) * (dubinsAirplane2State1->getY() - dubinsAirplane2State2->getY())) +
      ((dubinsAirplane2State1->getZ() - dubinsAirplane2State2->getZ()) * (dubinsAirplane2State1->getZ() - dubinsAirplane2State2->getZ()));

  eucl_dist = sqrtf(eucl_dist);

  const double dub_dist = fabs(dubinsAirplane2State1->getZ() - dubinsAirplane2State2->getZ()) * one_div_sin_gammaMax_;

#if(TIME_ENABLED)
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  duration_distance_ += std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() * 1e-9;
#endif
  if (useWind_)
    return eucl_dist;
  else
    return std::max(eucl_dist, dub_dist);

}


void DubinsAirplane2StateSpace::interpolate(const ob::State* from, const ob::State* to, const double t, ob::State* state) const {
#if(TIME_ENABLED)
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif

  bool firstTime = true;
  DubinsPath path;
  SegmentStarts segmentStarts;
  interpolate(from, to, t, firstTime, path, segmentStarts, state);

#if(TIME_ENABLED)
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  duration_interpolate_ += std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() * 1e-9;
#endif
}


void DubinsAirplane2StateSpace::interpolate(const ob::State* from, const ob::State* to, double t, bool& firstTime, DubinsPath& path,
                                            SegmentStarts& segmentStarts, ob::State* state) const {
#if(TIME_ENABLED)
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif

  // compute the path if interpolate is called the first time.
  if (firstTime) {
    if (useWind_)
      dubinsWithWind(from, to, path);
    else
      dubins(from, to, path);

    // compute the segment starts
    calculateSegmentStarts(from, path, segmentStarts);

    // this must be after the computation of the dubins path (otherwise a state of an invalid path is returned.
    if (t >= 1.) {
      if (to != state)
        copyState(state, to);
      return;
    }
    if (t <= 0.) {
      if (from != state)
        copyState(state, from);
      return;
    }
    firstTime = false;
  }

  if (isnan(path.length_3D())) {
    state->as<StateType>()->setXYZ(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  } else {
    if (useWind_)
      interpolateWithWind(from, path, segmentStarts, t, state);
    else
      interpolate(path, segmentStarts, t, state);
  }

#if(TIME_ENABLED)
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  duration_interpolate_motionValidator_ += std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() * 1e-9;
#endif
}


void DubinsAirplane2StateSpace::enforceBounds(ob::State* state) const {
  // RE3 part
  StateType *rstate = static_cast<StateType*>(state);
  for (unsigned int i = 0; i < getSubspace(0)->getDimension(); ++i) {
    if (rstate->as<ob::RealVectorStateSpace::StateType>(0)->values[i] > as<ob::RealVectorStateSpace>(0)->getBounds().high[i])
      rstate->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = as<ob::RealVectorStateSpace>(0)->getBounds().high[i];
    else if (rstate->as<ob::RealVectorStateSpace::StateType>(0)->values[i] < as<ob::RealVectorStateSpace>(0)->getBounds().low[i])
      rstate->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = as<ob::RealVectorStateSpace>(0)->getBounds().low[i];
  }

  // Orientation (S02 part)
  getSubspace(1)->enforceBounds(rstate->as<ob::SO2StateSpace::StateType>(1));
}


void DubinsAirplane2StateSpace::setMaxClimbingAngle(double maxClimb) {
  gammaMax_ = maxClimb;
  tanGammaMax_ = tan(gammaMax_);
  tanGammaMaxInv_ = 1.0 / tanGammaMax_;
  sin_gammaMax_ = sin(gammaMax_);
  one_div_sin_gammaMax_ = 1.0 / sin_gammaMax_;
}


double DubinsAirplane2StateSpace::getMaxClimbingAngle() const {
  return gammaMax_;
}


double DubinsAirplane2StateSpace::getOneDivSinGammaMax() const {
  return one_div_sin_gammaMax_;
}


void DubinsAirplane2StateSpace::setMinTurningRadius(double r_min) {
  rho_ = r_min;
  curvature_ = 1 / rho_;
}


double DubinsAirplane2StateSpace::getMinTurningRadius() const {
  return rho_;
}


double DubinsAirplane2StateSpace::getCurvature() const {
  return curvature_;
}


void DubinsAirplane2StateSpace::setUseOptStSp(bool useOptStSp) {
  optimalStSp_ = useOptStSp;
}


void DubinsAirplane2StateSpace::setUseEuclideanDistance(bool useEuclDist) {
  useEuclideanDistance_ = useEuclDist;
}


void DubinsAirplane2StateSpace::setUseWind(bool useWind) {
  useWind_ = useWind;
}


bool DubinsAirplane2StateSpace::getUseWind() const {
  return useWind_;
}


void DubinsAirplane2StateSpace::setDubinsWindPrintXthError(int print_xth_error) {
  dubinsWindPrintXthError_ = print_xth_error;
}


void DubinsAirplane2StateSpace::setMeteoGrid(const std::shared_ptr<base::MeteoGridClass>& meteoGrid) {
  meteoGrid_.reset();
  meteoGrid_ = meteoGrid;
}


std::shared_ptr<base::MeteoGridClass> DubinsAirplane2StateSpace::getMeteoGrid() const {
  return meteoGrid_;
}


bool DubinsAirplane2StateSpace::getUseEuclideanDistance() const {
  return useEuclideanDistance_;
}


bool DubinsAirplane2StateSpace::isMetricSpace() const {
  return false;
}


bool DubinsAirplane2StateSpace::hasSymmetricDistance() const {
  return false;
}


bool DubinsAirplane2StateSpace::hasSymmetricInterpolate() const {
  return false;
}


void DubinsAirplane2StateSpace::sanityChecks() const {
  double zero = std::numeric_limits<double>::epsilon();
  double eps = std::numeric_limits<float>::epsilon();
  int flags = ~(STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND);
  flags &= ~STATESPACE_DISTANCE_SYMMETRIC;
  // don't do the sanity check in case of wind as it takes a long time and therefore blocks the benchmarking
  // we know that for our purpose the functionality is given
  if (!useWind_)
    StateSpace::sanityChecks(zero, eps, flags);
}


void DubinsAirplane2StateSpace::setBounds(const ob::RealVectorBounds &bounds) {
  as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}


const ob::RealVectorBounds& DubinsAirplane2StateSpace::getBounds() const {
  return as<ob::RealVectorStateSpace>(0)->getBounds();
}


void DubinsAirplane2StateSpace::printStateSpaceProperties() const {
  std::cout << "DubinsAirplane2StateSpace [" << getName() << "]" << std::endl;
  std::cout << "  !!! This state space is asymmetric !!!" << std::endl;
  std::cout << "  Airplane speed relative to ground:  9 m/s" << std::endl;
  std::cout << "  Euclidean Distance: " << useEuclideanDistance_ << std::endl;
  std::cout << "  Use wind: " << useWind_ << std::endl;
  std::cout << "  Minimum turning radius: " << rho_ << " m" << std::endl;
  std::cout << "  Maximum climbing angle: " << gammaMax_ << " radian" << std::endl;
  std::cout << "  Using optimal Dubins airplane paths (not working properly): " << optimalStSp_ << std::endl;
  std::cout << "  State space bounds (in meter and radian): [" << this->getBounds().low[0] << " " << this->getBounds().high[0]
            << "], [" << this->getBounds().low[1] << " " << this->getBounds().high[1] << "], [" << this->getBounds().low[2]
            << " " << this->getBounds().high[2] << "], [" << "-pi pi" << ")";
  std::cout << std::endl << std::endl;
}


void DubinsAirplane2StateSpace::printCtrs() const {
  std::cout << "Number samples resulting in a CSC path: " << csc_ctr_ << " (in %: "
            << double(csc_ctr_) / double(csc_ctr_ + ccc_ctr_) << " )" << std::endl;
  std::cout << "Number samples resulting in a CCC path: " << ccc_ctr_ << " (in %: "
            << double(ccc_ctr_) / double(csc_ctr_ + ccc_ctr_) << " )" << std::endl << std::endl;
  std::cout << "Number samples resulting in a long path: " << long_ctr_ << " (in %: "
            << double(long_ctr_) / double(long_ctr_ + short_ctr_) << " )" << std::endl;
  std::cout << "Number samples resulting in a short  path: " << short_ctr_ << " (in %: "
            << double(short_ctr_) / double(long_ctr_ + short_ctr_) << " )" << std::endl << std::endl;

  if ((dp_failed_ctr_ != 0) || (dp_success_ctr_ != 0)) {
    std::cout << "Number of times failed to compute a path in wind:  " << dp_failed_ctr_ << " (in %: "
              << double(dp_failed_ctr_) / double(dp_failed_ctr_ + dp_success_ctr_) << " )" << std::endl;
    if (dp_failed_ctr_) {
      std::cout << "    Not converged:                   " << dp_failed_ctr_ - dp_failed_xy_wind_ctr_ - dp_failed_z_wind_ctr_ << " (in %: "
                    << double(dp_failed_ctr_ - dp_failed_xy_wind_ctr_ - dp_failed_z_wind_ctr_) / double(dp_failed_ctr_) << " )"
                    << std::endl;
      std::cout << "    Too strong wind in xy-direction: " << dp_failed_xy_wind_ctr_ << " (in %: "
                    << double(dp_failed_xy_wind_ctr_) / double(dp_failed_ctr_) << " )" << std::endl;
      std::cout << "    Too strong wind in z-direction:  " << dp_failed_z_wind_ctr_ << " (in %: "
                    << double(dp_failed_z_wind_ctr_) / double(dp_failed_ctr_) << " )" << std::endl;
    }
    std::cout << "Number of times successfully computed a path in wind:  " << dp_success_ctr_ << " (in %: "
              << double(dp_success_ctr_) / double(dp_failed_ctr_ + dp_success_ctr_) << " )" << std::endl << std::endl;
  }
}


void DubinsAirplane2StateSpace::printDurations() {
  duration_interpolate_motionValidator_ -= duration_interpolate_;
  std::cout << "DubinsAirplane2StateSpace" << std::endl;
  std::cout << "  total time in secs in distance function: " << duration_distance_ << " s" << std::endl;
  std::cout << "  total time in secs in interpolate function: " << duration_interpolate_ << " s" << std::endl;
  std::cout
      << "  total time in secs in interpolate function called in checkMotion function of the DubinsMotionValidator class (for DubinsAirplane2StateSpace, this  time is contained in the time of checkMotion): "
      << duration_interpolate_motionValidator_ << " s" << std::endl;
  std::cout << "  total time in secs in calculate wind drift function: " << duration_get_wind_drift_ << " s" << std::endl;
  std::cout << std::endl << std::endl;
}


void DubinsAirplane2StateSpace::resetCtrs() {
  csc_ctr_ = 0;
  ccc_ctr_ = 0;
  long_ctr_ = 0;
  short_ctr_ = 0;
  dp_failed_ctr_ = 0;
  dp_failed_xy_wind_ctr_ = 0;
  dp_failed_z_wind_ctr_ = 0;
  dp_success_ctr_ = 0;
}


void DubinsAirplane2StateSpace::resetDurations() {
  duration_distance_ = 0.0;
  duration_interpolate_ = 0.0;
  duration_interpolate_motionValidator_ = 0.0;
  duration_get_wind_drift_ = 0.0;
}


void DubinsAirplane2StateSpace::printDurationsAndCtrs() {
  printDurations();
  printCtrs();
}


void DubinsAirplane2StateSpace::resetDurationsAndCtrs() {
  resetDurations();
  resetCtrs();
}


/*-----------------------------------------------------------------------------------------------*/
/*- DubinsAirplane2StateSpace: Protected Functions-----------------------------------------------*/
/*-----------------------------------------------------------------------------------------------*/

void DubinsAirplane2StateSpace::dubins(const ob::State* state1, const ob::State* state2, DubinsPath& dp) const {
  // extract state 1
  const double x1 = state1->as<DubinsAirplane2StateSpace::StateType>()->getX();
  const double y1 = state1->as<DubinsAirplane2StateSpace::StateType>()->getY();
  const double z1 = state1->as<DubinsAirplane2StateSpace::StateType>()->getZ();
  const double th1 = state1->as<DubinsAirplane2StateSpace::StateType>()->getYaw();

  // extract state 2
  const double x2 = state2->as<DubinsAirplane2StateSpace::StateType>()->getX();
  const double y2 = state2->as<DubinsAirplane2StateSpace::StateType>()->getY();
  const double z2 = state2->as<DubinsAirplane2StateSpace::StateType>()->getZ();
  const double th2 = state2->as<DubinsAirplane2StateSpace::StateType>()->getYaw();

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
        double phi_i = std::get < 2 > (tuple);
        dp.setFoundOptimalPath(std::get < 1 > (tuple));
        dp.setAdditionalManeuver(true);
        dp.setSegmentLength(std::get < 0 > (tuple), 1);
        dp.setSegmentLength(std::get < 3 > (tuple), 3);
        dp.setSegmentLength(std::get < 4 > (tuple), 4);
        if (dz >= 0) {
          dp.setGamma(gammaMax_);
          dp.setSegmentLength(0.0, 0);
          dp.setSegmentLength(std::get < 2 > (tuple), 2);
        } else {
          dp.setGamma(-gammaMax_);
          dp.setSegmentLength(0.0, 5);
          dp.setSegmentLength(std::get < 2 > (tuple), 2);
        }
      } else {  // CCC cases (does not find any optimal path yet. Flies 2D Dubins car path with adequate climbing rate gamma)

        double rad = std::get < 0 > (tuple);
        dp.setFoundOptimalPath(std::get < 1 > (tuple));
        dp.setSegmentLength(std::get < 2 > (tuple), 1);
        dp.setSegmentLength(std::get < 3 > (tuple), 3);
        dp.setSegmentLength(std::get < 4 > (tuple), 4);

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
        dp.setGamma(atan2f(dz, L + dp.getSegmentLength(0)));  // need to use dp.length_2D since length changed in line before!
      } else {
        dp.setEndHelix(k, 1.0);
        dp.setGamma(atan2f(dz, L + dp.getSegmentLength(5)));  // need to use dp.length_2D since length changed in line before!
      }
    }
  }

  if (dp.getIdx() < 4) {  // CSC cases
    csc_ctr_ += 1;
  } else {
    ccc_ctr_ += 1;
  }
}


bool DubinsAirplane2StateSpace::dubinsWithWind(const ob::State* state1, const ob::State* state2, DubinsPath& dp) const {
  if (meteoGrid_->isSet()) {
    const double goal_x = state2->as<StateType>()->getX();
    const double goal_y = state2->as<StateType>()->getY();
    const double goal_z = state2->as<StateType>()->getZ();
    const double d_z = goal_z - state1->as<StateType>()->getZ();
    const double sign_d_z = std::copysign(1.0, d_z);
    SegmentStarts segmentStarts;

    // set the start state
    ob::State* shifted_goal = allocState();
    shifted_goal->as<StateType>()->setXYZYaw(
        goal_x,
        goal_y,
        goal_z,
        state2->as<StateType>()->getYaw());

    // compute the first path and drift
    dubins(state1, shifted_goal, dp);

    if (isnan(dp.length_2D())) {
      ++dp_failed_ctr_;
      if (dp_failed_ctr_ % dubinsWindPrintXthError_ == 0)
        ROS_INFO_STREAM("Failed to compute a dubins path with wind " << dp_failed_ctr_ << " times.");
      freeState(shifted_goal);
      return false;
    }

    calculateSegmentStarts(state1, dp, segmentStarts);

    DubinsAirplane2StateSpace::WindDrift wind_drift = calculateWindDrift(state1, 1.0, dp, segmentStarts);
    
    // prepare variables for adaptive wind-drift step size between iterations
    DubinsAirplane2StateSpace::WindDrift wind_drift_old = wind_drift;
    DubinsAirplane2StateSpace::WindDrift wind_drift_adj;
    Eigen::Vector3d goal_vector_old, goal_vector, difference_vector;
    goal_vector_old << wind_drift.x, wind_drift.y, wind_drift.z;
    double dist_goal_squared = goal_vector_old.squaredNorm();

    // preparations for stop conditions
    const double dist_goal_squared_initial = dist_goal_squared;
    double dist_goal_squared_old = dist_goal_squared;
    Eigen::Vector3d start_goal_vec_over_length_squared;
    start_goal_vec_over_length_squared << goal_x - state1->as<StateType>()->getX(), 
                                          goal_y - state1->as<StateType>()->getY(), 
                                          goal_z - state1->as<StateType>()->getZ();
    start_goal_vec_over_length_squared /= start_goal_vec_over_length_squared.squaredNorm();
    double relative_excessive_progress; 
    double relative_excessive_progress_old = start_goal_vec_over_length_squared.dot(goal_vector_old);


    double time_inverse = 0.0;
    double wind_drift_multi = 1.0;
    
    // compute shifted state
    shifted_goal->as<StateType>()->addToX(-wind_drift.x);
    shifted_goal->as<StateType>()->addToY(-wind_drift.y);

    // Adjust shift in Z-direction to vertical wind speed (to adjust for different path-length in next iteration)
    if (dp.getAltitudeCase() == DubinsPath::ALT_CASE_HIGH && fabs(wind_drift.z - d_z) > 0.2) { // 0.2 to avoid diverging
      shifted_goal->as<StateType>()->addToZ(-d_z * wind_drift.z / (d_z + wind_drift.z));
    } else {
      shifted_goal->as<StateType>()->addToZ(-wind_drift.z);
    }

    int counter(1);
    while (dist_goal_squared > THRESHOLD_DISTANCE_GOAL_SQUARED) {
      // compute dubins path
      dubins(state1, shifted_goal, dp);

      if (isnan(dp.length_2D())) {
        ++dp_failed_ctr_;
        if (dp_failed_ctr_ % dubinsWindPrintXthError_ == 0)
          ROS_INFO_STREAM("Failed to compute a dubins path " << dp_failed_ctr_ << " times.");
        freeState(shifted_goal);
        return false;
      }

      calculateSegmentStarts(state1, dp, segmentStarts);

      // compute the drift
      wind_drift = calculateWindDrift(state1, 1.0, dp, segmentStarts);

      // calculate vector to goal-state
      goal_vector <<  shifted_goal->as<StateType>()->getX() + wind_drift.x - goal_x,
                      shifted_goal->as<StateType>()->getY() + wind_drift.y - goal_y,
                      shifted_goal->as<StateType>()->getZ() + wind_drift.z - goal_z;
      dist_goal_squared = goal_vector.squaredNorm();

      // difference vector of last goal vector and current one
      difference_vector = goal_vector_old - goal_vector;
      // multiplicator for difference vector to get as close as possible towards goal
      // Max. of 2.0 to avoid fast diverging of path
      if (difference_vector.norm() != 0.0) {
        wind_drift_multi = std::min(MAX_WIND_DRIFT_MULTI, goal_vector_old.norm() / difference_vector.norm());
      } else {
        wind_drift_multi = 1.0;
      }

      // shorten/lengthen wind_drift based on multiplicator
      wind_drift_adj.x = wind_drift_multi * wind_drift.x + (1.0 - wind_drift_multi) * wind_drift_old.x;
      wind_drift_adj.y = wind_drift_multi * wind_drift.y + (1.0 - wind_drift_multi) * wind_drift_old.y;
      wind_drift_adj.z = wind_drift_multi * wind_drift.z + (1.0 - wind_drift_multi) * wind_drift_old.z;

      // store goal vector and wind drift for next iteration
      goal_vector_old = goal_vector;
      wind_drift_old = wind_drift;

      // compute shifted state
      shifted_goal->as<StateType>()->setX(goal_x - wind_drift_adj.x);
      shifted_goal->as<StateType>()->setY(goal_y - wind_drift_adj.y);
      shifted_goal->as<StateType>()->setZ(goal_z - wind_drift_adj.z);

      ++counter;
      
      // calculate relative excessive progress along start-goal-vector (still at start: -1, at goal: 0, tailwind: >0)
      relative_excessive_progress = start_goal_vec_over_length_squared.dot(goal_vector);

      if (((MAX_ITER <= counter) && (dist_goal_squared > THRESHOLD_DISTANCE_GOAL_SQUARED)) || // maximum number of iterations with no solution
          (dist_goal_squared > dist_goal_squared_initial * 4.0) || // diverging goal distance
          (dist_goal_squared > dist_goal_squared_old && ( // while error grows, ...
          (relative_excessive_progress <= relative_excessive_progress_old && relative_excessive_progress_old < -0.3) || // ... no progress along euclidean path while still being far away from goal
          (relative_excessive_progress >= relative_excessive_progress_old && relative_excessive_progress_old > 0.3)))) {
        dp = DubinsPath(DubinsPath::Index::TYPE_LRL);

        // update error statistic and free memory
        ++dp_failed_ctr_;
        if ((fabs(wind_drift.x * time_inverse) > params::v_air_param) || // too strong wind in x direction
          (fabs(wind_drift.y * time_inverse) > params::v_air_param))
          ++dp_failed_xy_wind_ctr_;
        if ((std::copysign(1.0, wind_drift.z) != sign_d_z) && // too strong wind in z direction opposite to dz
            fabs(wind_drift.z * time_inverse) > (params::v_air_param * sin_gammaMax_))
          ++dp_failed_z_wind_ctr_;
        if (dp_failed_ctr_ % dubinsWindPrintXthError_ == 0)
          ROS_INFO_STREAM("Failed to compute a dubins path " << dp_failed_ctr_ << " times.");

        freeState(shifted_goal);
        return false;
      }
      relative_excessive_progress_old = relative_excessive_progress;
      dist_goal_squared_old = dist_goal_squared;
    }

    ++dp_success_ctr_;
    // free memory
    freeState(shifted_goal);
  } else {
    dubins(state1, state2, dp);
  }
  return true;
}


void DubinsAirplane2StateSpace::dubins(double d, double alpha, double beta, DubinsPath& path) const {
  if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS) {

    path = DubinsPath(DubinsPath::TYPE_LSL, 0.0, d, 0.0);

  } else {

    const double sa = sinf(alpha);
    const double sb = sinf(beta);
    const double ca = cosf(alpha);
    const double cb = cosf(beta);

    // TODO: Check if that is necessary for the short path case or if it can be moved inside the bracket of the long distance cases.
    path.setClassification(classifyPath(alpha, beta));

    if (d > (sqrtf(4.0 - pow(fabs(ca) + fabs(cb), 2.0)) + fabs(sa) + fabs(sb))) {  // sufficient condition for optimality of CSC path type
      ++long_ctr_;
      calcDubPathWithClassification(path, d, alpha, beta, sa, sb, ca, cb);

    } else {  // path of type CCC or CSC will be optimal
      ++short_ctr_;
      calcDubPathWithoutClassification(path, d, alpha, beta, sa, sb, ca, cb);

    }
  }
}


void DubinsAirplane2StateSpace::calcDubPathWithClassification(DubinsPath& path, double d, double alpha,
                                   double beta, double sa, double sb, double ca, double cb) const {
  // TODO: Computational speed up could be achieved here by ordering the if-cases according to their relative probability (if known a priori)
  switch (path.getClassification()) {
    case DubinsPath::CLASS_A11: {  // class a_11: optimal path is RSL
      path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      break;
    }
    case DubinsPath::CLASS_A12: {  // class a_12: depending on S_12, optimal path is either RSR (S_12<0) or RSL (S_12>0)
      if (d * cb - 3.0 * sb * cb + sb * ca - cb * sa + ca * sb > 0.0) {  // RSL is optimal
        path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      } else {  // RSL or RSR is optimal
        if (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb)
            - 2.0 * (q_rsl(d, alpha, beta, sa, sb, ca, cb) - pi) > 0.0) {  // S_12>0: RSL is optimal
          path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
        } else {  // S_12<0: RSR is optimal
          path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
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
    case DubinsPath::CLASS_A21: {  // class a_21 (top. equiv. a_12): depending on S_21, optimal path is either LSL (S_21<0) or RSL (S_12>0)
      if (p_lsl(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb)
          - 2.0 * (t_rsl(d, alpha, beta, sa, sb, ca, cb) - pi) < 0.0) {  // S_21<0: LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      } else {  // S_21>0: RSL is optimal
        path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A22: {  // class a_22 (top. equiv. a_33): depending on alpha, beta, S^{1,2}_22,
      // optimal path is  LSL (alpha>beta && S^1_22<0) or RSL (alpha>beta && S^1_22>0)
      //                  RSR (alpha<beta && S^2_22<0) or RSL (alpha<beta && S^2_22>0)
      if (alpha >= beta
          && (p_lsl(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb)
              - 2.0 * (t_rsl(d, alpha, beta, sa, sb, ca, cb) - pi)) < 0.0) {  // LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      } else if (alpha < beta
          && (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb)
              - 2.0 * (q_rsl(d, alpha, beta, sa, sb, ca, cb) - pi)) < 0.0) {  // RSR is optimal
        path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
      } else {  // if( alpha < beta && (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_rsl(d, alpha, beta, sa, sb, ca, cb) - 2*(q_rsl(d, alpha, beta, sa, sb, ca, cb) - pi)) > 0 ) { // RSL is optimal
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
    case DubinsPath::CLASS_A31: {  // class a_31 (top. equiv. to a_13): depending on S_31, optimal path is LSL (S_31<0) or LSR (S_31>0)
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
      if (alpha <= beta
          && (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_lsr(d, alpha, beta, sa, sb, ca, cb)
              - 2.0 * (t_lsr(d, alpha, beta, sa, sb, ca, cb) - pi)) < 0.0) {  // alpha<beta && S^1_33<0: RSR is optimal
        path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
      } else if (alpha > beta
          && (p_lsl(d, alpha, beta, sa, sb, ca, cb) - p_lsr(d, alpha, beta, sa, sb, ca, cb)
              - 2.0 * (q_lsr(d, alpha, beta, sa, sb, ca, cb) - pi)) < 0.0) {  // alpha<beta && S^2_33<0: LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      } else {  // LSR is optimal
        path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A34: {  // class a_34 (top. equiv. to a_12): depending on S_34, optimal path is RSR (S_34<0) or LSR (S_34>0)
      if (p_rsr(d, alpha, beta, sa, sb, ca, cb) - p_lsr(d, alpha, beta, sa, sb, ca, cb)
          - 2.0 * (t_lsr(d, alpha, beta, sa, sb, ca, cb) - pi) < 0) {  // S_34<0: RSR is optimal
        path = dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
      } else {  // S_34>0: LSR is optimal
        path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
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
    case DubinsPath::CLASS_A42: {  // class a_42 (top. equiv. to a_13): depending on S_42, optimal path is LSL (S_42<0) or RSL (S_42>0)
      if (t_lsl(d, alpha, beta, sa, sb, ca, cb) - pi < 0.0) {  // S_42 < 0: LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      } else {  // S_42 > 0: RSL is optimal
        path = dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A43: {  // class a_43 (top. equiv. to a_34): depending on S_43, optimal path is LSL (S_43<0) or LSR (S_43>0)
      if (p_lsl(d, alpha, beta, sa, sb, ca, cb) - p_lsr(d, alpha, beta, sa, sb, ca, cb)
          - 2.0 * (q_lsr(d, alpha, beta, sa, sb, ca, cb) - pi) < 0.0) {  // S_43<0: LSL is optimal
        path = dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
      } else {  // S_43>0: LSR is optimal
        path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
      }
      break;
    }
    case DubinsPath::CLASS_A44: {  // class a_44: optimal path is LSR
      path = dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
      break;
    }
    default: {
      ROS_ERROR_STREAM("default (a not in set{0,1,...,15}), path.a: " << path.getClassification() <<
          " ,d: " << d << " ,alpha: " << alpha << ",beta: " << beta);
      assert(false && "class of path (path.a) was not assigned to an integer in the set {0,1,...,15}.");
    }
  }
}


void DubinsAirplane2StateSpace::calcDubPathWithoutClassification(DubinsPath& path, double d, double alpha,
                                                                 double beta, double sa, double sb,
                                                                 double ca, double cb) const {
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


std::tuple<double, bool, double, double, double> DubinsAirplane2StateSpace::additionalManeuver(const DubinsPath& dp,
                                                                                               double& L_2D,
                                                                                               const ob::State* state1,
                                                                                               const ob::State* state2) const {
  bool foundSol = false;

  // extract state 1
  const double x1 = state1->as<DubinsAirplane2StateSpace::StateType>()->getX();
  const double y1 = state1->as<DubinsAirplane2StateSpace::StateType>()->getY();
  const double z1 = state1->as<DubinsAirplane2StateSpace::StateType>()->getZ();
  const double th1 = state1->as<DubinsAirplane2StateSpace::StateType>()->getYaw();

  // extract state 2
  const double x2 = state2->as<DubinsAirplane2StateSpace::StateType>()->getX();
  const double y2 = state2->as<DubinsAirplane2StateSpace::StateType>()->getY();
  const double z2 = state2->as<DubinsAirplane2StateSpace::StateType>()->getZ();
  const double th2 = state2->as<DubinsAirplane2StateSpace::StateType>()->getYaw();

  const double dx = (x2 - x1) * curvature_; // dx scaled by the radius
  const double dy = (y2 - y1) * curvature_; // dy scaled by the radius
  const double dz = (z2 - z1) * curvature_; // dz scaled by the radius
  const double d = sqrtf(dx * dx + dy * dy); // 2D euclidean distance
  const double th = atan2f(dy, dx);
  const double alpha = mod2pi(th1 - th);
  const double beta = mod2pi(th2 - th);

  const double step = 0.26;  // 0.35 = 20 / 180 * pi, 0.26 = 15. / 180. * pi, 0.17 = 10. / 180. * pi, 0.09 = 5. / 180. * pi

  double L_desired2D = fabs(dz) * tanGammaMaxInv_;

  double error_abs = fabs(L_desired2D - L_2D);
  double error_min_abs = error_abs;

  // allocate variables outside the loop.
  double phi_min = dp.getSegmentLength(1);
  double t_min = 0.0;
  double p_min = dp.getSegmentLength(3);
  double q_min = dp.getSegmentLength(4);
  double x1_c, y1_c, z1_c, dx_c, dy_c, dz_c, d_c, th1_c, th_c, alpha_c, beta_c = 0.0;

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
        x1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getX();
        y1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getY();
        z1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getZ();
        th1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getYaw();
        dx_c = (x2 - x1_c) * curvature_;
        dy_c = (y2 - y1_c) * curvature_;
        dz_c = (z2 - z1_c) * curvature_;
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
        x1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getX();
        y1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getY();
        z1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getZ();
        th1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getYaw();
        dx_c = (x2 - x1_c) * curvature_;
        dy_c = (y2 - y1_c) * curvature_;
        dz_c = (z2 - z1_c) * curvature_;
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
        x1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getX();
        y1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getY();
        z1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getZ();
        th1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getYaw();
        dx_c = (x2 - x1_c) * curvature_;
        dy_c = (y2 - y1_c) * curvature_;
        dz_c = (z2 - z1_c) * curvature_;
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
        x1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getX();
        y1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getY();
        z1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getZ();
        th1_c = si->as<DubinsAirplane2StateSpace::StateType>()->getYaw();
        dx_c = (x2 - x1_c) * curvature_;
        dy_c = (y2 - y1_c) * curvature_;
        dz_c = (z2 - z1_c) * curvature_;
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
    case DubinsPath::TYPE_RLR: { // RLR
      // does not find any adequate solution so far. Returns 2D Dubins car path and flies with adequate climbing rate
      dp_tmp = dubinsRLR(d, alpha, beta); // TODO: Check if that is necessary, isn't that the same path as the input path?
      t_min = dp_tmp.getSegmentLength(1);
      p_min = dp_tmp.getSegmentLength(3);
      q_min = dp_tmp.getSegmentLength(4);
      foundSol = false;
      return std::make_tuple(rho_, foundSol, t_min, p_min, q_min);
    }
    case DubinsPath::TYPE_LRL: { // LRL
      // does not find any adqeuate solution so far. Returns 2D Dubins car path and flies with adequate climbing rate
      dp_tmp = dubinsLRL(d, alpha, beta); // TODO: Check if that is necessary, isn't that the same path as the input path?
      t_min = dp_tmp.getSegmentLength(1);
      p_min = dp_tmp.getSegmentLength(3);
      q_min = dp_tmp.getSegmentLength(4);
      foundSol = false;
      return std::make_tuple(rho_, foundSol, t_min, p_min, q_min);
    }
    default: {
      ROS_ERROR_STREAM("DubinsAirplane2::additionalManeuver: Invalid path index: "<< dp.getIdx());
      return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0);
    }
  }
}


DubinsPath::Classification DubinsAirplane2StateSpace::classifyPath(double alpha, double beta) const {
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
  assert(column >= 1 && column <= 4 && "beta is not in the range of [0,2pi] in classifyPath(double alpha, double beta).");
  assert((column - 1) + 4 * (row - 1) >= 0 && (column - 1) + 4 * (row - 1) <= 15 && "class is not in range [0,15].");
  return (DubinsPath::Classification) ((column - 1) + 4 * (row - 1));
}

double DubinsAirplane2StateSpace::computeOptRratio(double fabsHdist, double L, double fabsTanGamma, int k) const {
  return (fabsHdist - L * fabsTanGamma) / (twopi * fabsTanGamma * k);
}


void DubinsAirplane2StateSpace::interpolate(const DubinsPath &path, const SegmentStarts& segmentStarts,
                                            double t, ob::State* state) const {
  interpol_seg_ = t * path.length_2D();
  if (path.getGamma() == gammaMax_)
    interpol_tanGamma_ = tanGammaMax_;
  else if(path.getGamma() == -gammaMax_)
    interpol_tanGamma_ = -tanGammaMax_;
  else
    interpol_tanGamma_ = tanf(path.getGamma());

  for (interpol_iter_ = 0u; interpol_iter_ < 6u; ++interpol_iter_) {
    if ((interpol_seg_ < path.getSegmentLength(interpol_iter_) || (interpol_iter_ == 5u) )) {
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
            ROS_ERROR("This should never happen, otherwise something wrong in the DubinsAirplane2StateSpace::interpolate"
                "(const ob::State *from, const DubinsPath &path, double t, ob::State *state) const function.");
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
  ROS_ERROR("This should never happen, otherwise something wrong in the DubinsAirplane2StateSpace::interpolate"
      "(const ob::State *from, const DubinsPath &path, double t, ob::State *state) const function.");
  return;
}


void DubinsAirplane2StateSpace::interpolateWithWind(const ob::State* from, const DubinsPath &path,
                                                    const SegmentStarts& segmentStarts, double t, ob::State* state) const {
  if (meteoGrid_->isSet()) {
    if (isnan(path.length_3D())) {
      state->as<StateType>()->setXYZ(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    } else {
      interpolate(path, segmentStarts, t, state);
      DubinsAirplane2StateSpace::WindDrift drift = calculateWindDrift(from, t, path, segmentStarts);

      state->as<StateType>()->addToX(drift.x);
      state->as<StateType>()->addToY(drift.y);
      state->as<StateType>()->addToZ(drift.z);
    }
  } else {
    interpolate(path, segmentStarts, t, state);
  }
}


void DubinsAirplane2StateSpace::calculateSegmentStarts(const ob::State* from, const DubinsPath& path,
                                                       SegmentStarts& segmentStarts) const {
  if (isnan(path.length_2D()))
    return;

  interpol_seg_ = path.length_2D();
  if (path.getGamma() == gammaMax_)
    interpol_tanGamma_ = tanGammaMax_;
  else if(path.getGamma() == -gammaMax_)
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
          ROS_ERROR("This should never happen, otherwise something wrong in the DubinsAirplane2StateSpace::interpolate"
              "(const ob::State *from, const DubinsPath &path, double t, ob::State *state) const function.");
          break;
        }
    }
  }
}


void DubinsAirplane2StateSpace::getStateOnCircle(const ob::State* from,
                                                 int rl /* right (0), left (1)*/,
                                                 int ud /* up(0), down(1) */,
                                                 double t,
                                                 ob::State *state) const {
  // assuming flying with rho_ and gammaMax_
  StateType *s = allocState()->as<StateType>();

  s->setXYZ(0.0, 0.0, 0.0);
  s->setYaw(from->as<StateType>()->getYaw());
  const double phi = s->getYaw();

  switch (rl) {
    case 1:  // left
      // TODO: precompute tanf(gammaMax_)
      s->setXYZ(s->getX() + sinf(phi + t) - sinf(phi), s->getY() - cosf(phi + t) + cosf(phi), s->getZ() + t * tanf(ud * gammaMax_));
      s->setYaw(phi + t);
      break;

    case 0:  // right
      // TODO: precompute tanf(gammaMax_)
      s->setXYZ(s->getX() - sinf(phi - t) + sinf(phi), s->getY() + cosf(phi - t) - cosf(phi), s->getZ() + t * tanf(ud * gammaMax_));
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


unsigned int DubinsAirplane2StateSpace::convert_idx(unsigned int i) const {
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


DubinsAirplane2StateSpace::WindDrift DubinsAirplane2StateSpace::calculateWindDrift(const ob::State* from, double t,
                                                                                   const DubinsPath& dp, const SegmentStarts& segmentStarts) const {
#if(TIME_ENABLED)
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
  if (isnan(dp.length_3D())) {
    CWD_windDrift_.x = std::numeric_limits<double>::infinity();
    CWD_windDrift_.y = std::numeric_limits<double>::infinity();
    CWD_windDrift_.z = std::numeric_limits<double>::infinity();
    ROS_ERROR_STREAM("calculateWindDrift: The length of the input path (" << dp.length_3D() <<
        ") is nan. Returning infinity wind drift");
    return CWD_windDrift_;
  } else {
    CWD_windDrift_.x = 0.0;
    CWD_windDrift_.y = 0.0;
    CWD_windDrift_.z = 0.0;
  }

  switch (params::interpolation_approach_param) {
    case 0: {
      //Fixed step-size start-point integration approach (euler forward)
      
      const double dt_max = meteoGrid_->getResolution() * params::fixed_interpolation_stepsize_param / (rho_ * dp.length_3D());
      const double factor = rho_ * dp.length_3D() * vAirInv_;
      double t_interpol(0.0);
      double dt(0.0);
      
      while (t_interpol < t) {
        // get the current state
        interpolate(dp, segmentStarts, t_interpol, stateComputeWindDrift_);

        // geth the meteo data at the interpolated shifted state
        meteoGrid_->getMeteoData(
            stateComputeWindDrift_->as<StateType>()->getX() + CWD_windDrift_.x,
            stateComputeWindDrift_->as<StateType>()->getY() + CWD_windDrift_.y,
            stateComputeWindDrift_->as<StateType>()->getZ() + CWD_windDrift_.z,
            CWD_meteoData_);

        // Choose right dt also for last segment.
        dt = std::min(dt_max, t - t_interpol);

        // update the shift with the current meteo data
        CWD_windDrift_.x += dt * factor * CWD_meteoData_.u;
        CWD_windDrift_.y += dt * factor * CWD_meteoData_.v;
        CWD_windDrift_.z += dt * factor * CWD_meteoData_.w;

        // update the interpolation time
        t_interpol += dt;
      }
      break;
    }
    case 1: {
      //Fixed step 4th order Runge-Kutta

      const double h_opt = meteoGrid_->getResolution() * params::fixed_RK4_interpolation_stepsize_param / (rho_ * dp.length_3D());
      const double factor = rho_ * dp.length_3D() * vAirInv_;
      double t_interpol(0.0);
      double h(0.0);
      Eigen::Vector3d k1, k2, k3, k4;

      // get the current state
      interpolate(dp, segmentStarts, t_interpol, stateComputeWindDrift_);

      while (t_interpol < t) {
        // Choose right dt also for last segment.
        h = std::min(h_opt, t - t_interpol);

        // geth the meteo data at the interpolated shifted state
        meteoGrid_->getMeteoData(
            stateComputeWindDrift_->as<StateType>()->getX() + CWD_windDrift_.x,
            stateComputeWindDrift_->as<StateType>()->getY() + CWD_windDrift_.y,
            stateComputeWindDrift_->as<StateType>()->getZ() + CWD_windDrift_.z,
            CWD_meteoData_);
        k1 << CWD_meteoData_.u, CWD_meteoData_.v, CWD_meteoData_.w;

        // get the current state
        interpolate(dp, segmentStarts, t_interpol + h * 0.5, stateComputeWindDrift_);

        meteoGrid_->getMeteoData(
            stateComputeWindDrift_->as<StateType>()->getX() + CWD_windDrift_.x + k1.x() * h * factor * 0.5,
            stateComputeWindDrift_->as<StateType>()->getY() + CWD_windDrift_.y + k1.y() * h * factor * 0.5,
            stateComputeWindDrift_->as<StateType>()->getZ() + CWD_windDrift_.z + k1.z() * h * factor * 0.5,
            CWD_meteoData_);
        k2 << CWD_meteoData_.u, CWD_meteoData_.v, CWD_meteoData_.w;

        meteoGrid_->getMeteoData(
            stateComputeWindDrift_->as<StateType>()->getX() + CWD_windDrift_.x + k2.x() * h * factor * 0.5,
            stateComputeWindDrift_->as<StateType>()->getY() + CWD_windDrift_.y + k2.y() * h * factor * 0.5,
            stateComputeWindDrift_->as<StateType>()->getZ() + CWD_windDrift_.z + k2.z() * h * factor * 0.5,
            CWD_meteoData_);
        k3 << CWD_meteoData_.u, CWD_meteoData_.v, CWD_meteoData_.w;

        // get the current state
        interpolate(dp, segmentStarts, t_interpol + h, stateComputeWindDrift_);

        meteoGrid_->getMeteoData(
            stateComputeWindDrift_->as<StateType>()->getX() + CWD_windDrift_.x + k3.x() * h * factor,
            stateComputeWindDrift_->as<StateType>()->getY() + CWD_windDrift_.y + k3.y() * h * factor,
            stateComputeWindDrift_->as<StateType>()->getZ() + CWD_windDrift_.z + k3.z() * h * factor,
            CWD_meteoData_);
        k4 << CWD_meteoData_.u, CWD_meteoData_.v, CWD_meteoData_.w;

        k4 = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        // update the shift with the current meteo data
        CWD_windDrift_.x += h * factor * k4.x();
        CWD_windDrift_.y += h * factor * k4.y();
        CWD_windDrift_.z += h * factor * k4.z();

        // update the interpolation time
        t_interpol += h;
      }
      break;
    }
    case 2: {
      // Adaptive step 3rd order RK (Bogacki-Shampine)
      
      // Parameter 0.5: Minimum 2 iteration steps, even for short paths.
      double h = std::min(t, meteoGrid_->getResolution() * params::adaptive_RK3_interpolation_stepsize_param / (rho_ * dp.length_3D()));
      const double h_min = h * params::adaptive_RK3_interpolation_stepsize_min_param;
      const double h_max = std::min(t, h * params::adaptive_RK3_interpolation_stepsize_max_param);
      const double factor = rho_ * dp.length_3D() * vAirInv_;
      // Path-length dependent tolerance: Long paths require higher accuracy, as allowed goal-offset is constant.
      // Minimum tolerance constant for all paths under certain length (second parameter).
      const double tolerance = params::adaptive_RK3_interpolation_tolerance_param / 
          std::max(params::adaptive_RK3_interpolation_tolerance_min_length_param * vAirInv_, factor);
      double t_interpol(0.0);
      Eigen::Vector3d k1, k2, k3, k4, ynp1, znp1, error;
      
      // get the current state
      interpolate(dp, segmentStarts, t_interpol, stateComputeWindDrift_);
      // geth the meteo data at the interpolated shifted state
      meteoGrid_->getMeteoData(
          stateComputeWindDrift_->as<StateType>()->getX(),
          stateComputeWindDrift_->as<StateType>()->getY(),
          stateComputeWindDrift_->as<StateType>()->getZ(),
          CWD_meteoData_);
      k1 << CWD_meteoData_.u, CWD_meteoData_.v, CWD_meteoData_.w;

      double error_norm, h_multiplicator;
      while (t_interpol < t) {
        // get the current state
        interpolate(dp, segmentStarts, t_interpol + h * 0.5, stateComputeWindDrift_);

        meteoGrid_->getMeteoData(
            stateComputeWindDrift_->as<StateType>()->getX() + CWD_windDrift_.x + k1.x() * h * factor * 0.5,
            stateComputeWindDrift_->as<StateType>()->getY() + CWD_windDrift_.y + k1.y() * h * factor * 0.5,
            stateComputeWindDrift_->as<StateType>()->getZ() + CWD_windDrift_.z + k1.z() * h * factor * 0.5,
            CWD_meteoData_);
        k2 << CWD_meteoData_.u, CWD_meteoData_.v, CWD_meteoData_.w;

        interpolate(dp, segmentStarts, t_interpol + h * 0.75, stateComputeWindDrift_);

        meteoGrid_->getMeteoData(
            stateComputeWindDrift_->as<StateType>()->getX() + CWD_windDrift_.x + k2.x() * h * factor * 0.75,
            stateComputeWindDrift_->as<StateType>()->getY() + CWD_windDrift_.y + k2.y() * h * factor * 0.75,
            stateComputeWindDrift_->as<StateType>()->getZ() + CWD_windDrift_.z + k2.z() * h * factor * 0.75,
            CWD_meteoData_);
        k3 << CWD_meteoData_.u, CWD_meteoData_.v, CWD_meteoData_.w;

        ynp1 = (2.0 * k1 + 3.0 * k2 + 4.0 * k3) / 9.0;

        interpolate(dp, segmentStarts, t_interpol + h, stateComputeWindDrift_);

        meteoGrid_->getMeteoData(
            stateComputeWindDrift_->as<StateType>()->getX() + CWD_windDrift_.x + h * factor * ynp1.x(),
            stateComputeWindDrift_->as<StateType>()->getY() + CWD_windDrift_.y + h * factor * ynp1.y(),
            stateComputeWindDrift_->as<StateType>()->getZ() + CWD_windDrift_.z + h * factor * ynp1.z(),
            CWD_meteoData_);
        k4 << CWD_meteoData_.u, CWD_meteoData_.v, CWD_meteoData_.w;

        znp1 = (7.0 * k1 + 6.0 * k2 + 8.0 * k3 + 3.0 * k4) / 24.0;

        error = znp1 - ynp1;
        error_norm = error.norm();
        // MAX/MIN_STEP_CHANGE: Limit maximum change of step size to a factor larger/smaller.
        h_multiplicator = std::max(std::min(
            std::cbrt(tolerance / error_norm), 
            params::adaptive_RK3_interpolation_max_step_change), 
            params::adaptive_RK3_interpolation_min_step_change);
        
        if (h_multiplicator < params::adaptive_RK3_interpolation_multiplier_threshold_param && h > h_min) {
          h = std::max(h_min, h * h_multiplicator);
        } else {
          // update the shift with the current meteo data
          CWD_windDrift_.x += h * factor * ynp1.x();
          CWD_windDrift_.y += h * factor * ynp1.y();
          CWD_windDrift_.z += h * factor * ynp1.z();

          t_interpol += h;
          h = std::min(h_max, std::min(std::max(h_min, h * h_multiplicator), t - t_interpol));
          k1 = k4;
        }
      }
      break;
    }
    default: {
      ROS_ERROR_STREAM("DubinsAirplane2StateSpace::calculateWindDrift: Invalid interpolation approach in param-file.");
    }
  }

#if(TIME_ENABLED)
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  duration_get_wind_drift_ += std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() * 1e-9;
#endif
  return CWD_windDrift_;
}


double DubinsAirplane2StateSpace::t_lsr(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
  const double p = sqrtf(std::max(tmp, 0.0));
  const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
  return mod2pi(-alpha + theta);  // t
}


double DubinsAirplane2StateSpace::p_lsr(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
  return sqrtf(std::max(tmp, 0.0));  // p
}


double DubinsAirplane2StateSpace::q_lsr(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
  const double p = sqrtf(std::max(tmp, 0.0));
  const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
  return mod2pi(-beta + theta);  // q
}


double DubinsAirplane2StateSpace::t_rsl(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
  const double p = sqrtf(std::max(tmp, 0.0));
  const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
  return mod2pi(alpha - theta);  // t
}


double DubinsAirplane2StateSpace::p_rsl(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
  return sqrtf(std::max(tmp, 0.0));  // p
}


double DubinsAirplane2StateSpace::q_rsl(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
  const double p = sqrtf(std::max(tmp, 0.));
  const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
  return mod2pi(beta - theta);  // q
}


double DubinsAirplane2StateSpace::t_rsr(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double theta = atan2f(ca - cb, d - sa + sb);
  return mod2pi(alpha - theta);  // t
}


double DubinsAirplane2StateSpace::p_rsr(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
  return sqrtf(std::max(tmp, 0.0));  // p
}


double DubinsAirplane2StateSpace::q_rsr(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double theta = atan2f(ca - cb, d - sa + sb);
  return mod2pi(-beta + theta);  // q
}


double DubinsAirplane2StateSpace::t_lsl(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double theta = atan2f(cb - ca, d + sa - sb);
  return mod2pi(-alpha + theta);  // t
}


double DubinsAirplane2StateSpace::p_lsl(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
  return sqrtf(std::max(tmp, 0.0));  // p
}


double DubinsAirplane2StateSpace::q_lsl(double d, double alpha, double beta,
                                        double sa, double sb, double ca, double cb) const {
  const double theta = atan2f(cb - ca, d + sa - sb);
  return mod2pi(beta - theta);  // q
}


// ------------------------- LSL ------------------------- //
DubinsPath DubinsAirplane2StateSpace::dubinsLSL(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsLSL(d, alpha, beta, sa, sb, ca, cb);
}


DubinsPath DubinsAirplane2StateSpace::dubinsLSL(double d, double alpha, double beta, double sa,
                                                double sb, double ca, double cb) const {
  double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
  if (tmp >= DUBINS_ZERO) { // TODO Check if fabs is missing.
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
DubinsPath DubinsAirplane2StateSpace::dubinsRSR(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsRSR(d, alpha, beta, sa, sb, ca, cb);
}


DubinsPath DubinsAirplane2StateSpace::dubinsRSR(double d, double alpha, double beta, double sa,
                                                double sb, double ca, double cb) const {

  const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
  if (tmp >= DUBINS_ZERO) { //TODO Check if fabs is missing.
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
DubinsPath DubinsAirplane2StateSpace::dubinsRSL(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsRSL(d, alpha, beta, sa, sb, ca, cb);
}


DubinsPath DubinsAirplane2StateSpace::dubinsRSL(double d, double alpha, double beta, double sa,
                                                double sb, double ca, double cb) const {
  const double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
  if (tmp >= DUBINS_ZERO) { //TODO Check if here fabs is missing.
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
DubinsPath DubinsAirplane2StateSpace::dubinsLSR(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsLSR(d, alpha, beta, sa, sb, ca, cb);
}


DubinsPath DubinsAirplane2StateSpace::dubinsLSR(double d, double alpha, double beta, double sa,
                                                double sb, double ca, double cb) const {
  const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
  if (tmp >= DUBINS_ZERO) { //TODO Check if here fabs is missing.
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
DubinsPath DubinsAirplane2StateSpace::dubinsRLR(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsRLR(d, alpha, beta, sa, sb, ca, cb);
}


DubinsPath DubinsAirplane2StateSpace::dubinsRLR(double d, double alpha, double beta, double sa,
                                                double sb, double ca, double cb) const {
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
DubinsPath DubinsAirplane2StateSpace::dubinsLRL(double d, double alpha, double beta) const {
  const double ca = cosf(alpha);
  const double sa = sinf(alpha);
  const double cb = cosf(beta);
  const double sb = sinf(beta);

  return dubinsLRL(d, alpha, beta, sa, sb, ca, cb);
}


DubinsPath DubinsAirplane2StateSpace::dubinsLRL(double d, double alpha, double beta, double sa,
                                                double sb, double ca, double cb) const {
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

} // namespace spaces

} // namespace fw_planning

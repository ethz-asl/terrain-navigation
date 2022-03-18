/*!
 * \file params.hpp
 *
 *  Created on: Jan 04, 2016
 *      Author: Daniel Schneider, ASL
 *              Florian Achermann, ASL
 */

#ifndef FW_PLANNING_PLANNING__BASE__PARAMS_HPP_
#define FW_PLANNING_PLANNING__BASE__PARAMS_HPP_

#include <vector>

namespace fw_planning {

namespace params {

/** \brief Dimensions of box representing airplane (length x width x height).
 *  29.9m to avoid special cases if bounding box has the same size as the map resolution.
 */
const std::vector<double> airplane_bb_param = {29.9, 29.9, 29.9};

/** \brief Turning radius in meter.
 * Assuming a velocity of V = 9m/s, acceleration of gravity of 9.81m/s^2 this corresponds to a roll angle of:
 * 18.3deg = arctan(V_gnd^2/g/minTurningRadius_param)
 */
const double minTurningRadius_param = 25.0;  // [m]

/** \brief Airspeed of the airplane */
const double v_air_param = 15.0;  // m/s

/** Gravitational acceleration */
const double g_param = 9.81;  // [m/s^2]

/** Corresponding maximum roll angle of the airplane. */
const double maxRollingAngle_param = atan(v_air_param * v_air_param / (minTurningRadius_param * g_param));  // radian

/** \brief Climbing rate in radians.
 * Assuming a velocity of V_gnd = 9m/s, this corresponds to a climbing rate of 1.36m/s =
 * V_gnd*tan(maxClimbingAngle_param) or 1.36m in/decrease in height per 9m path length.
 */
const double maxClimbingAngle_param = 0.15;  // [radian]

/** Cell size in meter for a projection. Used for some planners (PDST, KPIECE). */
static double proj_cellsize_param[3] = {25.0 /* x */, 25.0 /* y */, 25.0 /* z */};

/** Interpolation approach:
 *      0: Fixed step size startpoint-step integration
 *      1: Fixed step 4th order Runge Kutta
 *      2: Adaptive step 3th order Runge Kutta
 */
const int interpolation_approach_param = 2;

/** Step size for integration approach 0, represents fraction of meteo-grid-resolution. */
const double fixed_interpolation_stepsize_param = 0.2;

/** Step size for integration approach 1, represents fraction of meteo-grid-resolution. */
const double fixed_RK4_interpolation_stepsize_param = 5.0;

/** Step size for integration approach 2, represents fraction of meteo-grid-resolution. */
const double adaptive_RK3_interpolation_stepsize_param = 2.0;

/** Minimum step size for integration approach 2, represents fraction of step-size. */
const double adaptive_RK3_interpolation_stepsize_min_param = 0.4;

/** Maximum step size for integration approach 2, represents fraction of step-size. */
const double adaptive_RK3_interpolation_stepsize_max_param = 5.0;

/** Tolerance for integration approach 2, represents a maximum expected error in meters. */
const double adaptive_RK3_interpolation_tolerance_param = 10.0;

/** Min path length for distance dependant tolerance in meters. */
const double adaptive_RK3_interpolation_tolerance_min_length_param = 2000.0;

/** Multiplier threshold for recalculation of current step. If below this value, step is recalculated with new h. */
const double adaptive_RK3_interpolation_multiplier_threshold_param = 0.8;

/** Step size can only increase by this factor per step. */
const double adaptive_RK3_interpolation_max_step_change = 3.0;

/** Step size can only decrease by this factor per step. */
const double adaptive_RK3_interpolation_min_step_change = 0.333;

}  // namespace params

}  // namespace fw_planning

#endif  // FW_PLANNING_PLANNING__BASE__PARAMS_HPP_

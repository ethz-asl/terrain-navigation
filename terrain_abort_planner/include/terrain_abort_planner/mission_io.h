#include "terrain_navigation/path.h"

Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw) {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Vector4d q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;

  q.normalize();

  return q;
}


PathSegment generateTrajectory(const double curvature, const double length, Eigen::Vector3d segment_start,
                               Eigen::Vector3d segment_end, Eigen::Vector3d current_vel, const double dt) {
  PathSegment trajectory;
  trajectory.states.clear();
  double progress = 0.0;
  double flightpath_angle = std::sin((segment_end(2) - segment_start(2)) / length);
  trajectory.flightpath_angle = flightpath_angle;
  /// TODO: Fix sign conventions for curvature
  trajectory.curvature = curvature;
  trajectory.dt = dt;
  // if (std::abs(curvature) < 0.0001) {
  //   curvature > 0.0 ? trajectory.curvature = 0.0001 : trajectory.curvature = -0.0001;
  // }

  double current_yaw = std::atan2(-1.0 * current_vel(1), current_vel(0));
  Eigen::Vector2d arc_center;
  if (std::abs(curvature) > 0.001) {
    arc_center = PathSegment::getArcCenter(trajectory.curvature, segment_start.head(2), current_vel.head(2),
                                           segment_end.head(2));
    double radial_yaw = std::atan2(-segment_start(1) + arc_center(1), segment_start(0) - arc_center(0));
    current_yaw = curvature > 0.0 ? radial_yaw - 0.5 * M_PI : radial_yaw + 0.5 * M_PI;
  }

  for (int i = 0; i < std::max(1.0, length / dt); i++) {
    State state_vector;
    double yaw = -trajectory.curvature * progress + current_yaw;
    if (std::abs(curvature) < 0.001) {
      Eigen::Vector3d tangent = (segment_end - segment_start).normalized();
      Eigen::Vector3d pos = progress * tangent + segment_start;

      Eigen::Vector3d vel = current_vel;
      const double roll = std::atan(trajectory.curvature / 9.81);
      const double pitch = flightpath_angle;
      Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);  // TODO: why the hell do you need to reverse signs?

      state_vector.position = pos;
      state_vector.velocity = vel;
      state_vector.attitude = att;
    } else {
      Eigen::Vector3d pos = (-1.0 / trajectory.curvature) * Eigen::Vector3d(std::sin(yaw) - std::sin(current_yaw),
                                                                            std::cos(yaw) - std::cos(current_yaw), 0) +
                            Eigen::Vector3d(0, 0, progress * std::sin(flightpath_angle)) + segment_start;

      Eigen::Vector3d vel = Eigen::Vector3d(std::cos(flightpath_angle) * std::cos(yaw),
                                            -std::cos(flightpath_angle) * std::sin(yaw), std::sin(flightpath_angle));
      const double roll = std::atan(trajectory.curvature / 9.81);
      const double pitch = flightpath_angle;
      Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);  // TODO: why the hell do you need to reverse signs?

      state_vector.position = pos;
      state_vector.velocity = vel;
      state_vector.attitude = att;
    }
    trajectory.states.push_back(state_vector);

    progress = progress + dt;
  }
  double end_yaw = -trajectory.curvature * length + current_yaw;
  State end_state_vector;
  end_state_vector.position = segment_end;
  end_state_vector.velocity = Eigen::Vector3d(std::cos(end_yaw), -std::sin(end_yaw), 0.0);
  end_state_vector.attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  trajectory.states.push_back(end_state_vector);
  return trajectory;
}


/**
 * @brief 
 * 
 * @param waypoint_list 
 * @param radius 
 * @return Path 
 */
Path generatePathFromWaypoints (std::vector<Eigen::Vector3d> waypoint_list, const double radius);

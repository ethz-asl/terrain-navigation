#include "terrain_planner/primitive_planner.h"

TrajectorySegments PrimitivePlanner::solve(const Eigen::Vector3d current_pos, const Eigen::Vector3d current_vel,
                                           const Eigen::Vector4d current_att, TrajectorySegments &current_path) {
  /// TODO: Return best next child with current segment
  maneuver_library_->generateMotionPrimitives(current_pos, current_vel, current_att, current_path);
  maneuver_library_->Solve();
  TrajectorySegments best_primitive = maneuver_library_->getBestPrimitive();
  return best_primitive;
}

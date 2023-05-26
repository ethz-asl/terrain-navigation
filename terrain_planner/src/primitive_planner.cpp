#include "terrain_planner/primitive_planner.h"

void PrimitivePlanner::setup(const Eigen::Vector3d current_pos, const Eigen::Vector3d current_vel,
                             const Eigen::Vector4d current_att, Path &current_path) {
  /// TODO: Return best next child with current segment
  maneuver_library_->generateMotionPrimitives(current_pos, current_vel, current_att, current_path);
}

Path PrimitivePlanner::solve(const double time) {
  maneuver_library_->Solve();
  Path best_primitive = maneuver_library_->getBestPrimitive();
  return best_primitive;
}

// bool PrimitivePlanner::checkViewUtilityTree(std::shared_ptr<Primitive> primitive) {
//   if (primitive->valid()) {
//     std::vector<ViewPoint> primitive_viewpoints =
//     maneuver_library_->sampleViewPointFromTrajectory(primitive->segment); double view_utility =
//     viewutility_map_->CalculateViewUtility(primitive_viewpoints, false); primitive->utility = view_utility;
//   } else {
//     primitive->utility = 0.0;
//   }

//   if (primitive->has_child()) {
//     for (auto &child : primitive->child_primitives) {
//       checkViewUtilityTree(child);
//     }
//   }

//   return true;
// }

bool PrimitivePlanner::setGoalPosition(const Eigen::Vector3d position) {
  if (maneuver_library_) {
    maneuver_library_->setGoalPosition(position);
    return true;
  }
  return false;
}

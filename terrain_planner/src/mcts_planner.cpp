#include "terrain_planner/mcts_planner.h"

TrajectorySegments MctsPlanner::solve(const Eigen::Vector3d current_pos, const Eigen::Vector3d current_vel,
                                      const Eigen::Vector4d current_att, TrajectorySegments &current_path) {
  int iter{0};
  int max_iter{10};

  Trajectory current_segment;
  if (!current_path.segments.empty()) {
    current_segment = current_path.getCurrentSegment(current_pos);
  } else {
    State state_vector;
    state_vector.position = current_pos;
    state_vector.velocity = current_vel.normalized();
    state_vector.attitude = current_att;
    current_segment.states.push_back(state_vector);
  }
  tree_.reset();
  tree_ = std::make_shared<Primitive>(current_segment);

  while (true) {
    if (iter > max_iter) break;
    rollout(current_pos, current_vel, current_att);
    iter++;
  }

  /// TODO: Return best next child with current segment

  // Get Best Motion Primitives
  TrajectorySegments best_primitive;
  best_primitive.appendSegment(tree_->segment);
  std::shared_ptr<Primitive> best_child = tree_->getBestChild();
  best_primitive.appendSegment(best_child->segment);
  /// TODO: Check validity of best primitive more systematically
  best_primitive.validity = tree_->valid() && best_child->valid();
  return best_primitive;
}

TrajectorySegments MctsPlanner::rollout(const Eigen::Vector3d current_pos, const Eigen::Vector3d current_vel,
                                        const Eigen::Vector4d current_att) {
  std::vector<std::shared_ptr<Primitive>> path;
  // root
  if (!tree_) {
    Trajectory current_segment;
    State state_vector;
    state_vector.position = current_pos;
    state_vector.velocity = current_vel.normalized();
    state_vector.attitude = current_att;
    current_segment.states.push_back(state_vector);
    tree_ = std::make_shared<Primitive>(current_segment);
  }

  /// Traverse Tree
  auto leaf = treePolicy(tree_, path);
  assert(!leaf);
  /// Default policy
  double utility = defaultPolicy(leaf, path);

  /// Backup(vd+1, ∆)
  backup(path, utility);
  // Get Best Motion Primitivef
  TrajectorySegments path_primitive;

  for (auto &node : path) {
    path_primitive.appendSegment(node->segment);
  }
  return path_primitive;
}

std::shared_ptr<Primitive> MctsPlanner::treePolicy(std::shared_ptr<Primitive> leaf,
                                                   std::vector<std::shared_ptr<Primitive>> &path) {
  double planning_horizon{10.0};
  while (!is_terminal(leaf)) {
    path.push_back(leaf);
    if (has_expandable_child(leaf)) {
      if (!leaf->has_child() && leaf->valid()) {
        maneuver_library_->expandPrimitives(leaf, maneuver_library_->getPrimitiveRates(), planning_horizon);
        /// TODO: Run MCTS with and without collisions if no terrain map is not set
        // std::vector<TrajectorySegments> valid_primitives;
        // maneuver_library_->checkCollisionsTree(leaf, valid_primitives);
        if (leaf->has_validchild()) {
          leaf = leaf->getValidChild();
          path.push_back(leaf);
        }
      } else {
        /// TODO: get valid random child
        leaf = getUnvisitedChild(leaf);
        path.push_back(leaf);
      }
      break;
    } else {
      // vd+1 ← Expansion(vd)
      leaf = getBestChild(leaf);
    }
  }
  return leaf;
}

double MctsPlanner::defaultPolicy(std::shared_ptr<Primitive> node, const std::vector<std::shared_ptr<Primitive>> path) {
  /// TODO: link active mapping utility function here
  double planning_horizon{10.0};
  std::vector<std::shared_ptr<Primitive>> rollout_path;
  std::shared_ptr<Primitive> leaf = std::make_shared<Primitive>(*node);
  while (!is_terminal(leaf)) {
    rollout_path.push_back(leaf);
    std::vector<Eigen::Vector3d> rates;
    rates.push_back(maneuver_library_->getRandomPrimitiveRate());
    maneuver_library_->expandPrimitives(leaf, rates, planning_horizon);
    leaf = leaf->child_primitives[0];
  }

  TrajectorySegments candidate_trajectory;
  for (auto &primitive : path) {
    candidate_trajectory.appendSegment(primitive->segment);
  }
  for (auto &primitive : rollout_path) {
    candidate_trajectory.appendSegment(primitive->segment);
  }

  double utility{0.0};
  if (viewutility_map_) {
    std::vector<ViewPoint> view_point_list =
        ManeuverLibrary::sampleViewPointFromTrajectorySegment(candidate_trajectory);
    utility = viewutility_map_->CalculateViewUtility(view_point_list, false);
  } else {
    /// TODO: Remove this case and handle null utility maps differently
    // Calculate goal utility
    Eigen::Vector3d end_pos = leaf->segment.position().back();
    utility = 1 / ((goal_ - end_pos).norm() + 0.01);
  }

  return utility;
}

void MctsPlanner::backup(std::vector<std::shared_ptr<Primitive>> path, const double utility) {
  for (auto &node : path) {
    node->utility += utility;
    node->visits++;
  }
}

std::shared_ptr<Primitive> MctsPlanner::getBestChild(std::shared_ptr<Primitive> node) {
  int best_idx{0};
  int visits = node->visits;
  double best_ucb{-1};
  double c{0.0001};
  for (size_t i = 0; i < node->child_primitives.size(); i++) {
    std::shared_ptr<Primitive> child = node->child_primitives[i];
    if (child->visits < 1) continue;
    double upper_confidence_bound =
        child->utility / child->visits + c * std::sqrt(2.0 * std::log(visits) / child->visits);
    if (upper_confidence_bound > best_ucb) {
      best_idx = i;
      best_ucb = upper_confidence_bound;
    }
  }

  /// TODO: Safe guard against empty child
  return node->child_primitives[best_idx];
}

bool MctsPlanner::is_terminal(std::shared_ptr<Primitive> node) { return bool(node->depth >= max_tree_depth_); }

bool MctsPlanner::has_expandable_child(std::shared_ptr<Primitive> node) {
  if (!node->has_child()) {
    return true;
  } else {
    for (auto &child : node->child_primitives) {
      if (child->visits < 1) {
        return true;
      }
    }
  }
  return false;
}

std::shared_ptr<Primitive> MctsPlanner::getUnvisitedChild(std::shared_ptr<Primitive> node) {
  for (auto &child : node->child_primitives) {
    if (child->visits < 1) return child;
  }
  return nullptr;
}

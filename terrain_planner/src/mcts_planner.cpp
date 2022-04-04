#include "terrain_planner/mcts_planner.h"

TrajectorySegments MctsPlanner::solve(const Eigen::Vector3d current_pos, const Eigen::Vector3d current_vel,
                                      const Eigen::Vector4d current_att, TrajectorySegments &current_path) {
  std::vector<std::shared_ptr<Primitive>> path;
  Trajectory current_segment;
  State state_vector;
  state_vector.position = current_pos;
  state_vector.velocity = current_vel.normalized();
  state_vector.attitude = current_att;
  current_segment.states.push_back(state_vector);
  // root
  if (!tree_) tree_ = std::make_shared<Primitive>(current_segment);

  /// Traverse Tree
  auto leaf = treePolicy(tree_, path);
  assert(!leaf);
  /// Default policy
  double utility = defaultPolicy(leaf);

  /// Backup(vd+1, ∆)
  backup(path, utility);

  // Get Best Motion Primitives
  TrajectorySegments path_primitive;

  for (auto& node : path) {
    path_primitive.appendSegment(node->segment);
  }
  return path_primitive;
}

std::shared_ptr<Primitive> MctsPlanner::treePolicy(std::shared_ptr<Primitive> leaf,
                                                   std::vector<std::shared_ptr<Primitive>> &path) {
  double planning_horizon{10.0};
  while (true) {
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

double MctsPlanner::defaultPolicy(std::shared_ptr<Primitive> leaf) {
  /// TODO: link active mapping utility function here
  Eigen::Vector3d goal_pos{Eigen::Vector3d(200.0, 200.0, 0.0)};
  Eigen::Vector3d end_pos = leaf->segment.position().back();

  double utility = 1 / ((goal_pos - end_pos).norm() + 0.01);
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

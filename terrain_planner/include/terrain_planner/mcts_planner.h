#ifndef TERRAIN_PLANNER_MCTS_PLANNER_H
#define TERRAIN_PLANNER_MCTS_PLANNER_H

#include <adaptive_viewutility/viewutility_map.h>

#include "terrain_planner/maneuver_library.h"
#include "terrain_planner/planner.h"

/**
 * @brief MCTS Planner implementing the UCT algorithm
 *
 */
class MctsPlanner : public Planner {
 public:
  MctsPlanner() : Planner() { maneuver_library_ = std::make_shared<ManeuverLibrary>(); };
  virtual ~MctsPlanner(){};

  /**
   * @brief Solve MCTS with a given time budget
   *
   * @param current_pos
   * @param current_vel
   * @param current_att
   * @param current_path
   */
  virtual TrajectorySegments solve(const Eigen::Vector3d current_pos, const Eigen::Vector3d current_vel,
                                   const Eigen::Vector4d current_att, TrajectorySegments &current_path) override;

  /**
   * @brief Solve MCTS with a given time budget
   *
   * @param current_pos
   * @param current_vel
   * @param current_att
   * @param current_path
   */
  TrajectorySegments rollout(const Eigen::Vector3d current_pos, const Eigen::Vector3d current_vel,
                             const Eigen::Vector4d current_att);

  /**
   * @brief Tree policy of
   *
   * @return std::shared_ptr<Primitive>
   */
  std::shared_ptr<Primitive> treePolicy(std::shared_ptr<Primitive> leaf, std::vector<std::shared_ptr<Primitive>> &path);

  /**
   * @brief Backup along the path the utility and visit counts
   *
   * @param path path of the tree traversed in the current rollout
   * @param utility utility of the current rollout
   */
  static void backup(std::vector<std::shared_ptr<Primitive>> path, const double utility);

  /**
   * @brief
   *
   * @param leaf
   * @return double
   */
  double defaultPolicy(std::shared_ptr<Primitive> node, const std::vector<std::shared_ptr<Primitive>> path);

  /**
   * @brief Get the Motion Primitives object
   *
   * @return std::vector<TrajectorySegments> Motion primitive tree as a vector of trajectories
   */
  std::vector<TrajectorySegments> getMotionPrimitives() { return tree_->getMotionPrimitives(); };

  void setViewUtilityMap(std::shared_ptr<ViewUtilityMap> map) { viewutility_map_ = map; };

 private:
  /**
   * @brief Get the Best Child object
   *
   * @param node Primitive child
   * @return std::shared_ptr<Primitive> best child
   */
  std::shared_ptr<Primitive> getBestChild(std::shared_ptr<Primitive> node);

  /**
   * @brief Check if primitive node is in terminal state
   *
   * @param node
   * @return true Is terminal state (tree depth exceeds limit)
   * @return false Not a terminal state (tree depth exceeds limit)
   */
  bool is_terminal(std::shared_ptr<Primitive> node);

  /**
   * @brief Return true if the node is not fully expanded
   *
   * @param node Primitve object
   * @return true node is not fully expanded
   * @return false node is fully expanded
   */
  bool has_expandable_child(std::shared_ptr<Primitive> node);

  /**
   * @brief Get the Unvisited Child object
   *
   * @return std::shared_ptr<Primitive> get Unvisited child node. Returns nullptr if there are no unvisited child node
   */
  std::shared_ptr<Primitive> getUnvisitedChild(std::shared_ptr<Primitive> node);

  std::shared_ptr<Primitive> tree_;
  std::shared_ptr<ManeuverLibrary> maneuver_library_;
  std::shared_ptr<ViewUtilityMap> viewutility_map_;

  int max_tree_depth_{5};
};

#endif

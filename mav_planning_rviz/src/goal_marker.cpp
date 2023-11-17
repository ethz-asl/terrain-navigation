#include "mav_planning_rviz/goal_marker.h"
#include <functional>

using std::placeholders::_1;

GoalMarker::GoalMarker(rclcpp::Node::SharedPtr node)
  : node_(node), marker_server_("goal", node) {
  set_goal_marker_.header.frame_id = "map";
  set_goal_marker_.name = "set_pose";
  set_goal_marker_.scale = 100.0;
  set_goal_marker_.controls.clear();

  const double kSqrt2Over2 = sqrt(2.0) / 2.0;

  // Set up controls: x, y, z, and yaw.
  visualization_msgs::msg::InteractiveMarkerControl control;
  set_goal_marker_.controls.clear();
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.name = "move plane";
  set_goal_marker_.controls.push_back(control);

  marker_server_.insert(set_goal_marker_);
  marker_server_.setCallback(set_goal_marker_.name,
      std::bind(&GoalMarker::processSetPoseFeedback, this, _1));
  marker_server_.applyChanges();
  grid_map_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
      "/grid_map", 1,
      std::bind(&GoalMarker::GridmapCallback, this, _1));
}

GoalMarker::~GoalMarker() = default;

Eigen::Vector3d GoalMarker::getGoalPosition() {
  return goal_pos_;
};

Eigen::Vector3d GoalMarker::toEigen(const geometry_msgs::msg::Pose &p) {
  Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
  return position;
}

void GoalMarker::processSetPoseFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  // TODO: Set goal position from menu
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    set_goal_marker_.pose = feedback->pose;
    Eigen::Vector2d marker_position_2d(set_goal_marker_.pose.position.x, set_goal_marker_.pose.position.y);
    if (map_.isInside(marker_position_2d)) {
      double elevation = map_.atPosition("elevation", marker_position_2d);
      set_goal_marker_.pose.position.z = elevation + 200.0;
      marker_server_.setPose(set_goal_marker_.name, set_goal_marker_.pose);
      goal_pos_ = toEigen(feedback->pose);
      goal_pos_(2) = elevation + 100.0;
    }
  }
  marker_server_.applyChanges();
}

void GoalMarker::GridmapCallback(const grid_map_msgs::msg::GridMap &msg) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  grid_map::GridMapRosConverter::fromMessage(msg, map_);
  Eigen::Vector2d marker_position_2d(set_goal_marker_.pose.position.x, set_goal_marker_.pose.position.y);
  if (map_.isInside(marker_position_2d)) {
    // set_goal_marker_.pose.position.z
    double elevation = map_.atPosition("elevation", marker_position_2d);
    if (elevation + 200.0 > set_goal_marker_.pose.position.z) {
      set_goal_marker_.pose.position.z = elevation + 200.0;
      marker_server_.setPose(set_goal_marker_.name, set_goal_marker_.pose);
      goal_pos_(2) = elevation + 100.0;
    }
  }
  marker_server_.applyChanges();
}

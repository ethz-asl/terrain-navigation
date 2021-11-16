#include "mav_planning_rviz/goal_marker.h"

GoalMarker::GoalMarker(const ros::NodeHandle &nh) : nh_(nh), marker_server_("goal") {
  set_goal_marker_.header.frame_id = "map";
  set_goal_marker_.name = "set_pose";
  set_goal_marker_.scale = 100.0;
  set_goal_marker_.controls.clear();

  constexpr double kSqrt2Over2 = sqrt(2.0) / 2.0;

  // Set up controls: x, y, z, and yaw.
  visualization_msgs::InteractiveMarkerControl control;
  set_goal_marker_.controls.clear();
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.name = "move plane";
  set_goal_marker_.controls.push_back(control);

  marker_server_.insert(set_goal_marker_);
  marker_server_.setCallback(set_goal_marker_.name, boost::bind(&GoalMarker::processSetPoseFeedback, this, _1));
  marker_server_.applyChanges();
  grid_map_sub_ = nh_.subscribe("/grid_map", 1, &GoalMarker::GridmapCallback, this, ros::TransportHints().tcpNoDelay());
}

GoalMarker::~GoalMarker() {}

void GoalMarker::processSetPoseFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  // TODO: Set goal position from menu
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
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

void GoalMarker::GridmapCallback(const grid_map_msgs::GridMap &msg) {
  grid_map::GridMapRosConverter::fromMessage(msg, map_);
}

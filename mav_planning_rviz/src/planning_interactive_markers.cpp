#include "mav_planning_rviz/planning_interactive_markers.h"

namespace mav_planning_rviz {

PlanningInteractiveMarkers::PlanningInteractiveMarkers(const ros::NodeHandle& nh)
    : nh_(nh), marker_server_("planning_markers"), frame_id_("odom"), initialized_(false) {}

void PlanningInteractiveMarkers::setFrameId(const std::string& frame_id) {
  frame_id_ = frame_id;
  set_pose_marker_.header.frame_id = frame_id_;
  marker_prototype_.header.frame_id = frame_id_;
}

void PlanningInteractiveMarkers::initialize() {
  createMarkers();
  initialized_ = true;
}

void PlanningInteractiveMarkers::createMarkers() {
  constexpr double kSqrt2Over2 = sqrt(2.0) / 2.0;

  // Set up controls: x, y, z, and yaw.
  visualization_msgs::InteractiveMarkerControl control;
  set_pose_marker_.controls.clear();
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.name = "move plane";
  set_pose_marker_.controls.push_back(control);

  // Create a marker prototype, as the default style for all markers in the
  // marker map:
  marker_prototype_.header.frame_id = frame_id_;
  marker_prototype_.scale = 1.0;
  control.markers.clear();
  control.name = "arrow";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  visualization_msgs::Marker default_marker;
  default_marker.type = visualization_msgs::Marker::ARROW;
  default_marker.scale.x = 0.75;
  default_marker.scale.y = 0.25;
  default_marker.scale.z = 0.25;
  control.markers.push_back(default_marker);
  visualization_msgs::Marker text_marker;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.scale.z = 0.5;
  text_marker.pose.position.z = 0.5;
  text_marker.text = "placeholder";
  text_marker.id = 1;
  control.markers.push_back(text_marker);

  marker_prototype_.controls.push_back(control);
}

void PlanningInteractiveMarkers::enableSetPoseMarker(const mav_msgs::EigenTrajectoryPoint& pose) {
  geometry_msgs::PoseStamped pose_stamped;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(pose, &pose_stamped);
  set_pose_marker_.pose = pose_stamped.pose;

  marker_server_.insert(set_pose_marker_);
  marker_server_.setCallback(set_pose_marker_.name,
                             boost::bind(&PlanningInteractiveMarkers::processSetPoseFeedback, this, _1));
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::disableSetPoseMarker() {
  marker_server_.erase(set_pose_marker_.name);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::setPose(const mav_msgs::EigenTrajectoryPoint& pose) {
  geometry_msgs::PoseStamped pose_stamped;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(pose, &pose_stamped);
  set_pose_marker_.pose = pose_stamped.pose;
  marker_server_.setPose(set_pose_marker_.name, set_pose_marker_.pose);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::processSetPoseFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    if (pose_updated_function_) {
      mav_msgs::EigenTrajectoryPoint pose;
      mav_msgs::eigenTrajectoryPointFromPoseMsg(feedback->pose, &pose);
      pose_updated_function_(pose);
    }
  }

  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::enableMarker(const std::string& id, const mav_msgs::EigenTrajectoryPoint& pose) {
  geometry_msgs::PoseStamped pose_stamped;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(pose, &pose_stamped);

  auto search = marker_map_.find(id);
  if (search != marker_map_.end()) {
    // Already exists, just update the pose and enable it.
    search->second.pose = pose_stamped.pose;
    marker_server_.insert(search->second);
    marker_server_.applyChanges();
    return;
  }

  // Doesn't exist yet... Have to create it from prototype.
  marker_map_[id] = marker_prototype_;
  marker_map_[id].name = id;
  marker_map_[id].controls[0].markers[1].text = id;
  marker_map_[id].pose = pose_stamped.pose;
  marker_server_.insert(marker_map_[id]);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::updateMarkerPose(const std::string& id, const mav_msgs::EigenTrajectoryPoint& pose) {
  auto search = marker_map_.find(id);
  if (search == marker_map_.end()) {
    return;
  }

  geometry_msgs::PoseStamped pose_stamped;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(pose, &pose_stamped);
  search->second.pose = pose_stamped.pose;
  marker_server_.setPose(id, pose_stamped.pose);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::disableMarker(const std::string& id) {
  marker_server_.erase(id);
  marker_server_.applyChanges();
}

}  // end namespace mav_planning_rviz

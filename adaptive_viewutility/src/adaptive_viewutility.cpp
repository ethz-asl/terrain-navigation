/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Adaptive view utility estimation node
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */
#include "adaptive_viewutility/adaptive_viewutility.h"
#include "terrain_navigation/profiler.h"

#include <visualization_msgs/MarkerArray.h>

AdaptiveViewUtility::AdaptiveViewUtility(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &AdaptiveViewUtility::statusLoopCallback,
                                      this);  // Define timer for constant loop rate
  camera_path_pub_ = nh_.advertise<nav_msgs::Path>("camera_path", 1, true);
  candidate_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("candidate_path", 1, true);
  vehicle_path_pub_ = nh_.advertise<nav_msgs::Path>("vehicle_path", 1);
  camera_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("camera_poses", 1, true);
  camera_utility_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
  normal_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("surface_normal_marker", 1, true);
  viewpoint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);

  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  viewpoint_image_pub_ = nh_.advertise<sensor_msgs::Image>("image", 1, true);

  grid_map_ = grid_map::GridMap({"roi", "elevation", "elevation_normal_x", "elevation_normal_y", "elevation_normal_z",
                                 "visibility", "geometric_prior", "normalized_prior"});
  viewutility_map_ = std::make_shared<ViewUtilityMap>(grid_map_);
  viewplanner_ = std::make_shared<ViewPlanner>();
}

AdaptiveViewUtility::~AdaptiveViewUtility() {}

void AdaptiveViewUtility::Init() {
  double statusloop_dt_ = 1.0;
  ros::TimerOptions statuslooptimer_options(ros::Duration(statusloop_dt_),
                                            boost::bind(&AdaptiveViewUtility::statusLoopCallback, this, _1),
                                            &statusloop_queue_);

  statusloop_spinner_.reset(new ros::AsyncSpinner(1, &statusloop_queue_));
  statusloop_spinner_->start();
}

void AdaptiveViewUtility::statusLoopCallback(const ros::TimerEvent &event) {}

void AdaptiveViewUtility::MapPublishOnce() {
  viewutility_map_->getGridMap().setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(viewutility_map_->getGridMap(), message);
  grid_map_pub_.publish(message);
}

void AdaptiveViewUtility::ViewpointPublishOnce() {
  std::vector<geometry_msgs::PoseStamped> posestampedhistory_vector;
  std::vector<geometry_msgs::Pose> posehistory_vector;
  std::vector<visualization_msgs::Marker> markerhistory_vector;
  std::vector<visualization_msgs::Marker> viewpoint_vector;

  int i = 0;
  for (auto viewpoint : viewpoint_) {
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(),
                                     vector3d2PoseStampedMsg(viewpoint.getCenterLocal(), viewpoint.getOrientation()));
    posehistory_vector.insert(posehistory_vector.begin(),
                              vector3d2PoseMsg(viewpoint.getCenterLocal(), viewpoint.getOrientation()));
    viewpoint_vector.insert(viewpoint_vector.begin(), Viewpoint2MarkerMsg(i, viewpoint));
    markerhistory_vector.insert(markerhistory_vector.begin(),
                                utility2MarkerMsg(viewpoint.getUtility(), viewpoint.getCenterLocal(), i));
    i++;
  }

  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posestampedhistory_vector;

  camera_path_pub_.publish(msg);

  geometry_msgs::PoseArray pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "map";
  pose_msg.poses = posehistory_vector;

  camera_pose_pub_.publish(pose_msg);

  visualization_msgs::MarkerArray viewpoint_marker_msg;
  viewpoint_marker_msg.markers = viewpoint_vector;
  viewpoint_pub_.publish(viewpoint_marker_msg);

  visualization_msgs::MarkerArray marker_msg;
  marker_msg.markers = markerhistory_vector;
  camera_utility_pub_.publish(marker_msg);
}

void AdaptiveViewUtility::NormalPublishOnce() {
  std::vector<visualization_msgs::Marker> normals_vector;

  int i = 0;
  grid_map::GridMap gridmap = viewutility_map_->getGridMap();

  for (grid_map::GridMapIterator iterator(gridmap); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    const grid_map::Index index(*iterator);

    Eigen::Vector3d cell_pos;
    gridmap.getPosition3("elevation", index, cell_pos);
    Eigen::Vector3d normal;
    normal << gridmap.at("elevation_normal_x", index), gridmap.at("elevation_normal_y", index),
        gridmap.at("elevation_normal_z", index);

    normals_vector.insert(normals_vector.begin(), normals2ArrowsMsg(cell_pos, normal, i));
    i++;
  }

  visualization_msgs::MarkerArray marker_msg;
  marker_msg.markers = normals_vector;
  normal_marker_pub_.publish(marker_msg);
}

bool AdaptiveViewUtility::AddViewPointFromImage(std::string &image_path) {
  GDALDataset *poSrcDS = (GDALDataset *)GDALOpen(image_path.c_str(), GA_ReadOnly);

  if (!poSrcDS) {
    return false;
  }

  std::string exif_gps_altitude = poSrcDS->GetMetadataItem("EXIF_GPSAltitude");
  std::string exif_gps_latitude = poSrcDS->GetMetadataItem("EXIF_GPSLatitude");
  std::string exif_gps_longitude = poSrcDS->GetMetadataItem("EXIF_GPSLongitude");
  std::string exif_gps_track = poSrcDS->GetMetadataItem("EXIF_GPSTrack");

  double viewpoint_altitude = StringToGeoReference(exif_gps_altitude);
  double viewpoint_latitude = StringToGeoReference(exif_gps_latitude);
  double viewpoint_longitude = StringToGeoReference(exif_gps_longitude);

  double time_seconds = GetTimeInSeconds(std::string(poSrcDS->GetMetadataItem("EXIF_DateTime")));
  GDALClose((GDALDatasetH)poSrcDS);

  int view_index = viewpoint_.size();

  this->AddViewPoint(view_index, viewpoint_longitude, viewpoint_latitude, viewpoint_altitude);

  viewpoint_.back().setTime(time_seconds);

  viewpoint_.back().setImage(image_path);

  return true;
}

void AdaptiveViewUtility::AddViewPoint(const int idx, const double &longitude, const double &latitude,
                                       const double &altitude) {
  ViewPoint viewpoint(idx, longitude, latitude, altitude);
  viewpoint_.push_back(viewpoint);
}

void AdaptiveViewUtility::LoadMap(const std::string &path) {
  // Default to empty map when no path is specified
  if (path == "") {
    viewutility_map_->initializeEmptyMap();
    return;
  }

  std::string file_extension = path.substr(path.find_last_of(".") + 1);

  // Check if the file exists on the path
  std::ifstream f(path.c_str());
  if (!f.good()) {
    throw std::invalid_argument("Cannot find file: " + path);
  }

  if (file_extension == "obj") {
    viewutility_map_->initializeFromMesh(path, target_map_resolution_);
  } else if (file_extension == "tif") {
    GDALAllRegister();
    GDALDataset *dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_ReadOnly);
    if (!dataset) {
      std::cout << "Failed to open" << std::endl;
      return;
    }

    viewutility_map_->initializeFromGeotiff(dataset);
  } else {
    throw std::runtime_error("Unknown file type: " + file_extension);
  }
}

void AdaptiveViewUtility::UpdateUtility(Trajectory &trajectory) {
  for (int i = 0; i < trajectory.states.size(); i++) {
    Eigen::Vector3d pos = trajectory.states[i].position;
    Eigen::Vector3d vel = trajectory.states[i].velocity;
    Eigen::Vector4d att = trajectory.states[i].attitude;
    int view_index = viewpoint_.size();
    ViewPoint viewpoint(view_index, pos, att);
    // viewpoint.setOrientation(att);

    UpdateUtility(viewpoint);

    viewpoint_.push_back(viewpoint);
  }

  for (int i = 0; i < trajectory.states.size(); i++) {
    vehicle_position_history_.push_back(trajectory.states[i].position);
    vehicle_orientation_history_.push_back(trajectory.states[i].attitude);
  }
}

void AdaptiveViewUtility::UpdateUtility(ViewPoint &viewpoint) { viewutility_map_->UpdateUtility(viewpoint); }

void AdaptiveViewUtility::publishViewpointHistory() {
  nav_msgs::Path msg;
  std::vector<geometry_msgs::PoseStamped> posestampedhistory_vector;
  Eigen::Vector4d orientation(1.0, 0.0, 0.0, 0.0);
  for (auto pos : vehicle_position_history_) {
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(), vector3d2PoseStampedMsg(pos, orientation));
  }
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posestampedhistory_vector;
  vehicle_path_pub_.publish(msg);
}

void AdaptiveViewUtility::PublishViewpointImage(ViewPoint &viewpoint) {
  cv_bridge::CvImage msg;
  msg.header.stamp = ros::Time::now();
  msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  msg.image = viewpoint.getImage();
  sensor_msgs::Image image_msg = *msg.toImageMsg();
  viewpoint_image_pub_.publish(image_msg);
}

void AdaptiveViewUtility::Visualize() {
  PublishViewpointImage(viewpoint_.back());
  ViewpointPublishOnce();
  MapPublishOnce();
}

bool AdaptiveViewUtility::generateMotionPrimitives() {
  std::vector<Trajectory> motion_primitives =
      viewplanner_->generateMotionPrimitives(vehicle_position_, vehicle_velocity_);

  publishCandidatePaths(motion_primitives);

  return true;
}

Trajectory AdaptiveViewUtility::getBestPrimitive() {
  Trajectory trajectory = viewplanner_->getBestPrimitive();
  // Trajectory trajectory = viewplanner_->getRandomPrimitive();

  return trajectory;
}

void AdaptiveViewUtility::publishCandidatePaths(std::vector<Trajectory> &motion_primitives) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  candidate_path_pub_.publish(msg);

  std::vector<visualization_msgs::Marker> maneuver_library_vector;
  int i = 0;
  for (auto maneuver : motion_primitives) {
    maneuver_library_vector.insert(maneuver_library_vector.begin(), trajectory2MarkerMsg(maneuver, i));
    i++;
  }
  msg.markers = maneuver_library_vector;
  candidate_path_pub_.publish(msg);
}

void AdaptiveViewUtility::estimateViewUtility() {
  std::vector<Trajectory> &motion_primitives = viewplanner_->getMotionPrimitives();
  for (auto &primitive : motion_primitives) {
    std::vector<ViewPoint> primitive_viewpoint;
    for (int i = 0; i < primitive.states.size(); i++) {
      Eigen::Vector3d pos = primitive.states[i].position;
      Eigen::Vector3d vel = primitive.states[i].velocity;
      Eigen::Vector4d att = primitive.states[i].attitude;
      int view_index = viewpoint_.size();
      ViewPoint viewpoint(view_index, pos, att);
      primitive_viewpoint.push_back(viewpoint);
    }
    primitive.utility = viewutility_map_->CalculateViewUtility(primitive_viewpoint, false);
    primitive_viewpoint.clear();
  }
}

void AdaptiveViewUtility::OutputMapData(const std::string &path) { viewutility_map_->OutputMapData(path); }

void AdaptiveViewUtility::InitializeVehicleFromMap(Eigen::Vector3d &init_pos, Eigen::Vector3d &init_vel) {
  double max_altitude = 150.0;
  grid_map::Index idx(static_cast<int>(getRandom(0, viewutility_map_->getGridMap().getSize()(0))),
                      static_cast<int>(getRandom(0, viewutility_map_->getGridMap().getSize()(1))));
  Eigen::Vector2d init_pos_xy;
  viewutility_map_->getGridMap().getPosition(idx, init_pos_xy);
  double altitude = viewutility_map_->getGridMap().at("elevation", idx);
  if (!std::isfinite(altitude)) {
    altitude = 0.0;
  }
  double init_z = altitude + getRandom(0, max_altitude);
  double init_theta = getRandom(-M_PI, M_PI);
  init_pos = Eigen::Vector3d(init_pos_xy(0), init_pos_xy(1), init_z);
  init_vel = 15.0 * Eigen::Vector3d(std::cos(init_theta), std::sin(init_theta), 0.0);

  return;
}

visualization_msgs::Marker AdaptiveViewUtility::trajectory2MarkerMsg(Trajectory &trajectory, const int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "normals";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  std::vector<geometry_msgs::Point> points;
  for (auto state : trajectory.states) {
    Eigen::Vector3d position = state.position;
    geometry_msgs::Point point;
    point.x = position(0);
    point.y = position(1);
    point.z = position(2);
    points.push_back(point);
  }
  marker.points = points;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 0.2;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  return marker;
}

visualization_msgs::Marker AdaptiveViewUtility::Viewpoint2MarkerMsg(int id, ViewPoint &viewpoint) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  const Eigen::Vector3d position = viewpoint.getCenterLocal();
  const Eigen::Matrix3d rotation = ViewPoint::quat2RotMatrix(viewpoint.getOrientation());
  std::vector<geometry_msgs::Point> points;
  Eigen::Vector3d view_center = position;
  std::vector<Eigen::Vector3d> vertex;
  vertex.push_back(position + rotation * Eigen::Vector3d(5.0, 5.0, -5.0));
  vertex.push_back(position + rotation * Eigen::Vector3d(5.0, -5.0, -5.0));
  vertex.push_back(position + rotation * Eigen::Vector3d(-5.0, -5.0, -5.0));
  vertex.push_back(position + rotation * Eigen::Vector3d(-5.0, 5.0, -5.0));

  for (size_t i = 0; i < vertex.size(); i++) {
    points.push_back(toPoint(position));  // Viewpoint center
    points.push_back(toPoint(vertex[i]));
    points.push_back(toPoint(vertex[i]));
    points.push_back(toPoint(vertex[(i + 1) % vertex.size()]));
  }

  marker.points = points;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  return marker;
}

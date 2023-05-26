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
 * 3. Neither the name terrain-navigation nor the names of its contributors may be
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

#ifndef COMMON_H
#define COMMON_H

#include "terrain_navigation/path.h"

#include <Eigen/Dense>
#include <fstream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
  Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
  return ev3;
}

Eigen::Vector3d toEigen(const geometry_msgs::Pose &p) {
  Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
  return position;
}

geometry_msgs::Vector3 toVector3(const Eigen::Vector3d &p) {
  geometry_msgs::Vector3 vector;
  vector.x = p(0);
  vector.y = p(1);
  vector.z = p(2);
  return vector;
}

geometry_msgs::Pose vector3d2PoseMsg(const Eigen::Vector3d position, const Eigen::Vector4d orientation) {
  geometry_msgs::Pose encode_msg;

  encode_msg.orientation.w = orientation(0);
  encode_msg.orientation.x = orientation(1);
  encode_msg.orientation.y = orientation(2);
  encode_msg.orientation.z = orientation(3);
  encode_msg.position.x = position(0);
  encode_msg.position.y = position(1);
  encode_msg.position.z = position(2);
  return encode_msg;
}

geometry_msgs::PoseStamped vector3d2PoseStampedMsg(const Eigen::Vector3d position, const Eigen::Vector4d orientation) {
  geometry_msgs::PoseStamped encode_msg;

  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

visualization_msgs::Marker utility2MarkerMsg(const double utility, const Eigen::Vector3d position, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  double vis_utility;
  if (utility < 1.0) {
    vis_utility = 1.0 / 1;
  } else {
    vis_utility = utility / 1;
  }
  marker.scale.x = vis_utility;
  marker.scale.y = vis_utility;
  marker.scale.z = vis_utility;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  return marker;
}

visualization_msgs::Marker vector2ArrowsMsg(const Eigen::Vector3d &position, const Eigen::Vector3d &normal, int id,
                                            Eigen::Vector3d color = Eigen::Vector3d(0.0, 1.0, 0.0),
                                            const std::string marker_namespace = "normals") {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point head;
  head.x = position(0);
  head.y = position(1);
  head.z = position(2);
  points.push_back(head);
  geometry_msgs::Point tail;
  tail.x = position(0) + normal(0);
  tail.y = position(1) + normal(1);
  tail.z = position(2) + normal(2);
  points.push_back(tail);

  marker.points = points;
  marker.scale.x = std::min(normal.norm(), 1.0);
  marker.scale.y = std::min(normal.norm(), 2.0);
  marker.scale.z = 0.0;
  marker.color.a = 1.0;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  return marker;
}

visualization_msgs::Marker trajectory2MarkerMsg(Path &trajectory, const int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "normals";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  std::vector<geometry_msgs::Point> points;
  for (auto position : trajectory.position()) {
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
  if (!trajectory.valid()) {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  } else {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  return marker;
}

visualization_msgs::Marker trajectory2MarkerMsg(PathSegment &trajectory, const int id,
                                                Eigen::Vector3d color = Eigen::Vector3d(0.0, 1.0, 0.0)) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "normals";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  std::vector<geometry_msgs::Point> points;
  for (auto &position : trajectory.position()) {
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
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();

  return marker;
}

visualization_msgs::Marker point2MarkerMsg(Eigen::Vector3d &position, const int id,
                                           Eigen::Vector3d color = Eigen::Vector3d(0.0, 1.0, 0.0)) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "normals";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point point;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 2.0;
  marker.scale.y = 2.0;
  marker.scale.z = 2.0;
  marker.color.a = 0.5;
  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();

  return marker;
}

double GetTimeInSeconds(std::string date_time) {
  std::stringstream ss(date_time);
  std::string tagged_time;
  std::string tagged_date;

  ss >> tagged_date >> tagged_time;

  std::stringstream ss_time(tagged_time);
  std::vector<std::string> time_hour;

  while (ss_time.good()) {
    std::string substr;
    std::getline(ss_time, substr, ':');
    time_hour.push_back(substr);
  }

  return 3600.0 * std::stof(time_hour[0]) + 60.0 * std::stof(time_hour[1]) + std::stoi(time_hour[2]);
}

double StringToGeoReference(std::string &exif_tag) {
  std::stringstream ss(exif_tag);
  std::vector<std::string> result;
  while (ss.good()) {
    std::string substr;
    std::getline(ss, substr, '(');
    std::getline(ss, substr, ')');
    result.push_back(substr);
  }
  double output;
  if (result.size() >= 4) {
    /// TODO: Check precision and coordinate frame of this conversion
    output = std::stod(result[0]) + 0.0166667 * std::stod(result[1]) + 0.000277778 * std::stod(result[2]);
  } else {
    output = std::stod(result[0]);
  }
  return output;
}

bool parseAttitudeFromText(std::string text_path, std::string image_file, Eigen::Vector4d &attitude) {
  std::ifstream file(text_path);
  std::string str;

  // Look for the image file name in the path
  while (getline(file, str)) {
    if (str.find(image_file) != std::string::npos) {
      std::stringstream ss(str);
      std::vector<std::string> camera_pose;
      camera_pose.resize(9);
      // #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
      ss >> camera_pose[0] >> camera_pose[1] >> camera_pose[2] >> camera_pose[3] >> camera_pose[4];
      attitude << std::stof(camera_pose[1]), std::stof(camera_pose[2]), std::stof(camera_pose[3]),
          std::stof(camera_pose[4]);
      return true;
    }
  }
  return false;
}

double getRandom(double min, double max) {
  return std::abs(max - min) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX) + std::min(max, min);
}

Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

bool validatePosition(const grid_map::GridMap &map, const Eigen::Vector3d goal, Eigen::Vector3d &valid_goal) {
  double upper_surface = map.atPosition("ics_+", goal.head(2));
  double lower_surface = map.atPosition("ics_-", goal.head(2));
  const bool is_goal_valid = (upper_surface < lower_surface) ? true : false;
  valid_goal(0) = goal(0);
  valid_goal(1) = goal(1);
  valid_goal(2) = (upper_surface + lower_surface) / 2.0;
  return is_goal_valid;
}

#endif

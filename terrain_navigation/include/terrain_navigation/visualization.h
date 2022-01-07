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
 * @brief Helper functions for visualizations
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "geometry_msgs/Point.h"
#include "terrain_navigation/viewpoint.h"
#include "visualization_msgs/Marker.h"

geometry_msgs::Point toPoint(const Eigen::Vector3d &p) {
  geometry_msgs::Point position;
  position.x = p(0);
  position.y = p(1);
  position.z = p(2);
  return position;
}

visualization_msgs::Marker Viewpoint2MarkerMsg(int id, ViewPoint &viewpoint) {
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

#endif  // VISUALIZATION_H

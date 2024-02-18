/****************************************************************************
 *
 *   Copyright (c) 2023 Jaeyoung Lim. All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

#ifndef TERRAIN_NAVIGATION_ROS_VISUALIZATION_H
#define TERRAIN_NAVIGATION_ROS_VISUALIZATION_H

#include <terrain_navigation/viewpoint.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &p);

void publishVehiclePose(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                        const Eigen::Vector3d &position, const Eigen::Vector4d &attitude,
                        std::string mesh_resource_path);

visualization_msgs::msg::Marker Viewpoint2MarkerMsg(int id, ViewPoint &viewpoint,
                                                    Eigen::Vector3d color = Eigen::Vector3d(0.0, 0.0, 1.0));

void publishCameraView(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                       const Eigen::Vector3d &position, const Eigen::Vector4d &attitude);

void publishViewpoints(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                       std::vector<ViewPoint> &viewpoint_vector,
                       Eigen::Vector3d color = Eigen::Vector3d(0.0, 0.0, 1.0));

#endif

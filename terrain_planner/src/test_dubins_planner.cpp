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
/**
 * @brief ROS Node to test dubins planner
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <ros/ros.h>
#include "terrain_planner/common.h"
#include "terrain_planner/dubins_planner.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

void publishPositionSetpoints(ros::Publisher &pub, const Eigen::Vector3d &position, const Eigen::Vector3d &velocity) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  pub.publish(marker);

  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 10.0;
  marker.scale.y = 2.0;
  marker.scale.z = 2.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  double yaw = std::atan2(velocity(1), velocity(0));
  marker.pose.orientation.w = std::cos(0.5 * yaw);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(0.5 * yaw);

  pub.publish(marker);
}

void publishTrajectory(ros::Publisher &pub, std::vector<Eigen::Vector3d> trajectory) {
  nav_msgs::Path msg;
  std::vector<geometry_msgs::PoseStamped> posestampedhistory_vector;
  Eigen::Vector4d orientation(1.0, 0.0, 0.0, 0.0);
  for (auto pos : trajectory) {
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(), vector3d2PoseStampedMsg(pos, orientation));
  }
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posestampedhistory_vector;
  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dubins_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher start_pos_pub = nh.advertise<visualization_msgs::Marker>("start_position", 1, true);
  ros::Publisher goal_pos_pub = nh.advertise<visualization_msgs::Marker>("goal_position", 1, true);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  std::shared_ptr<DubinsPlanner> planner_ = std::make_shared<DubinsPlanner>();

  Eigen::Vector3d start_pos{Eigen::Vector3d(0.0, 0.0, 0.0)};
  double start_heading{M_PI_2};
  Eigen::Vector3d goal_pos{Eigen::Vector3d(10.0, 0.0, 0.0)};
  double goal_heading{-M_PI_2};

  while (true) {
    start_heading += 0.02;
    goal_heading -= 0.04;
    Eigen::Vector3d start_vel{Eigen::Vector3d(std::cos(start_heading), std::sin(start_heading), 0.0)};
    Eigen::Vector3d goal_vel{Eigen::Vector3d(std::cos(goal_heading), std::sin(goal_heading), 0.0)};

    planner_->setStartPosition(start_pos, start_heading);
    planner_->setGoalPosition(goal_pos, goal_heading);

    TrajectorySegments shortest_path = planner_->Solve();
    publishPositionSetpoints(start_pos_pub, start_pos, start_vel);
    publishPositionSetpoints(goal_pos_pub, goal_pos, goal_vel);
    publishTrajectory(path_pub, shortest_path.position());
    ros::Duration(0.1).sleep();
  }

  ros::spin();
  return 0;
}

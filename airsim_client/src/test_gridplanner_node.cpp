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
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "adaptive_viewutility/adaptive_viewutility.h"
#include "adaptive_viewutility/dubins_planner.h"
#include "adaptive_viewutility/performance_tracker.h"
#include "airsim_client/airsim_client.h"
#include "terrain_navigation/profiler.h"

Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw) {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Vector4d q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;

  q.normalize();

  return q;
}

double getDubinsMetric(std::shared_ptr<DubinsPlanner> &planner, Eigen::Vector3d start_position,
                       Eigen::Vector2d start_vel, Eigen::Vector3d goal_position, Eigen::Vector2d goal_vel,
                       TrajectorySegments &shortest_path) {
  /// TODO: Use dubins distance for calculating turining time
  double start_heading = std::atan2(start_vel(1), start_vel(0));
  planner->setStartPosition(start_position, start_heading);
  double goal_heading = std::atan2(goal_vel(1), goal_vel(0));
  planner->setGoalPosition(goal_position, goal_heading);
  double shortest_distance = planner->Solve(shortest_path);
  double euclidean_distance = (start_position - goal_position).norm();
  double time = shortest_distance / 15.0;  // 15.0 m/s is the cruise speed
  return time;
}

visualization_msgs::Marker trajectory2MarkerMsg(TrajectorySegments &trajectory, const int id) {
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
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  return marker;
}

void publishTrajectorySegments(ros::Publisher &pub, TrajectorySegments &trajectory) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub.publish(msg);

  std::vector<visualization_msgs::Marker> maneuver_library_vector;
  int i = 0;
  bool visualize_invalid_trajectories = false;
  maneuver_library_vector.push_back(trajectory2MarkerMsg(trajectory, 0));
  msg.markers = maneuver_library_vector;
  pub.publish(msg);
}

void addViewpoint(Trajectory &trajectory, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector4d att) {
  State vehicle_state;
  vehicle_state.position = pos;
  vehicle_state.velocity = vel * 15.0;
  vehicle_state.attitude = att;
  trajectory.states.push_back(vehicle_state);
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher trajectory_pub_ = nh.advertise<visualization_msgs::MarkerArray>("path", 1, true);

  std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);
  std::shared_ptr<AirsimClient> airsim_client = std::make_shared<AirsimClient>();
  std::shared_ptr<DubinsPlanner> dubins_planner = std::make_shared<DubinsPlanner>();

  std::string file_path, output_file_path;
  std::string image_directory{""};
  double max_experiment_duration;
  double snapshot_interval;  // Interval (seconds) that the package will keep track of
  nh_private.param<std::string>("file_path", file_path, "resources/cadastre.tif");
  nh_private.param<double>("max_experiment_duration", max_experiment_duration, 500);
  nh_private.param<std::string>("output_file_path", output_file_path, "output/benchmark.csv");
  nh_private.param<std::string>("image_directory", image_directory, image_directory);
  nh_private.param<double>("snapshot_interval", snapshot_interval, 20.0);

  airsim_client->setImageDirectory(image_directory);

  Eigen::Vector3d airsim_player_start = airsim_client->getPlayerStart();

  // Add elevation map from GeoTIFF file defined in path
  adaptive_viewutility->LoadMap(file_path);
  adaptive_viewutility->getViewUtilityMap()->TransformMap(airsim_player_start);

  ros::Time start_time_ = ros::Time::now();
  double start_time_seconds_ = 0.0;

  /// set Current state of vehicle

  Profiler pipeline_perf("Planner Loop");
  pipeline_perf.tic();

  grid_map::Polygon polygon;

  // Hah! we have access to the map reference. be careful!
  grid_map::GridMap &map = adaptive_viewutility->getViewUtilityMap()->getGridMap();
  const Eigen::Vector2d map_pos = map.getPosition();
  const double map_width_x = map.getLength().x();
  const double map_width_y = map.getLength().y();

  polygon.setFrameId(map.getFrameId());
  double roi_portion = 0.5;
  polygon.addVertex(grid_map::Position(map_pos(0) - roi_portion * map_width_x, map_pos(1) - roi_portion * map_width_y));
  polygon.addVertex(grid_map::Position(map_pos(0) + roi_portion * map_width_x, map_pos(1) - roi_portion * map_width_y));
  polygon.addVertex(grid_map::Position(map_pos(0) + roi_portion * map_width_x, map_pos(1) + roi_portion * map_width_y));
  polygon.addVertex(grid_map::Position(map_pos(0) - roi_portion * map_width_x, map_pos(1) + roi_portion * map_width_y));

  adaptive_viewutility->getViewUtilityMap()->SetRegionOfInterest(polygon);

  // From "Huang, Wesley H. "Optimal line-sweep-based decompositions for coverage algorithms." Proceedings 2001 ICRA.
  // IEEE International Conference on Robotics and Automation (Cat. No. 01CH37164). Vol. 1. IEEE, 2001."
  // The optimal sweep pattern direction coinsides with the edge direction of the ROI polygon

  /// TODO: Get sweep distance from sensor model
  grid_map::Polygon offset_polygon = polygon;
  offset_polygon.offsetInward(1.0);

  Eigen::Vector2d start_pos_2d = offset_polygon.getVertex(0);

  // Run sweep segments until the boundary
  double view_distance = 90.0;
  double altitude = 100.0;
  Eigen::Vector2d sweep_direction = (polygon.getVertex(1) - polygon.getVertex(0)).normalized();
  Eigen::Vector2d sweep_perpendicular = (polygon.getVertex(3) - polygon.getVertex(0)).normalized();

  Eigen::Vector2d vehicle_pos_2d = start_pos_2d;
  Eigen::Vector2d vehicle_vel_2d = 15.0 * Eigen::Vector2d(sweep_direction(0), sweep_direction(1));
  int max_viewpoints = 1000;  // Terminate when the number of view points exceed the maximum

  // Generate a lawnmower pattern survey

  pipeline_perf.toc();

  std::shared_ptr<PerformanceTracker> performance_tracker = std::make_shared<PerformanceTracker>();

  double elevation = map.atPosition("elevation", vehicle_pos_2d);
  bool terrain_altitude = true;

  TrajectorySegments vehicle_trajectory;

  bool terminate_mapping = false;
  double planning_horizon = 2.0;
  double simulated_time{0.0};
  int snapshot_index{0};
  int num_images{0};
  double elapsed_time{0.0};
  while (true) {
    // Terminate if simulation time has exceeded
    if (simulated_time > max_experiment_duration) break;
    Trajectory reference_trajectory;
    if (terrain_altitude) elevation = map.atPosition("elevation", vehicle_pos_2d);
    Eigen::Vector3d vehicle_pos(vehicle_pos_2d(0), vehicle_pos_2d(1), elevation + altitude);
    Eigen::Vector3d vehicle_vel(vehicle_vel_2d(0), vehicle_vel_2d(1), 0.0);
    Eigen::Vector4d vehicle_att(1.0, 0.0, 0.0, 0.0);
    // vehicle_pos = vehicle_pos + origin;
    adaptive_viewutility->setCurrentState(vehicle_pos, vehicle_vel);
    addViewpoint(reference_trajectory, vehicle_pos, vehicle_vel, vehicle_att);
    // Acquire images along the motion primitive from airsim
    airsim_client->setPose(vehicle_pos, vehicle_att);
    num_images++;
    adaptive_viewutility->UpdateUtility(reference_trajectory);

    // Visualize results
    adaptive_viewutility->MapPublishOnce();
    adaptive_viewutility->ViewpointPublishOnce();
    adaptive_viewutility->publishViewpointHistory();
    publishTrajectorySegments(trajectory_pub_, vehicle_trajectory);

    /// TODO: Publish vehcile trajectory
    vehicle_vel_2d = 15.0 * sweep_direction;

    // Generate sweep pattern
    double dt = 4.0;  // Trigger time interval
    Eigen::Vector2d candidate_vehicle_pos_2d = vehicle_vel_2d * dt + vehicle_pos_2d;
    TrajectorySegments shortest_path;
    if (polygon.isInside(candidate_vehicle_pos_2d)) {
      double candidate_elevation = altitude + map.atPosition("elevation", candidate_vehicle_pos_2d);
      /// TODO: Constrain climbrate
      Eigen::Vector3d candidate_vehicle_pos(candidate_vehicle_pos_2d(0), candidate_vehicle_pos_2d(1),
                                            candidate_elevation);
      double dubins_metric = getDubinsMetric(dubins_planner, vehicle_pos, vehicle_vel_2d, candidate_vehicle_pos,
                                             vehicle_vel_2d, shortest_path);
      simulated_time += dubins_metric;
      elapsed_time += dubins_metric;
      vehicle_pos_2d = candidate_vehicle_pos_2d;
    } else {  // Reached the other end of the vertex
      sweep_direction = -1.0 * sweep_direction;
      Eigen::Vector2d candidate_vehicle_vel_2d = 15.0 * sweep_direction;
      candidate_vehicle_pos_2d = vehicle_pos_2d + sweep_perpendicular * view_distance;
      if (!polygon.isInside(candidate_vehicle_pos_2d)) break;
      double candidate_elevation = altitude + map.atPosition("elevation", candidate_vehicle_pos_2d);
      Eigen::Vector3d candidate_vehicle_pos(candidate_vehicle_pos_2d(0), candidate_vehicle_pos_2d(1),
                                            candidate_elevation);
      double dubins_metric = getDubinsMetric(dubins_planner, vehicle_pos, vehicle_vel_2d, candidate_vehicle_pos,
                                             candidate_vehicle_vel_2d, shortest_path);
      simulated_time += dubins_metric;
      elapsed_time += dubins_metric;
      vehicle_pos_2d = vehicle_pos_2d + sweep_perpendicular * view_distance;  // next row
      if (!polygon.isInside(vehicle_pos_2d)) {
        break;
      } else {
        std::cout << "is inside!" << std::endl;
      }
    }
    for (auto &segment : shortest_path.segments) {
      vehicle_trajectory.appendSegment(segment);
    }
    if (elapsed_time > snapshot_interval) {
      std::cout << "Taking snapshot:" << simulated_time << std::endl;
      std::cout << "  - Number of images: " << num_images << std::endl;
      std::cout << "  - Index: " << snapshot_index << std::endl;
      performance_tracker->Record(simulated_time, adaptive_viewutility->getViewUtilityMap()->getGridMap());
      std::string saved_map_path =
          image_directory + "/gridmap_" + std::to_string(static_cast<int>(snapshot_index)) + ".bag";
      grid_map::GridMapRosConverter::saveToBag(adaptive_viewutility->getViewUtilityMap()->getGridMap(), saved_map_path,
                                               "/grid_map");
      elapsed_time = 0.0;
      snapshot_index++;
    }
    ros::Duration(0.5).sleep();
  }
  performance_tracker->Output(output_file_path);
  std::cout << "[TestGridNode] Grid Planner Terminated" << std::endl;

  ros::spin();
  return 0;
}

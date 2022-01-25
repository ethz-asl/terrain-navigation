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

  std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);
  std::shared_ptr<AirsimClient> airsim_client = std::make_shared<AirsimClient>();

  std::string file_path, output_file_path;
  std::string image_directory{""};
  double max_experiment_duration;
  int snapshot_increment;
  nh_private.param<std::string>("file_path", file_path, "resources/cadastre.tif");
  nh_private.param<double>("max_experiment_duration", max_experiment_duration, 500);
  nh_private.param<std::string>("output_file_path", output_file_path, "output/benchmark.csv");
  nh_private.param<std::string>("image_directory", image_directory, image_directory);
  nh_private.param<int>("snapshot_increment", snapshot_increment, 10);

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
  double roi_portion = 0.45;
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

  double dt = 4.0;
  Eigen::Vector2d vehicle_pos_2d = start_pos_2d;
  Eigen::Vector2d vehicle_vel_2d = 15.0 * Eigen::Vector2d(sweep_direction(0), sweep_direction(1));
  int max_viewpoints = 1000;  // Terminate when the number of view points exceed the maximum

  // Generate a lawnmower pattern survey

  pipeline_perf.toc();

  std::shared_ptr<PerformanceTracker> performance_tracker = std::make_shared<PerformanceTracker>();

  bool terminate_mapping = false;
  double planning_horizon = 2.0;
  double turning_time = 10.0;
  double simulated_time{0.0};
  int increment{1};

  while (true) {
    Trajectory reference_trajectory;

    double elevation = map.atPosition("elevation", vehicle_pos_2d);
    Eigen::Vector3d vehicle_pos(vehicle_pos_2d(0), vehicle_pos_2d(1), elevation + altitude);
    Eigen::Vector3d vehicle_vel(vehicle_vel_2d(0), vehicle_vel_2d(1), 0.0);
    Eigen::Vector4d vehicle_att(1.0, 0.0, 0.0, 0.0);
    // vehicle_pos = vehicle_pos + origin;
    adaptive_viewutility->setCurrentState(vehicle_pos, vehicle_vel);
    addViewpoint(reference_trajectory, vehicle_pos, vehicle_vel, vehicle_att);
    // Acquire images along the motion primitive from airsim
    airsim_client->setPose(vehicle_pos, vehicle_att);

    adaptive_viewutility->UpdateUtility(reference_trajectory);

    // Visualize results
    adaptive_viewutility->MapPublishOnce();
    adaptive_viewutility->ViewpointPublishOnce();
    adaptive_viewutility->publishViewpointHistory();

    performance_tracker->Record(simulated_time, adaptive_viewutility->getViewUtilityMap()->getGridMap());

    vehicle_vel_2d = 15.0 * Eigen::Vector2d(sweep_direction(0), sweep_direction(1));

    // Generate sweep pattern
    Eigen::Vector2d candidate_vehicle_pos_2d = vehicle_vel_2d * dt + vehicle_pos_2d;
    if (polygon.isInside(candidate_vehicle_pos_2d)) {
      vehicle_pos_2d = candidate_vehicle_pos_2d;
      simulated_time += planning_horizon;
    } else {  // Reached the other end of the vertex
      sweep_direction = -1.0 * sweep_direction;
      vehicle_pos_2d = vehicle_pos_2d + sweep_perpendicular * view_distance;  // next row
      simulated_time += turning_time;

      if (!polygon.isInside(vehicle_pos_2d)) {
        break;
      }
    }
    if (increment % snapshot_increment == 0) {
      std::string saved_map_path =
          image_directory + "/gridmap_" + std::to_string(static_cast<int>(increment / snapshot_increment)) + ".bag";
      grid_map::GridMapRosConverter::saveToBag(adaptive_viewutility->getViewUtilityMap()->getGridMap(), saved_map_path,
                                               "/grid_map");
    }
    increment++;
    // Terminate if simulation time has exceeded
    if (simulated_time > max_experiment_duration) terminate_mapping = true;
    if (terminate_mapping) {
      break;
    }
    ros::Duration(0.5).sleep();
  }
  performance_tracker->Output(output_file_path);
  std::cout << "[TestGridNode] Grid Planner Terminated" << std::endl;

  ros::spin();
  return 0;
}

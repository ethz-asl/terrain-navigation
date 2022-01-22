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
#include "terrain_navigation/profiler.h"

void addViewpoint(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector4d att, Trajectory &trajectory) {
  State vehicle_state;
  vehicle_state.position = pos;
  vehicle_state.velocity = vel * 15.0;
  vehicle_state.attitude = att;
  trajectory.states.push_back(vehicle_state);
  return;
}

Trajectory getRandomViewPoint(std::shared_ptr<AdaptiveViewUtility> &adaptive_viewutility) {
  Eigen::Vector3d vehicle_pos;
  Eigen::Vector3d vehicle_vel(15.0, 0.0, 0.0);
  adaptive_viewutility->InitializeVehicleFromMap(vehicle_pos, vehicle_vel);
  adaptive_viewutility->setCurrentState(vehicle_pos, vehicle_vel);

  Trajectory viewpoint;
  adaptive_viewutility->InitializeVehicleFromMap(vehicle_pos, vehicle_vel);
  double elevation = adaptive_viewutility->getViewUtilityMap()->getGridMap().atPosition(
      "elevation", Eigen::Vector2d(vehicle_pos(0), vehicle_pos(1)));
  vehicle_pos(2) = elevation + getRandom(0.0, 300.0);
  double max_angle = 0.5 * M_PI * 0.5;
  double theta = getRandom(-max_angle, max_angle);
  Eigen::Vector3d axis;
  axis(0) = getRandom(-1.0, 1.0);
  axis(1) = getRandom(-1.0, 1.0);
  axis(2) = getRandom(-1.0, 1.0);
  axis.normalize();
  Eigen::Vector4d att(std::cos(0.5 * theta), std::sin(0.5 * theta) * axis(0), std::sin(0.5 * theta) * axis(1),
                      std::sin(0.5 * theta) * axis(2));
  addViewpoint(vehicle_pos, vehicle_vel, att, viewpoint);
  return viewpoint;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::string file_path, output_file_path;
  int num_experiments;
  double max_experiment_duration;
  bool enable_greedy{false};
  int snapshot_increment{20};
  nh_private.param<std::string>("file_path", file_path, "");
  nh_private.param<int>("num_experiments", num_experiments, 1);
  nh_private.param<int>("snapshot_interval", snapshot_increment, 25);
  nh_private.param<double>("max_experiment_duration", max_experiment_duration, 500);
  nh_private.param<bool>("enable_greedy", enable_greedy, false);
  nh_private.param<std::string>("output_file_path", output_file_path, "output/benchmark.csv");

  std::string image_directory{""};
  nh_private.param<std::string>("image_directory", image_directory, image_directory);

  /// set Current state of vehicle
  /// TODO: Randomly generate initial position

  for (int i = 0; i < num_experiments; i++) {
    std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);

    // Add elevation map from GeoTIFF file defined in path
    adaptive_viewutility->LoadMap(file_path);
    // adaptive_viewutility->getViewUtilityMap()->TransformMap(airsim_player_start);

    grid_map::Polygon polygon;

    // Hah! we have access to the map reference. be careful!
    grid_map::GridMap &map = adaptive_viewutility->getViewUtilityMap()->getGridMap();
    const Eigen::Vector2d map_pos = map.getPosition();
    const double map_width_x = map.getLength().x();
    const double map_width_y = map.getLength().y();

    polygon.setFrameId(map.getFrameId());
    polygon.addVertex(grid_map::Position(map_pos(0) - 0.4 * map_width_x, map_pos(1) - 0.4 * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) + 0.4 * map_width_x, map_pos(1) - 0.4 * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) + 0.4 * map_width_x, map_pos(1) + 0.4 * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) - 0.4 * map_width_x, map_pos(1) + 0.4 * map_width_y));

    adaptive_viewutility->getViewUtilityMap()->SetRegionOfInterest(polygon);

    Profiler pipeline_perf("Planner Loop");
    std::shared_ptr<PerformanceTracker> performance_tracker = std::make_shared<PerformanceTracker>(i);

    bool terminate_mapping = false;
    double simulated_time{0.0};
    int increment{0};
    const int num_samples{100};

    while (true) {
      pipeline_perf.tic();

      Trajectory first_segment;
      if (enable_greedy) {
        std::vector<Trajectory> candidate_viewpoints;
        for (int k = 0; k < num_samples; k++) {
          Trajectory view = getRandomViewPoint(adaptive_viewutility);
          candidate_viewpoints.push_back(view);
        }
        adaptive_viewutility->estimateViewUtility(candidate_viewpoints);
        first_segment = adaptive_viewutility->getBestPrimitive(candidate_viewpoints);
      } else {
        first_segment = getRandomViewPoint(adaptive_viewutility);
      }

      Eigen::Vector3d vehicle_pos = first_segment.states[0].position;
      Eigen::Vector4d vehicle_att = first_segment.states[0].attitude;

      adaptive_viewutility->UpdateUtility(first_segment);

      adaptive_viewutility->setCurrentState(first_segment.states.back().position, first_segment.states.back().velocity);

      pipeline_perf.toc();
      adaptive_viewutility->MapPublishOnce();
      adaptive_viewutility->ViewpointPublishOnce();
      adaptive_viewutility->publishViewpointHistory();

      double planning_horizon = 1.0;
      simulated_time += planning_horizon;

      double map_quality =
          performance_tracker->Record(simulated_time, adaptive_viewutility->getViewUtilityMap()->getGridMap());

      // Terminate if simulation time has exceeded
      if (simulated_time > max_experiment_duration) terminate_mapping = true;
      /// TODO: Define termination condition for map quality
      if (terminate_mapping) {
        break;
      }
    }
    performance_tracker->Output(output_file_path);
  }
  std::cout << "[TestPlannerNode] Planner terminated" << std::endl;

  ros::spin();
  return 0;
}

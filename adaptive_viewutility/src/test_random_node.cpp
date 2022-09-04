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
  double altitude = getRandom(50.0, 150.0);
  vehicle_pos(2) = elevation + altitude;
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

Trajectory &getRandomPrimitive(std::vector<Trajectory> &view_set) {
  while (true) {
    // Lazy sampling until we find a set that was not viewed
    int i = std::rand() % view_set.size();
    if (!view_set[i].viewed) {
      view_set[i].viewed = true;
      return view_set[i];
    }
  }
}

void OutputViewset(std::vector<Trajectory> &view_set, const std::string path) {
  // Write data to files
  int id;
  std::ofstream output_file;
  std::cout << "Writing viewset output to: " << path;
  output_file.open(path, std::ios::app);
  if (id == 0) {  // TODO: Make this nicer
    output_file << "id,x,y,z,qw, qx, qy, qz, padding,\n";
  }

  for (auto view : view_set) {
    output_file << id << ",";
    output_file << view.states[0].position(0) << ",";
    output_file << view.states[0].position(1) << ",";
    output_file << view.states[0].position(2) << ",";
    output_file << view.states[0].attitude(0) << ",";
    output_file << view.states[0].attitude(1) << ",";
    output_file << view.states[0].attitude(2) << ",";
    output_file << view.states[0].attitude(3) << ",";
    output_file << 0 << ",";
    output_file << "\n";
    id++;
  }
  output_file.close();
  return;
}

void ReadViewset(const std::string path, std::vector<Trajectory> &view_set) {
  // Write data to files
  bool parse_result;

  std::ifstream file(path);
  std::string str;

  // Look for the image file name in the path
  while (getline(file, str)) {
    std::stringstream ss(str);
    std::vector<std::string> data;
    std::string cc;
    while (getline(ss, cc, ',')) {
      data.push_back(cc);
    }
    // #   ID, x, y, z, qw, qx, qy, qz
    ss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6] >> data[7];
    if (data[0] == "id") continue;
    State state;
    state.position << std::stof(data[1]), std::stof(data[2]), std::stof(data[3]);
    state.attitude << std::stof(data[4]), std::stof(data[5]), std::stof(data[6]), std::stof(data[7]);

    Trajectory view;
    view.states.push_back(state);
    view_set.push_back(view);
  }
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher camera_path_pub = nh.advertise<nav_msgs::Path>("camera_path", 1, true);
  ros::Publisher camera_pose_pub = nh.advertise<geometry_msgs::PoseArray>("camera_poses", 1, true);
  ros::Publisher viewpoint_pub = nh.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);

  std::string file_path, output_dir_path, viewpoint_path;
  int num_experiments;
  double max_experiment_duration;
  bool enable_greedy{false};
  int snapshot_increment{20};
  int num_samples{200};
  nh_private.param<std::string>("file_path", file_path, "");
  nh_private.param<int>("num_experiments", num_experiments, 1);
  nh_private.param<int>("snapshot_interval", snapshot_increment, 20);
  nh_private.param<double>("max_experiment_duration", max_experiment_duration, 500);
  nh_private.param<int>("num_view_samples", num_samples, 200);
  nh_private.param<bool>("enable_greedy", enable_greedy, false);
  nh_private.param<std::string>("viewpoint_path", viewpoint_path, "");
  nh_private.param<std::string>("output_directory_path", output_dir_path, "output");

  std::string image_directory{""};
  nh_private.param<std::string>("image_directory", image_directory, image_directory);

  /// set Current state of vehicle
  /// TODO: Randomly generate initial position
  Eigen::Vector3d airsim_player_start{Eigen::Vector3d(374.47859375, -723.12984375, -286.77371094)};

  for (int i = 0; i < num_experiments; i++) {
    std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);

    // Add elevation map from GeoTIFF file defined in path
    adaptive_viewutility->LoadMap(file_path);
    adaptive_viewutility->getViewUtilityMap()->TransformMap(airsim_player_start);

    grid_map::Polygon polygon;

    // Hah! we have access to the map reference. be careful!
    grid_map::GridMap &map = adaptive_viewutility->getViewUtilityMap()->getGridMap();
    const Eigen::Vector2d map_pos = map.getPosition();
    const double map_width_x = map.getLength().x();
    const double map_width_y = map.getLength().y();

    polygon.setFrameId(map.getFrameId());
    double roi_ratio = 0.5;
    polygon.addVertex(grid_map::Position(map_pos(0) - roi_ratio * map_width_x, map_pos(1) - roi_ratio * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) + roi_ratio * map_width_x, map_pos(1) - roi_ratio * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) + roi_ratio * map_width_x, map_pos(1) + roi_ratio * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) - roi_ratio * map_width_x, map_pos(1) + roi_ratio * map_width_y));

    adaptive_viewutility->getViewUtilityMap()->SetRegionOfInterest(polygon);

    Profiler pipeline_perf("Planner Loop");
    std::shared_ptr<PerformanceTracker> performance_tracker = std::make_shared<PerformanceTracker>();

    bool terminate_mapping = false;
    double simulated_time{0.0};
    int increment{0};

    std::vector<Trajectory> candidate_viewpoints;
    if (viewpoint_path.empty()) {  // Generate viewpoints and save it to a file
      for (int k = 0; k < num_samples; k++) {
        Trajectory view = getRandomViewPoint(adaptive_viewutility);
        candidate_viewpoints.push_back(view);
      }
      std::string generated_view_set_path = output_dir_path + "/viewset.csv";
      OutputViewset(candidate_viewpoints, generated_view_set_path);
    } else {
      // Read viewpoints from saved path
      ReadViewset(viewpoint_path, candidate_viewpoints);
    }

    std::vector<Trajectory> executed_viewset;

    while (true) {
      // Terminate if simulation time has exceeded
      if (simulated_time >= max_experiment_duration) {
        break;
      }

      pipeline_perf.tic();
      /// TODO: Take out used viewpoints

      Trajectory first_segment;
      if (enable_greedy) {
        adaptive_viewutility->estimateViewUtility(candidate_viewpoints);
        first_segment = adaptive_viewutility->getBestPrimitive(candidate_viewpoints);
      } else {
        first_segment = getRandomPrimitive(candidate_viewpoints);
      }
      executed_viewset.push_back(first_segment);

      Eigen::Vector3d vehicle_pos = first_segment.states[0].position;
      Eigen::Vector4d vehicle_att = first_segment.states[0].attitude;

      adaptive_viewutility->UpdateUtility(first_segment);

      adaptive_viewutility->setCurrentState(first_segment.states.back().position, first_segment.states.back().velocity);

      pipeline_perf.toc();
      adaptive_viewutility->MapPublishOnce();
      adaptive_viewutility->ViewpointPublishOnce(camera_path_pub, camera_pose_pub);
      adaptive_viewutility->publishViewpoint(viewpoint_pub);
      adaptive_viewutility->publishCandidatePaths(candidate_viewpoints);

      double planning_horizon = 1.0;
      /// TODO: Use dubins distance for calculating time to reach viewpoint

      simulated_time += planning_horizon;
      increment++;

      performance_tracker->Record(simulated_time, adaptive_viewutility->getViewUtilityMap()->getGridMap());
      if (increment % snapshot_increment == 0) {
        std::string saved_map_path = image_directory + "/gridmap_" +
                                     std::to_string(static_cast<int>(increment / snapshot_increment) - 1) + ".bag";
        grid_map::GridMapRosConverter::saveToBag(adaptive_viewutility->getViewUtilityMap()->getGridMap(),
                                                 saved_map_path, "/grid_map");
      }
    }
    std::string saved_map_path = image_directory + "/gridmap" + ".bag";
    grid_map::GridMapRosConverter::saveToBag(adaptive_viewutility->getViewUtilityMap()->getGridMap(), saved_map_path,
                                             "/grid_map");
    std::cout << "Final : " << saved_map_path << std::endl;
    std::cout << "[TestPlannerNode] Planner terminated experiment: " << i << std::endl;
    std::string executed_view_set_path = output_dir_path + "/executed_viewset.csv";
    std::cout << "[TestRandomAirsimNode] Executed view set path: " << executed_view_set_path << std::endl;
    OutputViewset(executed_viewset, executed_view_set_path);
  }
  std::string benchmark_file_path = output_dir_path + "/benchmark.csv";
  std::cout << "[TestPlannerNode] Planner terminated" << std::endl;

  ros::spin();
  return 0;
}

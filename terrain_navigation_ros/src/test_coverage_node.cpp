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
#include "terrain_navigation/data_logger.h"
#include "terrain_navigation/profiler.h"
#include "terrain_planner/terrain_ompl_rrt.h"

void addViewpoint(Trajectory &trajectory, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector4d att) {
  State vehicle_state;
  vehicle_state.position = pos;
  vehicle_state.velocity = vel * 15.0;
  vehicle_state.attitude = att;
  trajectory.states.push_back(vehicle_state);
  return;
}

double interpolateDubins(Eigen::Vector3d start_pos, Eigen::Vector3d start_vel, Eigen::Vector3d goal_pos,
                         Eigen::Vector3d goal_vel, double progress, Eigen::Vector3d &interpolated_state) {
  auto dubins_ss = std::make_shared<fw_planning::spaces::DubinsAirplaneStateSpace>();
  dubins_ss->setMaxClimbingAngle(1.0);

  ompl::base::State *from = dubins_ss->allocState();
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(start_pos.x());
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(start_pos.y());
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(start_pos.z());
  double start_yaw = std::atan2(start_vel.y(), start_vel.x());
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(start_yaw);

  ompl::base::State *to = dubins_ss->allocState();
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(goal_pos.x());
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(goal_pos.y());
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(goal_pos.z());
  double goal_yaw = std::atan2(goal_vel.y(), goal_vel.x());
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(goal_yaw);

  ompl::base::State *state = dubins_ss->allocState();

  fw_planning::spaces::DubinsPath dubins_path;
  fw_planning::spaces::DubinsAirplaneStateSpace::SegmentStarts segmentStarts;
  bool first_time = true;
  dubins_ss->interpolate(from, to, progress, first_time, dubins_path, segmentStarts, state);

  interpolated_state.x() = state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX();
  interpolated_state.y() = state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY();
  interpolated_state.z() = state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ();

  double curvature = dubins_ss->getMinTurningRadius();

  dubins_ss->freeState(from);
  dubins_ss->freeState(to);

  return dubins_path.length_3D() * curvature;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::vector<std::shared_ptr<DataLogger>> benchmark_results;

  std::string file_path, result_directory;
  double max_experiment_duration;
  int num_experiments{1};
  double dt = 1.0;
  nh_private.param<std::string>("file_path", file_path, "resources/cadastre.tif");
  nh_private.param<double>("max_experiment_duration", max_experiment_duration, 500);
  nh_private.param<std::string>("result_directory", result_directory, "output");
  nh_private.param<int>("num_experiments", num_experiments, num_experiments);
  nh_private.param<double>("dt", dt, dt);

  ros::Publisher camera_path_pub = nh.advertise<nav_msgs::Path>("camera_path", 1, true);
  ros::Publisher camera_pose_pub = nh.advertise<geometry_msgs::PoseArray>("camera_poses", 1, true);
  ros::Publisher viewpoint_pub = nh.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);
  bool random_initial_state{false};

  for (int i = 0; i < num_experiments; i++) {
    std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);
    // Add elevation map from GeoTIFF file defined in path
    adaptive_viewutility->LoadMap(file_path);

    /// set Current state of vehicle
    grid_map::Polygon polygon;
    // Hah! we have access to the map reference. be careful!
    grid_map::GridMap &map = adaptive_viewutility->getViewUtilityMap()->getGridMap();
    const Eigen::Vector2d map_pos = map.getPosition();
    const double map_width_x = map.getLength().x();
    const double map_width_y = map.getLength().y();

    polygon.setFrameId(map.getFrameId());
    polygon.addVertex(grid_map::Position(map_pos(0) - 0.5 * map_width_x, map_pos(1) - 0.5 * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) + 0.5 * map_width_x, map_pos(1) - 0.5 * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) + 0.5 * map_width_x, map_pos(1) + 0.5 * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) - 0.5 * map_width_x, map_pos(1) + 0.5 * map_width_y));

    adaptive_viewutility->getViewUtilityMap()->SetRegionOfInterest(polygon);

    grid_map::Polygon offset_polygon = polygon;
    /// TODO: Get sweep distance from sensor model
    double altitude = 100.0;
    /// TODO: Define offset from sensor model
    offset_polygon.offsetInward(10.0);

    Eigen::Vector2d coverage_start_pos_2d = offset_polygon.getVertex(0);
    double coverage_start_elevation = map.atPosition("elevation", coverage_start_pos_2d);
    Eigen::Vector3d coverage_start_pos = Eigen::Vector3d(
        offset_polygon.getVertex(0).x(), offset_polygon.getVertex(0).y(), coverage_start_elevation + altitude);

    // Run sweep segments until the boundary
    double sweep_distance = 51.6;
    Eigen::Vector2d sweep_direction = (offset_polygon.getVertex(1) - offset_polygon.getVertex(0)).normalized();
    Eigen::Vector2d sweep_perpendicular = (offset_polygon.getVertex(3) - offset_polygon.getVertex(0)).normalized();
    // From "Huang, Wesley H. "Optimal line-sweep-based decompositions for coverage algorithms." Proceedings 2001 ICRA.
    // IEEE International Conference on Robotics and Automation (Cat. No. 01CH37164). Vol. 1. IEEE, 2001."
    // The optimal sweep pattern direction coinsides with the edge direction of the ROI polygon

    std::shared_ptr<PerformanceTracker> performance_tracker = std::make_shared<PerformanceTracker>();

    auto data_logger = std::make_shared<DataLogger>();
    data_logger->setKeys({"timestamp", "coverage", "quality"});

    /// TODO: Get start velocity from geometry
    Eigen::Vector3d vehicle_pos(map_pos(0), map_pos(1), 0.0);
    Eigen::Vector3d vehicle_vel(15.0, 0.0, 0.0);
    if (random_initial_state) {
      adaptive_viewutility->InitializeVehicleFromMap(vehicle_pos, vehicle_vel);
    } else {
      vehicle_pos = coverage_start_pos;
      random_initial_state = true;
    }

    // Place holders
    double simulated_time{0.0};
    double last_triggered_time{0.0};
    double recording_rate{2.0};  // seconds [s]
    bool survey_start = true;
    double planning_horizon = 2.0;
    Eigen::Vector3d previous_candidate_pos = vehicle_pos;
    Eigen::Vector3d previous_vehicle_vel = vehicle_vel;

    // Generate a lawnmower pattern survey

    // Get dubins distance to start position
    Eigen::Vector3d interpolated_pos;
    double dubins_distance =
        interpolateDubins(vehicle_pos, vehicle_vel, coverage_start_pos,
                          Eigen::Vector3d(sweep_direction.x(), sweep_direction.y(), 0.0), 1.0, interpolated_pos);
    double traverse_time = dubins_distance / vehicle_vel.norm();

    double elapsed_time = 0.0;
    double last_update_time = simulated_time;

    Eigen::Vector3d candidate_vehicle_pos = coverage_start_pos;
    Eigen::Vector2d candidate_vehicle_pos_2d = candidate_vehicle_pos.head(2);
    Eigen::Vector2d candiate_vehicle_vel_2d = Eigen::Vector2d(sweep_direction.x(), sweep_direction.y());

    std::vector<Eigen::Vector3d> vehicle_path;
    bool constrain_elevation{true};
    while (true) {
      elapsed_time = simulated_time - last_update_time;
      if (elapsed_time >= traverse_time) {
        previous_candidate_pos = candidate_vehicle_pos;
        // Update where the next viewpoint should be
        candiate_vehicle_vel_2d = 2 * 15.0 * Eigen::Vector2d(sweep_direction(0), sweep_direction(1));
        // Generate sweep pattern
        bool turning = false;
        candidate_vehicle_pos_2d = candiate_vehicle_vel_2d + vehicle_pos.head(2);
        if (!polygon.isInside(candidate_vehicle_pos_2d)) {  // Reached the other end of the vertex
          sweep_direction = -1.0 * sweep_direction;
          candidate_vehicle_pos_2d = vehicle_pos.head(2) + sweep_perpendicular * sweep_distance;  // next row
          candiate_vehicle_vel_2d = 2 * 15.0 * Eigen::Vector2d(sweep_direction(0), sweep_direction(1));
          turning = true;
          if (!polygon.isInside(candidate_vehicle_pos_2d)) {
            // Grid pattern has been completed
            break;
          }
        }
        double candidate_elevation = map.atPosition("elevation", candidate_vehicle_pos_2d) + altitude;

        if (constrain_elevation && !turning) {
          // Constrain next viewpoint with climbrate
          double gamma = 0.1;  // maximum flight path angle
          double max_elevation_difference =
              (previous_candidate_pos.head(2) - candidate_vehicle_pos_2d).norm() * std::tan(gamma);
          candidate_elevation =
              previous_candidate_pos.z() +
              std::max(std::min(candidate_elevation - previous_candidate_pos.z(), max_elevation_difference),
                       -max_elevation_difference);
        }
        candidate_vehicle_pos =
            Eigen::Vector3d(candidate_vehicle_pos_2d.x(), candidate_vehicle_pos_2d.y(), candidate_elevation);

        double residual_time = traverse_time - elapsed_time;
        Eigen::Vector3d interpolated_pos;
        double distance = interpolateDubins(
            vehicle_pos, vehicle_vel, candidate_vehicle_pos,
            Eigen::Vector3d(candiate_vehicle_vel_2d.x(), candiate_vehicle_vel_2d.y(), 0.0), 1.0, interpolated_pos);
        traverse_time = (distance / 15.0) + residual_time;
        previous_vehicle_vel = vehicle_vel;
        last_update_time = simulated_time;
        elapsed_time = 0.0;

        Trajectory reference_trajectory;
        adaptive_viewutility->setCurrentState(previous_candidate_pos, vehicle_vel);
        addViewpoint(reference_trajectory, previous_candidate_pos, vehicle_vel, Eigen::Vector4d(1.0, 0.0, 0.0, 0.0));
        adaptive_viewutility->UpdateUtility(reference_trajectory);
      }

      double progress = elapsed_time / traverse_time;
      interpolateDubins(previous_candidate_pos, previous_vehicle_vel, candidate_vehicle_pos,
                        Eigen::Vector3d(candiate_vehicle_vel_2d.x(), candiate_vehicle_vel_2d.y(), 0.0), progress,
                        vehicle_pos);

      vehicle_path.push_back(vehicle_pos);
      vehicle_vel = Eigen::Vector3d(candiate_vehicle_vel_2d.x(), candiate_vehicle_vel_2d.y(), 0.0);

      if ((simulated_time - last_triggered_time) > planning_horizon) {
        Metrics performance =
            performance_tracker->Record(simulated_time, adaptive_viewutility->getViewUtilityMap()->getGridMap());

        std::unordered_map<std::string, std::any> state;
        state.insert(std::pair<std::string, double>("timestamp", performance.time));
        state.insert(std::pair<std::string, double>("coverage", performance.coverage));
        state.insert(std::pair<std::string, double>("quality", performance.quality));
        data_logger->record(state);

        last_triggered_time = simulated_time;
      }

      // Visualize results
      std::vector<geometry_msgs::PoseStamped> posestampedhistory_vector;
      for (auto &state : vehicle_path) {
        posestampedhistory_vector.insert(posestampedhistory_vector.begin(),
                                         vector3d2PoseStampedMsg(state, Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
      }
      nav_msgs::Path msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "map";
      msg.poses = posestampedhistory_vector;
      camera_path_pub.publish(msg);

      adaptive_viewutility->MapPublishOnce();
      adaptive_viewutility->publishViewpoint(viewpoint_pub);

      // Terminate if simulation time has exceeded
      if (simulated_time > max_experiment_duration) {
        break;
      }
      simulated_time += dt;
      // ros::Duration(dt).sleep();
    }
    benchmark_results.emplace_back(data_logger);
  }

  std::cout << "Running evaluations: " << std::endl;
  // Evaluate benchmark results
  auto results_logger = std::make_shared<DataLogger>();
  results_logger->setPrintHeader(true);
  results_logger->setKeys({"timestamp", "coverage", "quality", "coverage_variance", "quality_variance", "coverage_min",
                           "coverage_max", "quality_min", "quality_max"});
  std::cout << "[TestPlannerNode] Number of benchmarks: " << benchmark_results.size() << std::endl;
  int num_data_points = benchmark_results[0]->count();
  for (int i = 0; i < benchmark_results[0]->count(); i++) {
    double time = std::any_cast<double &>(benchmark_results[0]->data()[i].at("timestamp"));
    double quality_mean{0.0};
    double coverage_mean{0.0};
    double quality_squared{0.0};
    double coverage_squared{0.0};
    double quality_min = std::numeric_limits<double>::infinity();
    double quality_max = -std::numeric_limits<double>::infinity();
    double coverage_min = std::numeric_limits<double>::infinity();
    double coverage_max = -std::numeric_limits<double>::infinity();
    int data_count{0};
    for (auto &result : benchmark_results) {
      if (i < result->data().size()) {
        double quality = std::any_cast<double &>(result->data()[i].at("quality"));
        quality_mean += quality;
        if (quality > quality_max) quality_max = quality;
        if (quality < quality_min) quality_min = quality;
        quality_squared += std::pow(quality, 2);
        double coverage = std::any_cast<double &>(result->data()[i].at("coverage"));
        coverage_mean += coverage;
        if (coverage > coverage_max) coverage_max = coverage;
        if (coverage < coverage_min) coverage_min = coverage;
        coverage_squared += std::pow(coverage, 2);
        data_count++;
      }
    }
    quality_mean = quality_mean / data_count;
    coverage_mean = coverage_mean / data_count;
    quality_squared = quality_squared / data_count;
    coverage_squared = coverage_squared / data_count;

    double quality_variance = quality_squared - std::pow(quality_mean, 2);
    double coverage_variance = coverage_squared - std::pow(coverage_mean, 2);
    std::unordered_map<std::string, std::any> average;
    average.insert(std::pair<std::string, double>("timestamp", time));
    average.insert(std::pair<std::string, double>("coverage", coverage_mean));
    average.insert(std::pair<std::string, double>("quality", quality_mean));
    average.insert(std::pair<std::string, double>("quality_variance", quality_variance));
    average.insert(std::pair<std::string, double>("coverage_variance", coverage_variance));
    average.insert(std::pair<std::string, double>("quality_variance", quality_variance));
    average.insert(std::pair<std::string, double>("coverage_min", coverage_min));
    average.insert(std::pair<std::string, double>("coverage_max", coverage_max));
    average.insert(std::pair<std::string, double>("quality_max", quality_max));
    average.insert(std::pair<std::string, double>("quality_min", quality_min));

    results_logger->record(average);
  }
  std::string benchmark_output_path = result_directory + "/coverage_benchmark.txt";
  results_logger->writeToFile(benchmark_output_path);
  std::cout << "[TestGridNode] Grid Planner Terminated" << std::endl;

  ros::spin();
  return 0;
}

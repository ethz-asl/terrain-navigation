#include "terrain_planner/visualization.h"

void publishCandidateManeuvers(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                               const std::vector<Path>& candidate_maneuvers, bool visualize_invalid_trajectories) {
  visualization_msgs::msg::MarkerArray msg;

  std::vector<visualization_msgs::msg::Marker> marker;
  visualization_msgs::msg::Marker mark;
  mark.action = visualization_msgs::msg::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub->publish(msg);

  std::vector<visualization_msgs::msg::Marker> maneuver_library_vector;
  int i = 0;
  for (auto maneuver : candidate_maneuvers) {
    if (maneuver.valid() || visualize_invalid_trajectories) {
      maneuver_library_vector.insert(maneuver_library_vector.begin(), trajectory2MarkerMsg(maneuver, i));
    }
    i++;
  }
  msg.markers = maneuver_library_vector;
  pub->publish(msg);
}

void publishPositionSetpoints(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                              const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                              const Eigen::Vector3d scale) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = rclcpp::Clock().now();
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  pub->publish(marker);

  marker.header.stamp = rclcpp::Clock().now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = scale(0);
  marker.scale.y = scale(1);
  marker.scale.z = scale(2);
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.pose.position = tf2::toMsg(position);
  tf2::Quaternion q;
  q.setRPY(0, 0, std::atan2(velocity.y(), velocity.x()));
  marker.pose.orientation = tf2::toMsg(q);

  pub->publish(marker);
}

void publishPath(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, std::vector<Eigen::Vector3d> path,
                 Eigen::Vector3d color) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = rclcpp::Clock().now();
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::ADD;
  std::vector<geometry_msgs::msg::Point> points;
  for (auto& position : path) {
    geometry_msgs::msg::Point point;
    point.x = position(0);
    point.y = position(1);
    point.z = position(2);
    points.push_back(point);
  }
  std::cout << "Points: " << points.size() << std::endl;
  marker.points = points;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 10.0;
  marker.scale.y = 10.0;
  marker.scale.z = 10.0;
  marker.color.a = 0.8;
  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();
  pub->publish(marker);
}

void publishTrajectory(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub, std::vector<Eigen::Vector3d> trajectory) {
  nav_msgs::msg::Path msg;
  std::vector<geometry_msgs::msg::PoseStamped> posestampedhistory_vector;
  Eigen::Vector4d orientation(1.0, 0.0, 0.0, 0.0);
  for (auto pos : trajectory) {
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(), vector3d2PoseStampedMsg(pos, orientation));
  }
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "map";
  msg.poses = posestampedhistory_vector;
  pub->publish(msg);
}

void publishTree(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                 std::shared_ptr<ompl::base::PlannerData> planner_data,
                 std::shared_ptr<ompl::OmplSetup> problem_setup) {
  visualization_msgs::msg::MarkerArray marker_array;
  std::vector<visualization_msgs::msg::Marker> marker;

  planner_data->decoupleFromPlanner();

  // Create states, a marker and a list to store edges
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> vertex(problem_setup->getSpaceInformation());
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> neighbor_vertex(
      problem_setup->getSpaceInformation());
  size_t marker_idx{0};
  auto dubins_ss = std::make_shared<fw_planning::spaces::DubinsAirplaneStateSpace>();
  for (size_t i = 0; i < planner_data->numVertices(); i++) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = rclcpp::Clock().now();
    marker.header.frame_id = "map";
    vertex = planner_data->getVertex(i).getState();
    marker.ns = "vertex";
    marker.id = marker_idx++;
    marker.pose.position.x = vertex[0];
    marker.pose.position.y = vertex[1];
    marker.pose.position.z = vertex[2];
    marker.pose.orientation.w = std::cos(0.5 * vertex[3]);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = std::sin(0.5 * vertex[3]);
    marker.scale.x = 10.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);

    // allocate variables
    std::vector<unsigned int> edge_list;
    int num_edges = planner_data->getEdges(i, edge_list);
    if (num_edges > 0) {
      for (unsigned int edge : edge_list) {
        visualization_msgs::msg::Marker edge_marker;
        edge_marker.header.stamp = rclcpp::Clock().now();
        edge_marker.header.frame_id = "map";
        edge_marker.id = marker_idx++;
        edge_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        edge_marker.ns = "edge";
        neighbor_vertex = planner_data->getVertex(edge).getState();
        // points.push_back(toMsg(Eigen::Vector3d(vertex[0], vertex[1], vertex[2])));
        // points.push_back(toMsg(Eigen::Vector3d(neighbor_vertex[0], neighbor_vertex[1], neighbor_vertex[2])));
        ompl::base::State* state = dubins_ss->allocState();
        ompl::base::State* from = dubins_ss->allocState();
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(vertex[0]);
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(vertex[1]);
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(vertex[2]);
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(vertex[3]);

        ompl::base::State* to = dubins_ss->allocState();
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(neighbor_vertex[0]);
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(neighbor_vertex[1]);
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(neighbor_vertex[2]);
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(neighbor_vertex[3]);
        if (dubins_ss->equalStates(from, to)) {
          continue;
        }
        std::vector<geometry_msgs::msg::Point> points;
        for (double t = 0.0; t < 1.0; t += 0.02) {
          dubins_ss->interpolate(from, to, t, state);
          auto interpolated_state =
              Eigen::Vector3d(state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(),
                              state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY(),
                              state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ());
          points.push_back(tf2::toMsg(interpolated_state));
        }
        points.push_back(tf2::toMsg(Eigen::Vector3d(neighbor_vertex[0], neighbor_vertex[1], neighbor_vertex[2])));
        edge_marker.points = points;
        edge_marker.action = visualization_msgs::msg::Marker::ADD;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.pose.orientation.x = 0.0;
        edge_marker.pose.orientation.y = 0.0;
        edge_marker.pose.orientation.z = 0.0;
        edge_marker.scale.x = 1.0;
        edge_marker.scale.y = 1.0;
        edge_marker.scale.z = 1.0;
        edge_marker.color.a = 0.5;  // Don't forget to set the alpha!
        edge_marker.color.r = 1.0;
        edge_marker.color.g = 1.0;
        edge_marker.color.b = 0.0;
        marker_array.markers.push_back(edge_marker);
      }
    }
  }
  pub->publish(marker_array);
}

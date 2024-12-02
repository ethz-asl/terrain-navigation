#include "terrain_abort_planner/mission_io.h"


Path generatePathFromWaypoints (std::vector<Eigen::Vector3d> waypoint_list, const double radius) {
  Path mission_path;
  double previous_distance_to_tangent = 0.0;
  for (int waypoint_id = 1; waypoint_id < waypoint_list.size() - 1; waypoint_id++) {
    const Eigen::Vector3d current_vertex = waypoint_list[waypoint_id];
    const Eigen::Vector3d current_edge = current_vertex - waypoint_list[waypoint_id - 1];
    const double current_edge_length = current_edge.norm();
    // if (current_edge_length < 0.01) continue; //Same position
    const Eigen::Vector3d current_tangent = current_edge / current_edge_length;
    const double current_course = std::atan2(current_tangent[1], current_tangent[0]);

    Eigen::Vector3d next_edge = waypoint_list[waypoint_id + 1] - current_vertex;
    double next_edge_length = next_edge.norm();
    Eigen::Vector3d next_tangent = next_edge / next_edge_length;
    double next_course = std::atan2(next_tangent[1], next_tangent[0]);

    double turn_angle = getTurnAngle(current_course, next_course);
    double distance_to_tangent = radius / std::abs(std::tan(0.5 * turn_angle));

    /// TODO: Arc segment
    Eigen::Vector3d arc_segment_start = current_vertex - distance_to_tangent * current_tangent;
    Eigen::Vector3d arc_segment_end = current_vertex + distance_to_tangent * next_tangent;
    double arc_segment_length = std::abs(M_PI - turn_angle) * radius;
    Eigen::Vector3d rotation_vector = current_tangent.cross(next_tangent);
    double curvature = rotation_vector(2) > 0.0 ? 1.0 / radius : -1.0 / radius;

    auto arc_segment =
        generateTrajectory(curvature, arc_segment_length, arc_segment_start, arc_segment_end, current_tangent, 0.1);
    /// TODO: Line segment
    Eigen::Vector3d line_segment_start =
        waypoint_list[waypoint_id - 1] + current_tangent * previous_distance_to_tangent;
    Eigen::Vector3d line_segment_end = arc_segment_start;
    double line_segment_length = current_edge_length - previous_distance_to_tangent - distance_to_tangent;
    auto line_segment =
        generateTrajectory(0.0, line_segment_length, line_segment_start, line_segment_end, current_tangent, 0.1);
    previous_distance_to_tangent = distance_to_tangent;

    mission_path.appendSegment(line_segment);
    mission_path.appendSegment(arc_segment);
  }
  return mission_path;
}

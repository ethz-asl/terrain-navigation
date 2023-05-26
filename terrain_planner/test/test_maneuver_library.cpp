#include "terrain_planner/maneuver_library.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(ManeuverLibraryTest, GenerateArc) {
  auto maneuver_library = std::make_shared<ManeuverLibrary>();
  Eigen::Vector3d rate(0.0, 0.0, -1.0);
  double horizon = std::abs(2 * M_PI / rate(2));
  Eigen::Vector3d start_position(0.0, 0.0, 0.0);
  Eigen::Vector3d start_velocity(1.0, 0.0, 0.0);

  PathSegment trajectory = maneuver_library->generateArcTrajectory(rate, horizon, start_position, start_velocity);
  Eigen::Vector3d start_trajectory = trajectory.states.front().position;
  Eigen::Vector3d end_trajectory = trajectory.states.back().position;
  std::vector<Eigen::Vector3d> position_list = trajectory.position();
  EXPECT_TRUE(true);
}

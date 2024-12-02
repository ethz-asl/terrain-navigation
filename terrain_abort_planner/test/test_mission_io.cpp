#include "terrain_abort_planner/mission_io.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(MissionIO, PathGeneration) {
  EXPECT_NEAR(getTurnAngle(0.25 * M_PI, 0.25*M_PI), M_PI, 0.001);
  EXPECT_NEAR(getTurnAngle(0.0, 0.5*M_PI), 0.5*M_PI, 0.001);
  EXPECT_NEAR(getTurnAngle(0.0, -0.5*M_PI), 0.5*M_PI, 0.001);
  EXPECT_NEAR(getTurnAngle(0.75 * M_PI, -0.75 * M_PI), 0.5*M_PI, 0.001);

  // EXPECT_NEAR(getTurnAngle(0.25 * M_PI, 0.25*M_PI), M_PI, 0.001);
  // EXPECT_NEAR(getTurnAngle(0.25 * M_PI, 0.25*M_PI), M_PI, 0.001);
  // EXPECT_NEAR(getTurnAngle(0.25 * M_PI, 0.25*M_PI), M_PI, 0.001);


  std::vector<Eigen::Vector3d> waypoint_list;
  double radius = 1.0;

}

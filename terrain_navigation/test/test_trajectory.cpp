#include "terrain_navigation/trajectory.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(TrajectoryTest, getArcCenter) {
  Eigen::Vector2d segment_start(0.0, 0.0);
  Eigen::Vector2d segment_end(0.0, 2.0);
  Eigen::Vector2d expected_center(0.0, 1.0);

  Eigen::Vector2d arc_center = Trajectory::getArcCenter(segment_start, segment_end, 1.0);
  ASSERT_TRUE(arc_center.isApprox(Eigen::Vector2d(0.0, 1.0)));
  Eigen::Vector2d arc_center2 = Trajectory::getArcCenter(segment_start, segment_end, -1.0);
  ASSERT_TRUE(arc_center2.isApprox(Eigen::Vector2d(0.0, 1.0)));

  Eigen::Vector2d arc_center3 = Trajectory::getArcCenter(segment_start, Eigen::Vector2d(1.0, 1.0), -1.0);
  ASSERT_TRUE(arc_center3.isApprox(Eigen::Vector2d(0.0, 1.0)));
}

TEST(TrajectoryTest, getArcProgress) {
  Eigen::Vector2d segment_start(0.0, 0.0);
  Eigen::Vector2d segment_end(0.0, 2.0);
  Eigen::Vector2d expected_center(0.0, 1.0);
  double theta;
  Eigen::Vector2d arc_center = Trajectory::getArcCenter(segment_start, segment_end, 1.0);
  theta = Trajectory::getArcProgress(arc_center, segment_start, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  theta = Trajectory::getArcProgress(arc_center, segment_end, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);
  theta = Trajectory::getArcProgress(arc_center, segment_start, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);
  theta = Trajectory::getArcProgress(arc_center, segment_end, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);

  Eigen::Vector2d test_position(0.0, -1.0);
  theta = Trajectory::getArcProgress(arc_center, test_position, segment_start, segment_end, 1.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  ASSERT_DOUBLE_EQ(theta, 0.0);
  theta = Trajectory::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  Eigen::Vector2d test_position2(1.0, 1.0);
  theta = Trajectory::getArcProgress(arc_center, test_position2, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 0.5);

  // Eigen::Vector2d test_position3(-1.0, 1.0);
  // theta = Trajectory::getArcProgress(arc_center, test_position3, segment_start, segment_end, -1.0);
  // ASSERT_DOUBLE_EQ(theta, 0.5);
}

TEST(TrajectoryTest, getLineProgress) {
  Eigen::Vector3d segment_start(0.0, 0.0, 0.0);
  Eigen::Vector3d segment_end(0.0, 0.0, 1.0);
  Eigen::Vector3d progress(0.0, 0.0, 0.5);

  double theta{0.0};
  theta = Trajectory::getLineProgress(progress, segment_start, segment_end);
  ASSERT_DOUBLE_EQ(theta, 0.5);

  theta = Trajectory::getLineProgress(segment_start, segment_start, segment_end);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  theta = Trajectory::getLineProgress(segment_end, segment_start, segment_end);
  ASSERT_DOUBLE_EQ(theta, 1.0);
}

TEST(TrajectorySegmentTest, getClosestPoint) {
  TrajectorySegments trajectory;
  Trajectory first_arc_segment;

  State segment_start;
  segment_start.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  segment_start.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  State segment_middle;
  segment_middle.position = Eigen::Vector3d(1.0, 1.0, 0.0);
  segment_middle.velocity = Eigen::Vector3d(0.0, 1.0, 0.0);

  first_arc_segment.curvature = -1.0;
  first_arc_segment.states.push_back(segment_start);
  first_arc_segment.states.push_back(segment_middle);
  trajectory.appendSegment(first_arc_segment);

  Trajectory second_straight_segment;

  State second_segment_end;
  second_segment_end.position = Eigen::Vector3d(1.0, 2.0, 0.0);
  second_segment_end.velocity = Eigen::Vector3d(0.0, 1.0, 0.0);

  second_straight_segment.curvature = 0.0;
  second_straight_segment.states.push_back(segment_middle);
  second_straight_segment.states.push_back(second_segment_end);
  trajectory.appendSegment(second_straight_segment);

  Eigen::Vector3d query_poistion;
  Eigen::Vector3d closest_point, tangent;
  double curvature;

  query_poistion = Eigen::Vector3d(0.0, -1.0, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d::Zero()));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(curvature, -1.0);

  query_poistion = Eigen::Vector3d(1.0, 0.0, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d(std::sin(M_PI / 4.0), 1.0 - std::cos(M_PI / 4.0), 0.0)));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(1.0, 1.0, 0.0).normalized()));
  EXPECT_DOUBLE_EQ(curvature, -1.0);

  query_poistion = Eigen::Vector3d(2.0, 1.1, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d(1.0, 1.1, 0.0)));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0)));
  EXPECT_DOUBLE_EQ(curvature, 0.0);
}

TEST(TrajectorySegmentTest, getClosestPoint2) {
  TrajectorySegments trajectory;
  Trajectory first_arc_segment;

  State segment_start;
  segment_start.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  segment_start.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  State segment_middle;
  segment_middle.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  segment_middle.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  first_arc_segment.curvature = 0.0;
  first_arc_segment.states.push_back(segment_start);
  first_arc_segment.states.push_back(segment_middle);
  trajectory.appendSegment(first_arc_segment);

  Trajectory second_straight_segment;

  State second_segment_end;
  second_segment_end.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  second_segment_end.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  second_straight_segment.curvature = -1.0;
  second_straight_segment.states.push_back(segment_middle);
  second_straight_segment.states.push_back(second_segment_end);
  trajectory.appendSegment(second_straight_segment);

  Eigen::Vector3d query_poistion;
  Eigen::Vector3d closest_point, tangent;
  double curvature;

  query_poistion = Eigen::Vector3d(0.0, -1.0, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d::Zero()));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(curvature, 0.0);

  query_poistion = Eigen::Vector3d(0.5, 1.0, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d(0.5, 0.0, 0.0)));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(curvature, 0.0);

  query_poistion = Eigen::Vector3d(2.0, 0.0, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d(1.0 + std::sin(M_PI / 4.0), 1.0 - std::cos(M_PI / 4.0), 0.0)));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(1.0, 1.0, 0.0).normalized()));
  EXPECT_DOUBLE_EQ(curvature, -1.0);

  query_poistion = Eigen::Vector3d(-0.5, 1.0, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0)));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(0.0, -1.0, 0.0).normalized()));
  EXPECT_DOUBLE_EQ(curvature, -1.0);
}

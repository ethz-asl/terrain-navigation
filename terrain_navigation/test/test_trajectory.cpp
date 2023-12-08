#include <gtest/gtest.h>

#include <iostream>

#include "terrain_navigation/path.h"

//! @todo(srmainwaring)check includes
// #include "terrain_navigation/data_logger.h"
// #include "terrain_navigation/path_segment.h"
// #include "terrain_navigation/path.h"
// #include "terrain_navigation/primitive.h"
// #include "terrain_navigation/profiler.h"
// #include "terrain_navigation/terrain_map.h"
// #include "terrain_navigation/viewpoint.h"
// #include "terrain_navigation/visualization.h"

#define FLOAT_EPS 1e-6

TEST(PathSegmentTest, getArcCenter) {
  Eigen::Vector2d segment_start(0.0, 0.0);
  Eigen::Vector2d segment_start_tangent;

  Eigen::Vector2d segment_end(0.0, 2.0);
  Eigen::Vector2d expected_center(0.0, 1.0);

  segment_start_tangent << 1.0, 0.0;
  Eigen::Vector2d arc_center = PathSegment::getArcCenter(segment_start, segment_start_tangent, 1.0);

  ASSERT_TRUE(arc_center.isApprox(Eigen::Vector2d(0.0, 1.0)));
  segment_start_tangent << -1.0, 0.0;
  Eigen::Vector2d arc_center2 = PathSegment::getArcCenter(segment_start, segment_start_tangent, -1.0);
  ASSERT_TRUE(arc_center2.isApprox(Eigen::Vector2d(0.0, 1.0)));
}

TEST(PathSegmentTest, getArcProgress) {
  Eigen::Vector2d segment_start(0.0, 0.0);
  Eigen::Vector2d segment_start_tangent(1.0, 0.0);
  Eigen::Vector2d segment_end(0.0, 2.0);
  Eigen::Vector2d expected_center(0.0, 1.0);
  double theta;
  Eigen::Vector2d arc_center = PathSegment::getArcCenter(segment_start, segment_start_tangent, 1.0);
  theta = PathSegment::getArcProgress(arc_center, segment_start, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  theta = PathSegment::getArcProgress(arc_center, segment_end, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);
  theta = PathSegment::getArcProgress(arc_center, segment_start, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);
  theta = PathSegment::getArcProgress(arc_center, segment_end, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);

  Eigen::Vector2d test_position(0.0, -1.0);
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, 1.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  ASSERT_DOUBLE_EQ(theta, 0.0);
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);
}

TEST(PathSegmentTest, getArcProgress2) {
  // Test close to full circle arc segments
  Eigen::Vector2d segment_start(1.0, 0.0);
  Eigen::Vector2d segment_start_tangent(0.0, 1.0);
  double end_theta = 1.75 * M_PI;
  Eigen::Vector2d segment_end(std::cos(end_theta), std::sin(end_theta));
  Eigen::Vector2d expected_center(0.0, 0.0);
  double theta;
  Eigen::Vector2d arc_center = PathSegment::getArcCenter(segment_start, segment_start_tangent, 1.0);
  ASSERT_TRUE((arc_center - expected_center).norm() < FLOAT_EPS);

  theta = PathSegment::getArcProgress(arc_center, segment_start, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  theta = PathSegment::getArcProgress(arc_center, segment_end, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);

  Eigen::Vector2d test_position(2.0, 0.0);
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  test_position << 0.0, 2.0;
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 2.0 / 7.0);
}

TEST(PathSegmentTest, getArcProgress3) {
  // Test close to full circle arc segments
  Eigen::Vector2d segment_start(1.0, 0.0);
  Eigen::Vector2d segment_start_tangent(0.0, -1.0);
  double end_theta = -1.75 * M_PI;
  Eigen::Vector2d segment_end(std::cos(end_theta), std::sin(end_theta));
  Eigen::Vector2d expected_center(0.0, 0.0);
  double theta;
  Eigen::Vector2d arc_center = PathSegment::getArcCenter(segment_start, segment_start_tangent, -1.0);
  ASSERT_TRUE((arc_center - expected_center).norm() < FLOAT_EPS);

  theta = PathSegment::getArcProgress(arc_center, segment_start, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  theta = PathSegment::getArcProgress(arc_center, segment_end, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);

  Eigen::Vector2d test_position(2.0, 0.0);
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  test_position << 0.0, 2.0;
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 6.0 / 7.0);

  test_position << 1.0, 1.0;
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);
}

TEST(PathSegmentTest, getArcProgress4) {
  // Test close to full circle arc segments
  Eigen::Vector2d segment_start(1.0, 0.0);
  Eigen::Vector2d segment_start_tangent(0.0, -1.0);
  double end_theta = -1.5 * M_PI;
  Eigen::Vector2d segment_end(std::cos(end_theta), std::sin(end_theta));
  Eigen::Vector2d expected_center(0.0, 0.0);
  double theta;
  Eigen::Vector2d arc_center = PathSegment::getArcCenter(segment_start, segment_start_tangent, -1.0);
  ASSERT_TRUE((arc_center - expected_center).norm() < FLOAT_EPS);

  theta = PathSegment::getArcProgress(arc_center, segment_start, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  theta = PathSegment::getArcProgress(arc_center, segment_end, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);

  Eigen::Vector2d test_position(2.0, 0.0);
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  test_position << 0.0, 2.0;
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);

  test_position << 1.0, 1.0;
  theta = PathSegment::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 1.1666666666666667);
}

TEST(PathSegmentTest, getLineProgress) {
  Eigen::Vector3d segment_start(0.0, 0.0, 0.0);
  Eigen::Vector3d segment_end(0.0, 0.0, 1.0);
  Eigen::Vector3d progress(0.0, 0.0, 0.5);

  double theta{0.0};
  theta = PathSegment::getLineProgress(progress, segment_start, segment_end);
  ASSERT_DOUBLE_EQ(theta, 0.5);

  theta = PathSegment::getLineProgress(segment_start, segment_start, segment_end);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  theta = PathSegment::getLineProgress(segment_end, segment_start, segment_end);
  ASSERT_DOUBLE_EQ(theta, 1.0);
}

TEST(PathSegmentSegmentTest, getClosestPoint) {
  Path trajectory;
  PathSegment first_arc_segment;

  State segment_start;
  segment_start.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  segment_start.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  State segment_middle;
  segment_middle.position = Eigen::Vector3d(1.0, 1.0, 0.0);
  segment_middle.velocity = Eigen::Vector3d(0.0, 1.0, 0.0);

  first_arc_segment.curvature = 1.0;
  first_arc_segment.states.push_back(segment_start);
  first_arc_segment.states.push_back(segment_middle);
  trajectory.appendSegment(first_arc_segment);

  PathSegment second_straight_segment;

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
  EXPECT_DOUBLE_EQ(curvature, 1.0);

  query_poistion = Eigen::Vector3d(1.0, 0.0, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d(std::sin(M_PI / 4.0), 1.0 - std::cos(M_PI / 4.0), 0.0)));
  Eigen::Vector3d expected_closest_point = Eigen::Vector3d(std::sin(M_PI / 4.0), 1.0 - std::cos(M_PI / 4.0), 0.0);
  ASSERT_TRUE((closest_point - expected_closest_point).norm() < FLOAT_EPS);
  Eigen::Vector3d expected_tangent = Eigen::Vector3d(1.0, 1.0, 0.0).normalized();
  ASSERT_TRUE(tangent.isApprox(expected_tangent));
  EXPECT_DOUBLE_EQ(curvature, 1.0);

  query_poistion = Eigen::Vector3d(2.0, 1.1, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d(1.0, 1.1, 0.0)));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0)));
  EXPECT_DOUBLE_EQ(curvature, 0.0);
}

TEST(PathSegmentSegmentTest2, getClosestPoint) {
  Path trajectory;
  PathSegment first_arc_segment;

  State segment_start;
  segment_start.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  segment_start.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  State segment_middle;
  segment_middle.position = Eigen::Vector3d(1.0, -1.0, 0.0);
  segment_middle.velocity = Eigen::Vector3d(0.0, -1.0, 0.0);

  first_arc_segment.curvature = -1.0;
  first_arc_segment.states.push_back(segment_start);
  first_arc_segment.states.push_back(segment_middle);
  trajectory.appendSegment(first_arc_segment);

  PathSegment second_straight_segment;

  State second_segment_end;
  second_segment_end.position = Eigen::Vector3d(1.0, -2.0, 0.0);
  second_segment_end.velocity = Eigen::Vector3d(0.0, -1.0, 0.0);

  second_straight_segment.curvature = 0.0;
  second_straight_segment.states.push_back(segment_middle);
  second_straight_segment.states.push_back(second_segment_end);
  trajectory.appendSegment(second_straight_segment);

  Eigen::Vector3d query_poistion;
  Eigen::Vector3d closest_point, tangent;
  double curvature;

  query_poistion = Eigen::Vector3d(0.0, 1.0, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d::Zero()));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(curvature, -1.0);

  query_poistion = Eigen::Vector3d(1.0, 0.0, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d(std::sin(M_PI / 4.0), -1.0 + std::cos(M_PI / 4.0), 0.0)));
  Eigen::Vector3d expected_closest_point = Eigen::Vector3d(std::sin(M_PI / 4.0), -1.0 + std::cos(M_PI / 4.0), 0.0);
  ASSERT_TRUE((closest_point - expected_closest_point).norm() < FLOAT_EPS);
  Eigen::Vector3d expected_tangent = Eigen::Vector3d(1.0, -1.0, 0.0).normalized();
  ASSERT_TRUE(tangent.isApprox(expected_tangent));
  EXPECT_DOUBLE_EQ(curvature, -1.0);

  query_poistion = Eigen::Vector3d(2.0, -1.1, 0.0);
  trajectory.getClosestPoint(query_poistion, closest_point, tangent, curvature);
  ASSERT_TRUE(closest_point.isApprox(Eigen::Vector3d(1.0, -1.1, 0.0)));
  ASSERT_TRUE(tangent.isApprox(Eigen::Vector3d(0.0, -1.0, 0.0)));
  EXPECT_DOUBLE_EQ(curvature, 0.0);
}

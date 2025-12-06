#include <gtest/gtest.h>
#include <math.h>
#include <cmath>
#include <vector>
#include "path_smoothing/path_smoother.hpp"

using namespace path_smoothing;

TEST(PathSmootherTest, TwoWaypoints) {
    std::vector<Point2D> waypoints = {Point2D(0.0, 0.0), Point2D(10.0, 0.0)};
    PathSmoother smoother(waypoints);
    Point2D start = smoother.evaluate(0.0);
    Point2D end = smoother.evaluate(1.0);
    EXPECT_NEAR(start.x, 0.0, 1e-6);
    EXPECT_NEAR(start.y, 0.0, 1e-6);
    EXPECT_NEAR(end.x, 10.0, 1e-6);
    EXPECT_NEAR(end.y, 0.0, 1e-6);
}

TEST(PathSmootherTest, CollinearPoints) {
  std::vector<Point2D> waypoints = {Point2D(0.0, 0.0), Point2D(5.0, 0.0), Point2D(10.0, 0.0)};
  PathSmoother smoother(waypoints);
  for (int i = 0; i <= 100; ++i) {
    Point2D p = smoother.evaluate(i / 100.0);
    EXPECT_NEAR(p.y, 0.0, 1e-6);
  }
}

TEST(PathSmootherTest, LShapedPath) {
  std::vector<Point2D> waypoints = {Point2D(0.0, 0.0), Point2D(10.0, 0.0), Point2D(10.0, 10.0)};
  PathSmoother smoother(waypoints);
  Point2D p0 = smoother.evaluate(0.0);
  Point2D p2 = smoother.evaluate(1.0);
  EXPECT_NEAR(p0.x, 0.0, 1e-6);
  EXPECT_NEAR(p0.y, 0.0, 1e-6);
  EXPECT_NEAR(p2.x, 10.0, 1e-6);
  EXPECT_NEAR(p2.y, 10.0, 1e-6);
}

TEST(PathSmootherTest, InvalidInput) {
  std::vector<Point2D> waypoints = {Point2D(0.0, 0.0)};
  EXPECT_THROW({path_smoothing::PathSmoother ps(waypoints);}, std::invalid_argument);
}

TEST(PathSmootherTest, CurveSampling) {
  std::vector<Point2D> waypoints = {Point2D(0.0, 0.0), Point2D(10.0, 0.0), Point2D(10.0, 10.0)};
  PathSmoother smoother(waypoints);
  std::vector<Point2D> samples = smoother.sampleCurve(50);
  EXPECT_EQ(samples.size(), 50);
  EXPECT_NEAR(samples[0].x, 0.0, 0.1);
  EXPECT_NEAR(samples[0].y, 0.0, 0.1);
  EXPECT_NEAR(samples[49].x, 10.0, 0.1);
  EXPECT_NEAR(samples[49].y, 10.0, 0.1);
}

TEST(PathSmootherTest, ParameterBounds) {
  std::vector<Point2D> waypoints = {Point2D(0.0, 0.0), Point2D(10.0, 0.0)};
  PathSmoother smoother(waypoints);
  EXPECT_THROW(smoother.evaluate(-0.1), std::out_of_range);
  EXPECT_THROW(smoother.evaluate(1.1), std::out_of_range);
  EXPECT_NO_THROW(smoother.evaluate(0.0));
  EXPECT_NO_THROW(smoother.evaluate(0.5));
  EXPECT_NO_THROW(smoother.evaluate(1.0));
}

TEST(PathSmootherTest, Continuity) {
  std::vector<Point2D> waypoints = {Point2D(0.0, 0.0), Point2D(5.0, 10.0), Point2D(15.0, 5.0), Point2D(20.0, 15.0)};
  PathSmoother smoother(waypoints);
  Point2D prev = smoother.evaluate(0.0);
  for (int i = 1; i <= 100; ++i) {
    Point2D curr = smoother.evaluate(i / 100.0);
    double dist = prev.distanceTo(curr);
    EXPECT_LT(dist, 1.0);
    prev = curr;
  }
}

TEST(PathSmootherTest, SecondDerivativeContinuity) {
  std::vector<Point2D> waypoints = {Point2D(0.0, 0.0), Point2D(10.0, 5.0), Point2D(20.0, 0.0)};
  PathSmoother smoother(waypoints);
  for (double p = 0.0; p <= 1.0; p += 0.05) {
    Point2D d2 = smoother.getSecondDerivative(p);
    EXPECT_TRUE(std::isfinite(d2.x));
    EXPECT_TRUE(std::isfinite(d2.y));
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
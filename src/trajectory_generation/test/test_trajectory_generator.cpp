#include <gtest/gtest.h>
#include <math.h>
#include <cmath>
#include <vector>
#include "trajectory_generation/trajectory_generator.hpp"

using namespace trajectory_generation;

TEST(TrajectoryGeneratorTest, TwoWaypoints3D) {
  std::vector<Point3D> waypoints = {
      Point3D(0.0, 0.0, 0.0),
      Point3D(10.0, 0.0, 5.0)
  };
  TrajectoryGenerator gen(waypoints, 1.0, 0.5);
  
  Point3D start = gen.getPosition(0.0);
  Point3D end = gen.getPosition(1.0);
  
  EXPECT_NEAR(start.x, 0.0, 1e-6);
  EXPECT_NEAR(start.y, 0.0, 1e-6);
  EXPECT_NEAR(start.z, 0.0, 1e-6);
  EXPECT_NEAR(end.x, 10.0, 1e-6);
  EXPECT_NEAR(end.y, 0.0, 1e-6);
  EXPECT_NEAR(end.z, 5.0, 1e-6);
}

TEST(TrajectoryGeneratorTest, InvalidInput) {
  std::vector<Point3D> waypoints = {Point3D(0.0, 0.0, 0.0)};
  EXPECT_THROW(TrajectoryGenerator(waypoints, 1.0, 0.5), std::invalid_argument);
}

TEST(TrajectoryGeneratorTest, NegativeVelocity) {
  std::vector<Point3D> waypoints = {
      Point3D(0.0, 0.0, 0.0),
      Point3D(10.0, 0.0, 0.0)
  };
  EXPECT_THROW(TrajectoryGenerator(waypoints, -1.0, 0.5), std::invalid_argument);
}

TEST(TrajectoryGeneratorTest, TrajectoryGeneration) {
  std::vector<Point3D> waypoints = {
      Point3D(0.0, 0.0, 0.0),
      Point3D(5.0, 5.0, 0.0),
      Point3D(10.0, 0.0, 0.0)
  };
  TrajectoryGenerator gen(waypoints, 1.0, 0.5);
  
  Trajectory traj = gen.generate(10.0, 50);
  
  EXPECT_EQ(traj.positions.size(), 50);
  EXPECT_EQ(traj.velocities.size(), 50);
  EXPECT_EQ(traj.accelerations.size(), 50);
  EXPECT_EQ(traj.times.size(), 50);
}

TEST(TrajectoryGeneratorTest, VelocityFinite) {
  std::vector<Point3D> waypoints = {
      Point3D(0.0, 0.0, 0.0),
      Point3D(10.0, 10.0, 0.0)
  };
  TrajectoryGenerator gen(waypoints, 2.0, 1.0);
  
  for (double t = 0.0; t <= 1.0; t += 0.1) {
    Point3D vel = gen.getVelocity(t);
    double speed = std::sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
    EXPECT_TRUE(std::isfinite(speed));
  }
}

TEST(TrajectoryGeneratorTest, ParameterBounds) {
  std::vector<Point3D> waypoints = {
      Point3D(0.0, 0.0, 0.0),
      Point3D(10.0, 0.0, 0.0)
  };
  TrajectoryGenerator gen(waypoints, 1.0, 0.5);
  
  EXPECT_THROW(gen.getPosition(-0.1), std::out_of_range);
  EXPECT_THROW(gen.getPosition(1.1), std::out_of_range);
  EXPECT_NO_THROW(gen.getPosition(0.0));
  EXPECT_NO_THROW(gen.getPosition(0.5));
  EXPECT_NO_THROW(gen.getPosition(1.0));
}

TEST(TrajectoryGeneratorTest, MultipleWaypoints) {
  std::vector<Point3D> waypoints = {
      Point3D(0.0, 0.0, 0.0),
      Point3D(5.0, 0.0, 0.0),
      Point3D(10.0, 5.0, 0.0),
      Point3D(10.0, 10.0, 5.0)
  };
  TrajectoryGenerator gen(waypoints, 1.0, 0.5);
  
  Point3D p0 = gen.getPosition(0.0);
  Point3D p1 = gen.getPosition(1.0);
  
  EXPECT_NEAR(p0.x, 0.0, 0.1);
  EXPECT_NEAR(p0.y, 0.0, 0.1);
  EXPECT_NEAR(p1.x, 10.0, 0.1);
  EXPECT_NEAR(p1.y, 10.0, 0.1);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
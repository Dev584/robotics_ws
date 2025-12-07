#include <gtest/gtest.h>
#include <cmath>
#include "trajectory_controller/pure_pursuit_controller.hpp"

using namespace trajectory_controller;

TEST(PurePursuitControllerTest, StraightLine) {
    PurePursuitController controller(1.0, 1.0, 1.0, 0.1);
    std::vector<Pose2D> trajectory = {
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(2.0, 0.0, 0.0),
        Pose2D(4.0, 0.0, 0.0),
        Pose2D(6.0, 0.0, 0.0)
    };

    ControlCommand cmd = controller.compute(Pose2D(0.0, 0.0, 0.0), trajectory, 0.1);

    EXPECT_NEAR(cmd.linear_velocity, 1.0, 0.1);
    EXPECT_NEAR(cmd.angular_velocity, 0.0, 0.1);
}

TEST(PurePursuitControllerTest, TurnRight) {
    PurePursuitController controller(1.0, 1.0, 1.0, 0.1);
    std::vector<Pose2D> trajectory = {
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 1.0, 0.7854),
        Pose2D(1.0, 2.0, 1.5708)
    };

    ControlCommand cmd = controller.compute(Pose2D(0.0, 0.0, 0.0), trajectory, 0.1);

    EXPECT_GT(cmd.angular_velocity, 0.0);
    EXPECT_LE(std::abs(cmd.angular_velocity), 1.0);
}

TEST(PurePursuitControllerTest, VelocityClipping) {
    PurePursuitController controller(1.0, 0.5, 1.0, 0.1);

    // Force high curvature to test clipping
    std::vector<Pose2D> trajectory = {
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.1, 1.0, 1.5708)
    };

    ControlCommand cmd = controller.compute(Pose2D(0.0, 0.0, 0.0), trajectory, 0.1);
  
    EXPECT_LE(cmd.linear_velocity, 0.5);
    EXPECT_GE(cmd.linear_velocity, 0.1);
}

TEST(PurePursuitControllerTest, EmptyTrajectory) {
    PurePursuitController controller(1.0, 1.0, 1.0, 0.1);
    std::vector<Pose2D> empty_trajectory;
    
    ControlCommand cmd = controller.compute(Pose2D(0.0, 0.0, 0.0), empty_trajectory, 0.1);
    
    EXPECT_EQ(cmd.linear_velocity, 0.0);
    EXPECT_EQ(cmd.angular_velocity, 0.0);
}

TEST(PurePursuitControllerTest, InvalidConstructor) {
    EXPECT_THROW(PurePursuitController(-1.0, 1.0, 1.0, 0.1), std::invalid_argument);
    EXPECT_THROW(PurePursuitController(1.0, -1.0, 1.0, 0.1), std::invalid_argument);
}

TEST(PurePursuitControllerTest, AtGoal) {
    PurePursuitController controller(1.0, 1.0, 1.0, 0.1);
    std::vector<Pose2D> trajectory = {Pose2D(1.0, 0.0, 0.0)};
    
    ControlCommand cmd = controller.compute(Pose2D(1.0, 0.0, 0.0), trajectory, 0.1);
    
    EXPECT_NEAR(cmd.linear_velocity, 0.1, 0.05);  // Minimum velocity
    EXPECT_NEAR(cmd.angular_velocity, 0.0, 0.1);
}

TEST(PurePursuitControllerTest, AngularClipping) {
    PurePursuitController controller(0.5, 1.0, 0.5, 0.1);
    std::vector<Pose2D> trajectory = {
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.0, 2.0, 1.5708)  // Sharp turn
    };
    
    ControlCommand cmd = controller.compute(Pose2D(0.0, 0.0, 0.0), trajectory, 0.1);
    
    EXPECT_LE(std::abs(cmd.angular_velocity), 0.5);
}

TEST(PurePursuitControllerTest, LookaheadSelection) {
    PurePursuitController controller(1.5, 1.0, 1.0, 0.1);
    std::vector<Pose2D> trajectory = {
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.5, 0.0, 0.0),
        Pose2D(1.5, 0.0, 0.0),
        Pose2D(2.5, 0.0, 0.0)
    };
    
    size_t idx = controller.findLookaheadPoint(Pose2D(0.0, 0.0, 0.0), trajectory);
    EXPECT_EQ(idx, 2);  // Should select point at 1.5m (index 2)
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
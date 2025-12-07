#ifndef TRAJECTORY_CONTROLLER_PURE_PURSUIT_CONTROLLER_HPP_
#define TRAJECTORY_CONTROLLER_PURE_PURSUIT_CONTROLLER_HPP_

#include <vector>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace trajectory_controller 
{
    struct Pose2D
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        
        Pose2D() = default;
        Pose2D(double x_val, double y_val, double yaw_val) : x(x_val), y(y_val), yaw(yaw_val) {}
    };

    struct ControlCommand 
    {
        double linear_velocity = 0.0;
        double angular_velocity = 0.0;
        
        ControlCommand() = default;
        ControlCommand(double linear, double angular) : linear_velocity(linear), angular_velocity(angular) {}
    };

    class PurePursuitController 
    {
        public:
            explicit PurePursuitController(double lookahead_distance = 1.0,
                                           double max_linear_velocity = 1.0,
                                           double max_angular_velocity = 1.0,
                                           double min_linear_velocity = 0.1);
            
            ControlCommand compute(const Pose2D& current_pose, const std::vector<Pose2D>& trajectory, double dt);
            
            double getLookaheadDistance() const { return lookahead_distance_; }
            double getMaxLinearVelocity() const { return max_linear_velocity_; }

            size_t findLookaheadPoint(const Pose2D& current_pose, const std::vector<Pose2D>& trajectory) const;
        
        private:
            double lookahead_distance_;
            double max_linear_velocity_;
            double max_angular_velocity_;
            double min_linear_velocity_;
            
            double computeCurvature(const Pose2D& current_pose, const Pose2D& lookahead_point) const;
            
            double clipVelocity(double velocity) const;
    };

}

#endif
#include "trajectory_controller/pure_pursuit_controller.hpp"
#include <algorithm>
#include <cmath>
#define _USE_MATH_DEFINES  // For M_PI

namespace trajectory_controller
{

    PurePursuitController::PurePursuitController(double lookahead_distance, double max_linear_velocity, double max_angular_velocity, double min_linear_velocity) : lookahead_distance_(lookahead_distance), max_linear_velocity_(max_linear_velocity), max_angular_velocity_(max_angular_velocity), min_linear_velocity_(min_linear_velocity) {
        if (lookahead_distance <= 0.0) {
            throw std::invalid_argument("Lookahead distance must be positive");
        }
        if (max_linear_velocity <= 0.0 || max_angular_velocity <= 0.0) {
            throw std::invalid_argument("Max velocities must be positive");
        }
        if (min_linear_velocity < 0.0 || min_linear_velocity >= max_linear_velocity) {
            throw std::invalid_argument("Invalid min_linear_velocity");
        }
    }

    size_t PurePursuitController::findLookaheadPoint(const Pose2D& current_pose, const std::vector<Pose2D>& trajectory) const {
        if (trajectory.empty()) {
            return 0;
        }
        
        for (size_t i = 0; i < trajectory.size(); ++i) {
            double dx = trajectory[i].x - current_pose.x;
            double dy = trajectory[i].y - current_pose.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance + 1e-6 >= lookahead_distance_) {
                return i;
            }
        }
        
        return trajectory.size() - 1;
    }

    double PurePursuitController::computeCurvature(const Pose2D& current_pose, const Pose2D& lookahead_point) const {
        double dx = lookahead_point.x - current_pose.x;
        double dy = lookahead_point.y - current_pose.y;
        
        double L = lookahead_distance_;
        double alpha = std::atan2(dy, dx) - current_pose.yaw;
        
        if (std::abs(alpha) > M_PI) {
            alpha = alpha > 0 ? alpha - 2*M_PI : alpha + 2*M_PI;
        }
        
        return std::tan(alpha) / L;
    }

    double PurePursuitController::clipVelocity(double velocity) const {
    return std::max(min_linear_velocity_, std::min(max_linear_velocity_, velocity));
    }

    ControlCommand PurePursuitController::compute(const Pose2D& current_pose, const std::vector<Pose2D>& trajectory, [[maybe_unused]] double dt) {
        if (trajectory.empty()) {
            return ControlCommand(0.0, 0.0);
        }
        
        size_t lookahead_idx = findLookaheadPoint(current_pose, trajectory);
        if (lookahead_idx >= trajectory.size()) {
            return ControlCommand(0.0, 0.0);
        }
        
        const Pose2D& lookahead_point = trajectory[lookahead_idx];

        double dx = lookahead_point.x - current_pose.x;
        double dy = lookahead_point.y - current_pose.y;
        double distance_to_lookahead = std::sqrt(dx*dx + dy*dy);
        
        if (distance_to_lookahead < 0.1) {
            return ControlCommand(min_linear_velocity_, 0.0);
        }
        
        double curvature = computeCurvature(current_pose, lookahead_point);
        double linear_velocity = clipVelocity(max_linear_velocity_ / (1.0 + std::abs(curvature)));
        double angular_velocity = curvature * linear_velocity;
        
        angular_velocity = std::min(max_angular_velocity_, std::max(-max_angular_velocity_, angular_velocity));
        
        return ControlCommand(linear_velocity, angular_velocity);
    }

}
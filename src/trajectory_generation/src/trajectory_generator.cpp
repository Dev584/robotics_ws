#include "trajectory_generation/trajectory_generator.hpp"
#include <algorithm>
#include <iostream>

namespace trajectory_generation {

    TrajectoryGenerator::TrajectoryGenerator(const std::vector<Point3D>& waypoints, double max_velocity, double max_acceleration) : waypoints_(waypoints), max_velocity_(max_velocity), max_acceleration_(max_acceleration) {
        if (waypoints.size() < 2) {
            throw std::invalid_argument("Need at least 2 waypoints");
        }
        if (max_velocity <= 0.0 || max_acceleration <= 0.0) {
            throw std::invalid_argument("Velocity and acceleration must be positive");
        }
    }

    std::vector<double> TrajectoryGenerator::computeSegmentTimes() {
        std::vector<double> segment_times;
        double cumulative_time = 0.0;
        segment_times.push_back(0.0);
        
        for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
            double segment_time = computeMinimumTime(waypoints_[i], waypoints_[i + 1]);
            cumulative_time += segment_time;
            segment_times.push_back(cumulative_time);
        }
        
        total_time_ = cumulative_time;
        return segment_times;
    }

    double TrajectoryGenerator::computeMinimumTime(const Point3D& start, const Point3D& end) {
        double distance = start.distanceTo(end);
        
        // Time under triangular velocity profile
        double t_accel = max_velocity_ / max_acceleration_;
        double distance_accel_decel = max_acceleration_ * t_accel * t_accel;
        
        if (2.0 * distance_accel_decel >= distance) {
            // Acceleration limited: v_max not reached
            return 2.0 * std::sqrt(distance / max_acceleration_);
        } else {
            // Velocity limited: v_max is reached
            double t_coast = (distance - distance_accel_decel) / max_velocity_;
            return 2.0 * t_accel + t_coast;
        }
    }

    Trajectory TrajectoryGenerator::generate(double total_time, size_t num_samples) {
        Trajectory trajectory;
        
        if (num_samples < 2) {
            throw std::invalid_argument("Need at least 2 samples");
        }
        
        std::vector<double> segment_times = computeSegmentTimes();
        
        for (size_t i = 0; i < num_samples; ++i) {
            double t = (num_samples > 1) 
                        ? total_time * static_cast<double>(i) / (num_samples - 1)
                        : 0.0;
            
            trajectory.times.push_back(t);
            trajectory.positions.push_back(getPosition(t / total_time_));
            trajectory.velocities.push_back(getVelocity(t / total_time_));
            trajectory.accelerations.push_back(getAcceleration(t / total_time_));
        }
        
        return trajectory;
    }

    Point3D TrajectoryGenerator::getPosition(double t) const {
        if (t < 0.0 || t > 1.0) {
            throw std::out_of_range("Time parameter must be in [0, 1]");
        }
        
        if (waypoints_.size() < 2) return Point3D(0, 0, 0);
        
        size_t n_segments = waypoints_.size() - 1;
        double segment_t = t * n_segments;
        size_t segment_idx = static_cast<size_t>(segment_t);
        
        if (segment_idx >= n_segments) {
            segment_idx = n_segments - 1;
        }
        
        double local_t = segment_t - segment_idx;
        
        const Point3D& p0 = waypoints_[segment_idx];
        const Point3D& p1 = waypoints_[segment_idx + 1];
        
        // Linear interpolation between waypoints
        return p0 + (p1 + p0 * (-1.0)) * local_t;
    }

    Point3D TrajectoryGenerator::getVelocity(double t) const {
        if (t < 0.0 || t > 1.0) {
            throw std::out_of_range("Time parameter must be in [0, 1]");
        }
        
        double dt = 0.001;
        Point3D pos_forward = getPosition(std::min(1.0, t + dt));
        Point3D pos_backward = getPosition(std::max(0.0, t - dt));
        
        return (pos_forward + pos_backward * (-1.0)) * (1.0 / (2.0 * dt));
    }

    Point3D TrajectoryGenerator::getAcceleration(double t) const {
        if (t < 0.0 || t > 1.0) {
            throw std::out_of_range("Time parameter must be in [0, 1]");
        }
        
        double dt = 0.001;
        Point3D vel_forward = getVelocity(std::min(1.0, t + dt));
        Point3D vel_backward = getVelocity(std::max(0.0, t - dt));
        
        return (vel_forward + vel_backward * (-1.0)) * (1.0 / (2.0 * dt));
    }

}
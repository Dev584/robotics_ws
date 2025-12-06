#ifndef TRAJECTORY_GENERATION_TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATION_TRAJECTORY_GENERATOR_HPP_

#include <vector>
#include <cmath>
#include <stdexcept>

namespace trajectory_generation {

    struct Point3D {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        
        Point3D() = default;
        Point3D(double x_val, double y_val, double z_val) 
            : x(x_val), y(y_val), z(z_val) {}
        
        double distanceTo(const Point3D& other) const {
            double dx = x - other.x;
            double dy = y - other.y;
            double dz = z - other.z;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        
        Point3D operator+(const Point3D& other) const {
            return Point3D(x + other.x, y + other.y, z + other.z);
        }
        
        Point3D operator*(double scale) const {
            return Point3D(x * scale, y * scale, z * scale);
        }
    };

    struct Trajectory {
        std::vector<Point3D> positions;
        std::vector<Point3D> velocities;
        std::vector<Point3D> accelerations;
        std::vector<double> times;
    };

    class TrajectoryGenerator {
        public:
            explicit TrajectoryGenerator(const std::vector<Point3D>& waypoints,
                                        double max_velocity = 1.0,
                                        double max_acceleration = 0.5);
            
            Trajectory generate(double total_time, size_t num_samples);
            Point3D getPosition(double t) const;
            Point3D getVelocity(double t) const;
            Point3D getAcceleration(double t) const;
            double getTotalTime() const { return total_time_; }
        
        private:
            std::vector<Point3D> waypoints_;
            double max_velocity_;
            double max_acceleration_;
            double total_time_ = 0.0;
            
            std::vector<double> computeSegmentTimes();
            double computeMinimumTime(const Point3D& start, const Point3D& end);
    };

}

#endif
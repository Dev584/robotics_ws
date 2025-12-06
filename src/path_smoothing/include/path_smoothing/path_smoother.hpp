#ifndef PATH_SMOOTHING_PATH_SMOOTHER_HPP_
#define PATH_SMOOTHING_PATH_SMOOTHER_HPP_

#include <vector>
#include <math.h>
#include <cmath>
#include <stdexcept>

namespace path_smoothing {
    struct Point2D {
        double x = 0.0;
        double y = 0.0;
        Point2D() = default;
        Point2D(double x_val, double y_val) : x(x_val), y(y_val) {}
        double distanceTo(const Point2D& other) const {
            double dx = x - other.x;
            double dy = y - other.y;
            return std::sqrt(dx * dx + dy * dy);
        }
    };


    class PathSmoother {
        public:
            explicit PathSmoother(const std::vector<Point2D>& waypoints);
            Point2D evaluate(double parameter) const;
            Point2D getDerivative(double parameter) const;
            Point2D getSecondDerivative(double parameter) const;
            double getArcLength() const { return arc_length_; }
            size_t getNumWaypoints() const { return waypoints_.size(); }
            std::vector<Point2D> sampleCurve(size_t num_samples) const;

        private:
            struct CubicCoefficients {
                double a = 0.0, b = 0.0, c = 0.0, d = 0.0;
                double evaluate(double t) const {
                    return a + b * t + c * t * t + d * t * t * t;
                }
                double getDerivative(double t) const {
                    return b + 2.0 * c * t + 3.0 * d * t * t;
                }
                double getSecondDerivative(double t) const {
                    return 2.0 * c + 6.0 * d * t;
                }
            };

            std::vector<Point2D> waypoints_;
            std::vector<CubicCoefficients> spline_x_;
            std::vector<CubicCoefficients> spline_y_;
            double arc_length_ = 0.0;

            std::vector<CubicCoefficients> computeNaturalSpline(const std::vector<double>& points) const;
            std::vector<double> solveTridiagonal(const std::vector<double>& a,
                                                const std::vector<double>& b,
                                                const std::vector<double>& c,
                                                const std::vector<double>& d) const;
            void computeArcLength();
    };
}

#endif

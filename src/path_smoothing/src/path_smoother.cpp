#include "path_smoothing/path_smoother.hpp"
#include <algorithm>

namespace path_smoothing {
    PathSmoother::PathSmoother(const std::vector<Point2D>& waypoints) : waypoints_(waypoints) {
        if (waypoints.size() < 2) throw std::invalid_argument("Need at least 2 waypoints");
        std::vector<double> x_coords, y_coords;
        for (const auto& point : waypoints_) {
            x_coords.push_back(point.x);
            y_coords.push_back(point.y);
        }
        spline_x_ = computeNaturalSpline(x_coords);
        spline_y_ = computeNaturalSpline(y_coords);
        computeArcLength();
    }
        
    std::vector<PathSmoother::CubicCoefficients> PathSmoother::computeNaturalSpline(const std::vector<double>& points) const {
        size_t n = points.size() - 1;
        std::vector<CubicCoefficients> coefficients(n);
        std::vector<double> h(n);
        for (size_t i = 0; i < n; ++i) h[i] = 1.0;

        std::vector<double> alpha(n + 1, 0.0);
        for (size_t i = 1; i < n; ++i) {
            alpha[i] = (3.0 / h[i]) * (points[i + 1] - points[i]) - (3.0 / h[i - 1]) * (points[i] - points[i - 1]);
        }

    std::vector<double> a(n + 1), b(n + 1), c(n + 1), d(n + 1);
    b[0] = 1.0; c[0] = 0.0; d[0] = 0.0;

    for (size_t i = 1; i < n; ++i) {
        a[i] = h[i - 1];
        b[i] = 2.0 * (h[i - 1] + h[i]);
        c[i] = h[i];
        d[i] = alpha[i];
    }
    a[n] = 0.0; b[n] = 1.0; d[n] = 0.0;

    std::vector<double> second_derivatives = solveTridiagonal(a, b, c, d);

    for (size_t i = 0; i < n; ++i) {
        double hi = h[i];
        coefficients[i].a = points[i];
        coefficients[i].b = (points[i + 1] - points[i]) / hi - hi * (2.0 * second_derivatives[i] +second_derivatives[i + 1]) / 6.0;
        coefficients[i].c = second_derivatives[i] / 2.0;
        coefficients[i].d = (second_derivatives[i + 1] - second_derivatives[i]) / (6.0 * hi);
    }
    return coefficients;
    }

    std::vector<double> PathSmoother::solveTridiagonal(const std::vector<double>& a,
                                                       const std::vector<double>& b,
                                                       const std::vector<double>& c,
                                                       const std::vector<double>& d) const {
        size_t n = b.size();
        std::vector<double> solution(n);
        std::vector<double> c_prime(n), d_prime(n);

        c_prime[0] = c[0] / b[0];
        d_prime[0] = d[0] / b[0];

        for (size_t i = 1; i < n; ++i) {
            double denom = b[i] - a[i] * c_prime[i - 1];
            if (std::abs(denom) < 1e-15) throw std::runtime_error("Singular matrix");
            c_prime[i] = c[i] / denom;
            d_prime[i] = (d[i] - a[i] * d_prime[i - 1]) / denom;
        }

        solution[n - 1] = d_prime[n - 1];
        for (int i = n - 2; i >= 0; --i) {
            solution[i] = d_prime[i] - c_prime[i] * solution[i + 1];
        }
        return solution;
    }

    Point2D PathSmoother::evaluate(double parameter) const {
        if (parameter < 0.0 || parameter > 1.0) throw std::out_of_range("Parameter must be in [0, 1]");
        size_t n_segments = spline_x_.size();
        double segment_param = parameter * n_segments;
        size_t segement_idx = static_cast<size_t>(segment_param);
        if (segement_idx >= n_segments) segement_idx = n_segments - 1;
        double local_param = segment_param - segement_idx;
        double x = spline_x_[segement_idx].evaluate(local_param);
        double y = spline_y_[segement_idx].evaluate(local_param);
        return Point2D(x, y);
    }

    Point2D PathSmoother::getDerivative(double parameter) const {
        if (parameter < 0.0 || parameter > 1.0) throw std::out_of_range("Parameter must be in [0, 1]");
        size_t n_segments = spline_x_.size();
        double segment_param = parameter * n_segments;
        size_t segement_idx = static_cast<size_t>(segment_param);
        if (segement_idx >= n_segments) segement_idx = n_segments - 1;
        double local_param = segment_param - segement_idx;
        double dx_dlocal = spline_x_[segement_idx].getDerivative(local_param);
        double dy_dlocal = spline_y_[segement_idx].getDerivative(local_param);
        double scale = n_segments;
        return Point2D(dx_dlocal * scale, dy_dlocal * scale);
    }

    Point2D PathSmoother::getSecondDerivative(double parameter) const {
        if (parameter < 0.0 || parameter > 1.0) throw std::out_of_range("Parameter must be in [0, 1]");
        size_t n_segments = spline_x_.size();
        double segment_param = parameter * n_segments;
        size_t segement_idx = static_cast<size_t>(segment_param);
        if (segement_idx >= n_segments) segement_idx = n_segments - 1;
        double local_param = segment_param - segement_idx;
        double d2x_dlocal = spline_x_[segement_idx].getSecondDerivative(local_param);
        double d2y_dlocal = spline_y_[segement_idx].getSecondDerivative(local_param);
        double scale = n_segments * n_segments;
        return Point2D(d2x_dlocal * scale, d2y_dlocal * scale);
    }

    std::vector<Point2D> PathSmoother::sampleCurve(size_t num_samples) const{
        std::vector<Point2D> samples;
        for (size_t i = 0; i < num_samples; ++i) {
            double parameter = (num_samples > 1) ? static_cast<double>(i) / (num_samples - 1) : 0.5;
            samples.push_back(evaluate(parameter));
        }
        return samples;
    }

    void PathSmoother::computeArcLength() {
        const size_t num_samples = 100;
        arc_length_ = 0.0;
        Point2D prev_point = evaluate(0.0);
        for (size_t i = 1; i <= num_samples; ++i) {
            double parameter = static_cast<double>(i) / num_samples;
            Point2D current_point = evaluate(parameter);
            arc_length_ += prev_point.distanceTo(current_point);
            prev_point = current_point;
        }
    }
}
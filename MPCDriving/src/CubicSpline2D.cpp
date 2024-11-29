#include "mpc-driving/math/CubicSpline2D.hpp"
#include <algorithm>
#include <cmath>

CubicSpline2D::CubicSpline2D(const std::vector<double>& x_points,
                             const std::vector<double>& y_points) {
    cumulative_distance_ = calculateCumulativeDistance(x_points, y_points);
    spline_x = CubicSpline(cumulative_distance_, x_points);
    spline_y = CubicSpline(cumulative_distance_, y_points);
}

std::vector<double> CubicSpline2D::calculateCumulativeDistance(
    const std::vector<double>& x_points, const std::vector<double>& y_points) {
    std::vector<double> segment_lengths;
    for (size_t i = 0; i < x_points.size() - 1; i++) {
        segment_lengths.push_back(std::sqrt(std::pow(x_points[i + 1] - x_points[i], 2) +
                                            std::pow(y_points[i + 1] - y_points[i], 2)));
    }
    std::vector<double> cumulative_distance = {0};
    for (size_t i = 0; i < segment_lengths.size(); i++) {
        cumulative_distance.push_back(cumulative_distance.back() + segment_lengths[i]);
    }
    return cumulative_distance;
}

std::pair<double, double> CubicSpline2D::calculatePosition(double s) {
    double x_position = spline_x.calculatePosition(s);
    double y_position = spline_y.calculatePosition(s);
    return {x_position, y_position};
}

double CubicSpline2D::calculateCurvature(double s) {
    double dx = spline_x.calculateFirstDerivative(s);
    double ddx = spline_x.calculateSecondDerivative(s);
    double dy = spline_y.calculateFirstDerivative(s);
    double ddy = spline_y.calculateSecondDerivative(s);
    return (ddy * dx - ddx * dy) / std::pow(dx, 2) + std::pow(dy, 2);
}

double CubicSpline2D::calculateYaw(double s) {
    double dx = spline_x.calculateFirstDerivative(s);
    double dy = spline_y.calculateFirstDerivative(s);
    return std::atan2(dy, dx);
}

std::vector<double> CubicSpline2D::getCumulativeDistance() const { return cumulative_distance_; }
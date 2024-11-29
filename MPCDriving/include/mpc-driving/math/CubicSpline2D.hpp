#ifndef CUBIC_SPLINE_2D_HPP
#define CUBIC_SPLINE_2D_HPP

#include "mpc-driving/math/CubicSpline.hpp"

#include <vector>

class CubicSpline2D {
public:
    CubicSpline2D() = delete;
    CubicSpline2D(const std::vector<double>& x_points, const std::vector<double>& y_points);
    std::pair<double, double> calculatePosition(double s);
    double calculateCurvature(double s);
    double calculateYaw(double s);
    std::vector<double> getCumulativeDistance() const;

private:
    CubicSpline spline_x, spline_y;
    std::vector<double> cumulative_distance_;
    std::vector<double> segment_lengths;

    std::vector<double> calculateCumulativeDistance(const std::vector<double>& x_points,
                                                    const std::vector<double>& y_points);
};

#endif  // CUBIC_SPLINE_2D_HPP

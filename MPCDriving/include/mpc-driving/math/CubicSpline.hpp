#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include <Eigen/Dense>
#include <vector>

class CubicSpline {
public:
    CubicSpline() = default;
    CubicSpline(const std::vector<double>& x_points, const std::vector<double>& y_points);

    double calculatePosition(double t);
    double calculateFirstDerivative(double t);
    double calculateSecondDerivative(double t);

private:
    std::vector<double> x_points_, y_points_, a_coeffs_, b_coeffs_, d_coeffs_;
    Eigen::VectorXd c_coeffs_;
    size_t num_points_;

    size_t findSegmentIndex(double t);
    Eigen::MatrixXd buildMatrixA(const std::vector<double>& segment_lengths);
    Eigen::VectorXd buildVectorB(const std::vector<double>& segment_lengths);
};

#endif  // CUBIC_SPLINE_HPP

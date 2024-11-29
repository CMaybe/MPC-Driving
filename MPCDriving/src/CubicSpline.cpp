#include "mpc-driving/math/CubicSpline.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>

CubicSpline::CubicSpline(const std::vector<double>& x_points, const std::vector<double>& y_points)
    : x_points_(x_points), y_points_(y_points), num_points_(x_points.size()) {
    assert(num_points_ == y_points.size());

    a_coeffs_ = y_points;

    std::vector<double> segment_lengths(num_points_ - 1);
    for (size_t i = 0; i < num_points_ - 1; i++) {
        segment_lengths[i] = x_points[i + 1] - x_points[i];
    }

    Eigen::MatrixXd A = buildMatrixA(segment_lengths);
    Eigen::VectorXd B = buildVectorB(segment_lengths);
    c_coeffs_ = A.lu().solve(B);

    for (size_t i = 0; i < num_points_ - 1; i++) {
        d_coeffs_.push_back((c_coeffs_[i + 1] - c_coeffs_[i]) / (3.0 * segment_lengths[i]));
        b_coeffs_.push_back((a_coeffs_[i + 1] - a_coeffs_[i]) / segment_lengths[i] -
                            segment_lengths[i] * (c_coeffs_[i + 1] + 2.0 * c_coeffs_[i]) / 3.0);
    }
}

double CubicSpline::calculatePosition(double t) {
    if (t < x_points_[0] || t > x_points_[num_points_ - 1]) {
        return NAN;
    }

    size_t i = findSegmentIndex(t);
    double delta_x = t - x_points_[i];

    return a_coeffs_[i] + b_coeffs_[i] * delta_x + c_coeffs_[i] * std::pow(delta_x, 2) +
           d_coeffs_[i] * std::pow(delta_x, 3);
}

double CubicSpline::calculateFirstDerivative(double t) {
    if (t < x_points_[0] || t > x_points_[num_points_ - 1]) {
        return NAN;
    }

    size_t i = findSegmentIndex(t);
    double delta_x = t - x_points_[i];

    return b_coeffs_[i] + 2.0 * c_coeffs_[i] * delta_x + 3.0 * d_coeffs_[i] * std::pow(delta_x, 2);
}

double CubicSpline::calculateSecondDerivative(double t) {
    if (t < x_points_[0] || t > x_points_[num_points_ - 1]) {
        return NAN;
    }

    size_t i = findSegmentIndex(t);
    double delta_x = t - x_points_[i];

    return 2.0 * c_coeffs_[i] + 6.0 * d_coeffs_[i] * delta_x;
}

size_t CubicSpline::findSegmentIndex(double t) {
    for (int i = 0; i < x_points_.size(); i++) {
        std::cout << x_points_[i] << '\t';
    }
    std::cout << std::endl;
    auto it = std::upper_bound(x_points_.begin(), x_points_.end(), t);
    return std::distance(x_points_.begin(), it) - 1;
}

Eigen::MatrixXd CubicSpline::buildMatrixA(const std::vector<double>& segment_lengths) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_points_, num_points_);
    A(0, 0) = 1.0;

    for (size_t i = 0; i < num_points_ - 1; i++) {
        if (i != (num_points_ - 2)) {
            A(i + 1, i + 1) = 2.0 * (segment_lengths[i] + segment_lengths[i + 1]);
        }
        A(i + 1, i) = segment_lengths[i];
        A(i, i + 1) = segment_lengths[i];
    }

    A(0, 1) = 0.0;
    A(num_points_ - 1, num_points_ - 2) = 0.0;
    A(num_points_ - 1, num_points_ - 1) = 1.0;

    return A;
}

Eigen::VectorXd CubicSpline::buildVectorB(const std::vector<double>& segment_lengths) {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(num_points_);
    for (size_t i = 0; i < num_points_ - 2; i++) {
        B[i + 1] = 3.0 * (a_coeffs_[i + 2] - a_coeffs_[i + 1]) / segment_lengths[i + 1] -
                   3.0 * (a_coeffs_[i + 1] - a_coeffs_[i]) / segment_lengths[i];
    }
    return B;
}

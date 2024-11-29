#include "mpc-driving/math/spline.hpp"

SplineFunction::SplineFunction(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec)
    : x_min_(x_vec.minCoeff())
    , x_max_(x_vec.maxCoeff())
    , spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
          y_vec.transpose(), std::min<int>(x_vec.rows() - 1, 3), scaled_values(x_vec))) {}

double SplineFunction::operator()(double x) const { return spline_(scaled_value(x))(0); }

double SplineFunction::scaled_value(double x) const { return (x - x_min_) / (x_max_ - x_min_); }

Eigen::RowVectorXd SplineFunction::scaled_values(Eigen::VectorXd const &x_vec) const {
    return x_vec.unaryExpr([this](double x) { return scaled_value(x); }).transpose();
}

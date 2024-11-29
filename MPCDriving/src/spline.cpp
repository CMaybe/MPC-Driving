#include "mpc-driving/math/spline.hpp"

Spline2D::Spline2D(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec)
    : x_min_(x_vec.minCoeff())
    , x_max_(x_vec.maxCoeff())
    , spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
          y_vec.transpose(), std::min<int>(x_vec.rows() - 1, 3), scaled_values(x_vec))) {}

double Spline2D::operator()(double const &x) const { return spline_(scaled_value(x))(0); }
double Spline2D::scaled_value(double x) const { return (x - x_min_) / (x_max_ - x_min_); }

Eigen::VectorXd Spline2D::operator()(Eigen::VectorXd const &x_vec) const {
    return x_vec.unaryExpr([this](double x) { return spline_(scaled_value(x))(0); });
}

std::vector<double> Spline2D::operator()(std::vector<double> const &x_vec) {
    Eigen::VectorXd eigenVec =
        Eigen::Map<Eigen::VectorXd>(const_cast<double *>(x_vec.data()), x_vec.size());
    eigenVec = eigenVec.unaryExpr([this](double x) { return spline_(scaled_value(x))(0); });
    return std::vector<double>(eigenVec.data(), eigenVec.data() + eigenVec.size());
}

Eigen::VectorXd Spline2D::scaled_values(Eigen::VectorXd const &x_vec) const {
    return x_vec.unaryExpr([this](double x) { return scaled_value(x); });
}

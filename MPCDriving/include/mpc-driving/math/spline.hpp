#ifndef SPLINE_HPP
#define SPLINE_HPP

#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <vector>

class Spline2D {
public:
    Spline2D() = delete;
    Spline2D(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec);

    double operator()(double const &x) const;
    Eigen::VectorXd operator()(Eigen::VectorXd const &x_vec) const;
    std::vector<double> operator()(std::vector<double> const &x_vec);
    Eigen::VectorXd getCurvature();

private:
    // Helpers to scale X values down to [0, 1]
    double scaled_value(double x) const;
    Eigen::VectorXd scaled_values(Eigen::VectorXd const &x_vec) const;

    double x_min_;
    double x_max_;

    // Spline of one-dimensional "points."
    Eigen::Spline<double, 1> spline_;
    Eigen::VectorXd spline_x_;
    Eigen::VectorXd spline_y_;
};
#endif

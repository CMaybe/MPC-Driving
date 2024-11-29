#ifndef SPLINE_HPP
#define SPLINE_HPP

#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

class SplineFunction {
public:
    SplineFunction() = delete;
    SplineFunction(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec);

    double operator()(double x) const;

private:
    // Helpers to scale X values down to [0, 1]
    double scaled_value(double x) const;
    Eigen::RowVectorXd scaled_values(Eigen::VectorXd const &x_vec) const;

    double x_min_;
    double x_max_;

    // Spline of one-dimensional "points."
    Eigen::Spline<double, 1> spline_;
};
#endif

#define _USE_MATH_DEFINES

#include "mpc-driving/math/spline.hpp"

#include <matplotlibcpp.h>
#include <iostream>
#include <string>
#include <vector>
namespace plt = matplotlibcpp;

int main() {
    Eigen::VectorXd xvals(5);
    Eigen::VectorXd yvals(xvals.rows());

    xvals << 0.1, 0.4, 1.2, 1.8, 2.0;
    yvals << 0.1, 0.7, 0.6, 1.1, 0.9;

    std::vector<double> rx, ry;

    SplineFunction s(xvals, yvals);
    double dt = 0.02;
    for (double x = 0; x < 2.0; x += dt) {
        rx.push_back(x);
        ry.push_back(s(x));
    }

    plt::plot(rx, ry, "-r");
    plt::grid(true);
    plt::xlabel("X Coordinate");
    plt::ylabel("Y Coordinate");
    plt::title("2D Spline Curve");

    plt::show();

    return 0;
}
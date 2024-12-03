#define _USE_MATH_DEFINES

#include "mpc/math/CubicSpline2D.hpp"

#include <Eigen/Dense>

#include <matplotlibcpp.h>
#include <iostream>
#include <string>
#include <vector>
namespace plt = matplotlibcpp;

int main() {
    std::vector<double> x = {0.0, 10.0, 20.0, 25.0, 30.0, 40.0};
    std::vector<double> y = {0.0, 5.0, 0.0, 0.0, 0.0, 0.0};
    CubicSpline2D sp(x, y);

    std::vector<double> path_x, path_y, path_yaw;
    std::vector<double> s;
    for (double t = 0; t < sp.getCumulativeDistance().back(); t += 1) {
        s.push_back(t);
    }

    for (double i_s : s) {
        auto [tx, ty] = sp.calculatePosition(i_s);
        path_x.push_back(tx);
        path_y.push_back(ty);
        path_yaw.push_back(sp.calculateYaw(i_s));
    }
    // plt::subplot(1);
    plt::figure();
    plt::plot(x, y, "xb");
    plt::plot(path_x, path_y, "-r");
    plt::figure();
    plt::plot(s, path_yaw, "-r");

    plt::show();

    return 0;
}
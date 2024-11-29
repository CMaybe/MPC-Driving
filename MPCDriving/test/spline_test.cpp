#define _USE_MATH_DEFINES

#include "mpc-driving/math/CubicSpline2D.hpp"

#include <Eigen/Dense>

#include <matplotlibcpp.h>
#include <iostream>
#include <string>
#include <vector>
namespace plt = matplotlibcpp;

int main() {
    std::vector<double> x = {-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    std::vector<double> y = {0.7, -6, 5, 6.5, 0.0, 5.0, -2.0};
    CubicSpline2D sp(x, y);

    std::vector<double> rx, ry, ryaw, rk;
    std::vector<double> s;
    for (double t = -0.2; t < sp.getCumulativeDistance().back(); t += 0.1) {
        s.push_back(t);
    }

    for (double i_s : s) {
        auto [tx, ty] = sp.calculatePosition(i_s);
        rx.push_back(tx);
        ry.push_back(ty);
        ryaw.push_back(sp.calculateYaw(i_s));
        rk.push_back(sp.calculateCurvature(i_s));
    }
    // plt::subplot(1);
    plt::figure();
    plt::plot(x, y, "xb");
    plt::plot(rx, ry, "-r");
    plt::figure();
    plt::plot(s, ryaw, "-r");
    plt::figure();
    plt::plot(s, rk, "-r");

    plt::show();

    return 0;
}
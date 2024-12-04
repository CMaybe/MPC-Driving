#define _USE_MATH_DEFINES

#include "mpc/MPC.hpp"
#include "mpc/math/CubicSpline2D.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <matplotlibcpp.h>
#include <vector>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;
namespace plt = matplotlibcpp;

int main() {
    std::vector<double> x = {0.0, 10.0, 20.0, 25.0, 30.0, 40.0};
    std::vector<double> y = {0.0, 5.0, 0.0, 0.0, 0.0, 0.0};
    CubicSpline2D sp(x, y);

    double sx = 0.0;
    double sy = -5;
    ;

    double dt = 0.1;
    double wheel_base = 3.5;
    size_t prediction_horizon = 20;

    std::vector<double> s;
    std::vector<double> x_ref, y_ref, yaw_ref, velocity_ref;

    for (double t = 0; t < sp.getCumulativeDistance().back(); t += dt) {
        s.push_back(t);
    }

    for (double i_s : s) {
        auto [tx, ty] = sp.calculatePosition(i_s);
        x_ref.push_back(tx);
        y_ref.push_back(ty);
        yaw_ref.push_back(sp.calculateYaw(i_s));
        velocity_ref.push_back(5.0);
    }
    SystemModel system(dt);
    system.setState(sx, sy, yaw_ref.front(), 0.0);
    MPC mpc(system, prediction_horizon, dt);
    Eigen::Vector2d input;

    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> yaw;

    path_x.push_back(sx);
    path_y.push_back(sy);

    plt::figure();
    plt::plot(x_ref, y_ref, "-r");

    while (x_ref.size() >= prediction_horizon + 1) {
        mpc.run(x_ref, y_ref, yaw_ref, velocity_ref);
        Eigen::Vector4d output = mpc.getOutput();
        path_x.push_back(output[0]);
        path_y.push_back(output[1]);
        x_ref.erase(x_ref.begin());
        y_ref.erase(y_ref.begin());
        yaw_ref.erase(yaw_ref.begin());
        velocity_ref.erase(velocity_ref.begin());
    }
    plt::plot(path_x, path_y, "xb");
    plt::show();

    return 0;
}
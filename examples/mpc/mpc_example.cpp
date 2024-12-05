#define _USE_MATH_DEFINES

#include "mpc/MPC.hpp"
#include "mpc/Utils.hpp"
#include "mpc/math/CubicSpline2D.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <matplotlibcpp.h>
#include <vector>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;
namespace plt = matplotlibcpp;

void plot_car(double x, double y, double yaw) {
    const double length = 3.5;
    const double width = 2.0;
    const double wheel_diameter = 1.0;
    const double wheel_width = 0.5;

    Eigen::Matrix2d rotate;
    rotate << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);

    Eigen::MatrixXd car(2, 5);
    car.row(0) << -length / 2, -length / 2, length / 2, length / 2, -length / 2;
    car.row(1) << width / 2, -width / 2, -width / 2, width / 2, width / 2;
    car = rotate * car;
    car.row(0).array() += x;
    car.row(1).array() += y;

    std::vector<double> car_x(car.cols());
    std::vector<double> car_y(car.cols());

    for (int i = 0; i < car.cols(); ++i) {
        car_x[i] = car(0, i);
        car_y[i] = car(1, i);
    }

    plt::plot(car_x, car_y, "-k");

    Eigen::MatrixXd rear_wheel(2, 5), front_wheel;
    rear_wheel.row(0) << -wheel_diameter / 2, -wheel_diameter / 2, wheel_diameter / 2,
        wheel_diameter / 2, -wheel_diameter / 2;
    rear_wheel.row(1) << wheel_width / 2, -wheel_width / 2, -wheel_width / 2, wheel_width / 2,
        wheel_width / 2;
    front_wheel = rear_wheel;

    rear_wheel.row(0).array() -= length / 2;
    rear_wheel = rotate * rear_wheel;
    rear_wheel.row(0).array() += x;
    rear_wheel.row(1).array() += y;

    std::vector<double> rear_wheel_x(rear_wheel.cols());
    std::vector<double> rear_wheel_y(rear_wheel.cols());

    for (int i = 0; i < rear_wheel.cols(); ++i) {
        rear_wheel_x[i] = rear_wheel(0, i);
        rear_wheel_y[i] = rear_wheel(1, i);
    }

    front_wheel.row(0).array() += length / 2;
    front_wheel = rotate * front_wheel;
    front_wheel.row(0).array() += x;
    front_wheel.row(1).array() += y;

    std::vector<double> front_wheel_x(front_wheel.cols());
    std::vector<double> front_wheel_y(front_wheel.cols());

    for (int i = 0; i < front_wheel.cols(); ++i) {
        front_wheel_x[i] = front_wheel(0, i);
        front_wheel_y[i] = front_wheel(1, i);
    }

    plt::plot(rear_wheel_x, rear_wheel_y, "-b");
    plt::plot(front_wheel_x, front_wheel_y, "-b");
    plt::plot({x}, {y}, "*");
}

int main() {
    std::vector<double> x = {0.0, 10.0, 20.0, 25.0, 30.0, 40.0};
    std::vector<double> y = {0.0, 5.0, 0.0, 0.0, -3.0, 0.0};
    CubicSpline2D sp(x, y);

    double sx = -5.0;
    double gx = 40.0;
    double sy = -5;
    double gy = 0.0;

    double dt = 0.1;
    double wheel_base = 3.0;
    size_t prediction_horizon = 20;

    std::vector<double> s;
    std::vector<double> x_ref, y_ref, yaw_ref, speed_ref;

    for (double t = 0; t < sp.getCumulativeDistance().back(); t += dt) {
        s.push_back(t);
    }

    for (double i_s : s) {
        auto [tx, ty] = sp.calculatePosition(i_s);
        x_ref.push_back(tx);
        y_ref.push_back(ty);
        yaw_ref.push_back(sp.calculateYaw(i_s));
        speed_ref.push_back(6.0);
    }
    SystemModel system(dt, wheel_base);
    system.setState(sx, sy, yaw_ref.front(), 0.0);
    Eigen::Vector4d state_weight;
    Eigen::Vector2d input_weight;

    state_weight << 3.0, 3.0, 1.0, 0.01;
    input_weight << 0.05, 0.05;
    MPC mpc(system, state_weight, input_weight, prediction_horizon, dt);

    std::vector<double> path_x;
    std::vector<double> path_y;

    std::vector<double> result_x;
    std::vector<double> result_y;

    plt::figure_size(1440, 1080);
    int target_point_idx = 0;
    Eigen::Vector4d state(sx, sy, 0, 0);
    std::map<std::string, std::string> options;
    options["color"] = "black";
    options["linestyle"] = "none";  // No line (just markers)
    options["marker"] = "x";        // Use 'x' markers
    options["markersize"] = "10";
    while (std::hypot(gx - state[0], gy - state[1]) >= 0.001) {
        if (x_ref.size() - target_point_idx <= prediction_horizon) break;
        mpc.run(slice(x_ref, target_point_idx, x_ref.size()),
                slice(y_ref, target_point_idx, y_ref.size()),
                slice(yaw_ref, target_point_idx, yaw_ref.size()),
                slice(speed_ref, target_point_idx, speed_ref.size()));
        target_point_idx = getClosestPointOnPath(x_ref, y_ref, state, target_point_idx, 5);
        path_x = mpc.getPredictedX();
        path_y = mpc.getPredictedY();
        result_x.push_back(state[0]);
        result_y.push_back(state[1]);
        plt::cla();
        plt::plot(x, y, options);
        state = mpc.getState();
        plt::plot(result_x, result_y, "ob");
        plt::plot(path_x, path_y, "xr");
        plot_car(state[0], state[1], state[2]);
        plt::axis("equal");
        plt::xlim(-10, 50);
        plt::ylim(-20, 20);
        plt::grid(true);
        plt::pause(0.001);
    }

    plt::show();

    return 0;
}
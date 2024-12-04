#define _USE_MATH_DEFINES

#include "mpc/math/CubicSpline2D.hpp"
#include "mpc/model/State.hpp"

#include <Eigen/Dense>

#include <matplotlibcpp.h>
#include <iostream>
#include <string>
#include <vector>

#include <math.h>
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>

using CppAD::AD;
using Eigen::VectorXd;
namespace plt = matplotlibcpp;

size_t N = 20;
double dt = 0.1;

const double wheel_base = 3.5;
double ref_v = 5;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t yaw_start = y_start + N;
size_t v_start = yaw_start + N;

//  input
size_t steer_start = v_start + N;
size_t acc_start = steer_start + N - 1;

std::vector<double> rx, ry, ryaw, rk;

class FG_eval {
public:
    // Coefficients of the fitted polynomial.
    FG_eval() {}

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    // `fg` is a vector containing the cost and constraints.
    // `vars` is a vector containing the variable values (state & actuators).
    void operator()(ADvector& fg, const ADvector& vars) {
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // The part of the cost based on the reference state.
        for (int t = 0; t < N; ++t) {
            fg[0] += 5.0 * CppAD::pow(vars[x_start + t] - rx[t], 2);
            fg[0] += 5.0 * CppAD::pow(vars[y_start + t] - ry[t], 2);
            fg[0] += 2.0 * CppAD::pow(vars[v_start + t] - ryaw[t], 2);
            fg[0] += 0.5 * CppAD::pow(vars[v_start + t] - ref_v, 2);
        }

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; ++t) {
            fg[0] += 0.01 * CppAD::pow(vars[steer_start + t], 2);
            fg[0] += 0.01 * CppAD::pow(vars[acc_start + t], 2);
        }

        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + yaw_start] = vars[yaw_start];
        fg[1 + v_start] = vars[v_start];

        // The rest of the constraints
        for (int t = 1; t < N; ++t) {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + t];
            AD<double> y1 = vars[y_start + t];
            AD<double> yaw1 = vars[yaw_start + t];
            AD<double> vel1 = vars[v_start + t];

            // The state at time t.
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> y0 = vars[y_start + t - 1];
            AD<double> yaw0 = vars[yaw_start + t - 1];
            AD<double> vel0 = vars[v_start + t - 1];

            // Only consider the actuation at time t.
            AD<double> steer0 = vars[steer_start + t - 1];
            AD<double> acc0 = vars[acc_start + t - 1];

            fg[1 + x_start + t] = x1 - (x0 + vel0 * CppAD::cos(yaw0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + vel0 * CppAD::sin(yaw0) * dt);
            fg[1 + yaw_start + t] = yaw1 - (yaw0 + vel0 / wheel_base * CppAD::tan(steer0) * dt);
            fg[1 + v_start + t] = vel1 - (vel0 + acc0 * dt);
        }
    }
};

std::vector<double> Solve(const VectorXd& x0) {
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = x0[0];
    double y = x0[1];
    double yaw = x0[2];
    double velocity = x0[3];

    // number of independent variables
    // N timesteps == N - 1 actuations
    size_t n_vars = N * 4 + (N - 1) * 2;
    // Number of constraints
    size_t n_constraints = N * 4;

    // Initial value of the independent variables.
    // Should be 0 except for the initial values.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; ++i) {
        vars[i] = 0.0;
    }
    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[yaw_start] = yaw;
    vars[v_start] = velocity;
    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    for (int i = x_start; i < y_start; ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    for (int i = y_start; i < yaw_start; ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    for (int i = yaw_start; i < v_start; ++i) {
        vars_lowerbound[i] = -M_PI;
        vars_upperbound[i] = M_PI;
    }
    for (int i = v_start; i < steer_start; ++i) {
        vars_lowerbound[i] = 0;
        vars_upperbound[i] = 20;
    }

    for (int i = steer_start; i < acc_start; ++i) {
        vars_lowerbound[i] = -M_PI_4;
        vars_upperbound[i] = M_PI_4;
    }
    for (int i = acc_start; i < n_vars; ++i) {
        vars_lowerbound[i] = -4.0;
        vars_upperbound[i] = 1.0;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; ++i) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[yaw_start] = yaw;
    constraints_lowerbound[v_start] = velocity;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[yaw_start] = yaw;
    constraints_upperbound[v_start] = velocity;

    // Object that computes objective and constraints
    FG_eval fg_eval;

    // options
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FG_eval>(options,
                                          vars,
                                          vars_lowerbound,
                                          vars_upperbound,
                                          constraints_lowerbound,
                                          constraints_upperbound,
                                          fg_eval,
                                          solution);

    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    auto cost = solution.obj_value;
    return {solution.x[x_start + 1],
            solution.x[y_start + 1],
            solution.x[yaw_start + 1],
            solution.x[v_start + 1],
            solution.x[steer_start],
            solution.x[acc_start]};
}

int main() {
    std::vector<double> x = {0.0, 10.0, 20.0, 25.0, 30.0, 40.0};
    std::vector<double> y = {0.0, 5.0, 0.0, 0.0, 0.0, 0.0};
    CubicSpline2D sp(x, y);

    double sx = x[0];
    double sy = -5;  // y[0];

    std::vector<double> s;
    for (double t = 0; t < sp.getCumulativeDistance().back(); t += dt) {
        s.push_back(t);
    }

    for (double i_s : s) {
        auto [tx, ty] = sp.calculatePosition(i_s);
        rx.push_back(tx);
        ry.push_back(ty);
        ryaw.push_back(sp.calculateYaw(i_s));
        rk.push_back(sp.calculateCurvature(i_s));
    }
    State state = State(sx, sy, ryaw.front(), 0.0);
    Eigen::Vector2d input;

    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> yaw;

    path_x.push_back(state.getX());
    path_y.push_back(state.getY());

    plt::figure();
    plt::plot(rx, ry, "-r");
    while (rx.size() >= N + 1) {
        auto vars = Solve(state.getState());
        state.updateState(vars[4], vars[5], dt, wheel_base);
        path_x.push_back(state.getX());
        path_y.push_back(state.getY());
        rx.erase(rx.begin());
        ry.erase(ry.begin());
        ryaw.erase(ryaw.begin());
    }
    plt::plot(path_x, path_y, "xb");
    plt::show();

    return 0;
}
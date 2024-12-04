#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

#define _USE_MATH_DEFINES

#include "mpc/math/CubicSpline2D.hpp"
#include "mpc/model/State.hpp"
#include "mpc/model/SystemModel.hpp"

#include <Eigen/Dense>

#include <math.h>
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>

using CppAD::AD;
class FG_eval {
public:
    FG_eval() = delete;
    FG_eval(const SystemModel& system,
            const std::vector<double>& x_ref,
            const std::vector<double>& y_ref,
            const std::vector<double>& yaw_ref,
            const std::vector<double>& velocity_ref,
            const size_t& prediction_horizon,
            const size_t& x_idx,
            const size_t& y_idx,
            const size_t& yaw_idx,
            const size_t& velocity_idx,
            const size_t& steer_idx,
            const size_t& acc_idx);

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars);

private:
    SystemModel system_;
    size_t prediction_horizon_;
    size_t x_idx_, y_idx_, yaw_idx_, velocity_idx_;
    size_t steer_idx_, acc_idx_;
    std::vector<double> x_ref_, y_ref_, yaw_ref_, velocity_ref_;

    double dt_, wheel_base_;
};

class Optimizer {
public:
    Optimizer() = delete;
    Optimizer(const SystemModel& system, const size_t& prediction_horizon);

    std::vector<double> Solve(const std::vector<double>& x_ref,
                              const std::vector<double>& y_ref,
                              const std::vector<double>& yaw_ref,
                              const std::vector<double>& velocity_ref);

private:
    SystemModel system_;
    size_t prediction_horizon_;
    size_t x_idx_, y_idx_, yaw_idx_, velocity_idx_;
    size_t steer_idx_, acc_idx_;
};

#endif
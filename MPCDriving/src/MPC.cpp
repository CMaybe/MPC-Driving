#include "mpc/MPC.hpp"
#include "mpc/Optimizer.hpp"

#include <cmath>
#include <iostream>

using namespace std;

MPC::MPC(const SystemModel& system, const size_t& prediction_horizon, const double& dt)
    : num_state_(4)
    , num_input_(2)
    , system_(system)
    , prediction_horizon_(prediction_horizon)
    , dt_(dt) {}

void MPC::run(const std::vector<double>& path_x,
              const std::vector<double>& path_y,
              const std::vector<double>& path_yaw,
              const std::vector<double>& path_velocity) {
    Optimizer optimizer(system_, prediction_horizon_);
    auto vars = optimizer.Solve(path_x, path_y, path_yaw, path_velocity);
    Eigen::Vector4d state(vars[0], vars[1], vars[2], vars[3]);
    Eigen::Vector2d input(vars[4], vars[5]);
    system_.updateModel(vars[2], vars[3], vars[4]);
    system_.updateState(state, input);
    output_ = system_.getState().getState();
}

Eigen::Vector4d MPC::getOutput() const { return output_; }

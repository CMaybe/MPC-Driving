#include "mpc_driving/MPC.hpp"
#include "mpc_driving/Optimizer.hpp"

#include <cmath>
#include <iostream>

using namespace std;

MPC::MPC(const SystemModel& system,
         const Eigen::Vector4d& state_weight,
         const Eigen::Vector2d& input_weight,
         const Eigen::Vector4d& state_lowerbound,
         const Eigen::Vector4d& state_upperbound,
         const Eigen::Vector2d& input_lowerbound,
         const Eigen::Vector2d& input_upperbound,
         const size_t& prediction_horizon,
         const double& dt)
    : system_(system)
    , state_weight_(state_weight)
    , input_weight_(input_weight)
    , state_lowerbound_(state_lowerbound)
    , state_upperbound_(state_upperbound)
    , input_lowerbound_(input_lowerbound)
    , input_upperbound_(input_upperbound)
    , prediction_horizon_(prediction_horizon)
    , dt_(dt) {
    predicted_x_.resize(prediction_horizon);
    predicted_y_.resize(prediction_horizon);
    predicted_yaw_.resize(prediction_horizon);
    predicted_velocity_.resize(prediction_horizon);
    predicted_steer_.resize(prediction_horizon);
    predicted_acc_.resize(prediction_horizon);
}

void MPC::run(const std::vector<double>& path_x,
              const std::vector<double>& path_y,
              const std::vector<double>& path_yaw,
              const std::vector<double>& path_velocity) {
    Optimizer optimizer(system_,
                        state_weight_,
                        input_weight_,
                        state_lowerbound_,
                        state_upperbound_,
                        input_lowerbound_,
                        input_upperbound_,
                        prediction_horizon_);
    auto vars = optimizer.Solve(path_x, path_y, path_yaw, path_velocity);
    for (size_t i = 0; i < prediction_horizon_; i++) {
        predicted_x_[i] = vars[i];
        predicted_y_[i] = vars[i + prediction_horizon_];
        predicted_yaw_[i] = vars[i + 2 * prediction_horizon_];
        predicted_velocity_[i] = vars[i + 3 * prediction_horizon_];
        predicted_steer_[i] = vars[i + 4 * prediction_horizon_];
        predicted_acc_[i] = vars[i + 5 * prediction_horizon_];
    }
    state_ << predicted_x_[1], predicted_y_[1], predicted_yaw_[1], predicted_velocity_[1];
    input_ << predicted_steer_[0], predicted_acc_[0];
}

void MPC::update() {
    system_.updateModel(state_[2], state_[3], input_[0]);
    system_.updateState(state_, input_);
}
void MPC::update(const Eigen::Vector4d& state, const Eigen::Vector2d& input) {
    state_ = state;
    input_ = input;
    system_.updateModel(state_[2], state_[3], input_[0]);
    system_.updateState(state_, input_);
}

Eigen::Vector4d MPC::getState() const { return state_; }
Eigen::Vector2d MPC::getInput() const { return input_; }

std::vector<double> MPC::getPredictedX() const { return predicted_x_; }
std::vector<double> MPC::getPredictedY() const { return predicted_y_; }
std::vector<double> MPC::getPredictedYaw() const { return predicted_yaw_; }
std::vector<double> MPC::getPredictedVelocoity() const { return predicted_velocity_; }
std::vector<double> MPC::getPredictedSteer() const { return predicted_steer_; }
std::vector<double> MPC::getPredictedAcc() const { return predicted_acc_; }
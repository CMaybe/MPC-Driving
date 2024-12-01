#include "mpc/model/State.hpp"

State::State(double x_state, double y_state, double yaw, double velocity)
    : x_(x_state), y_(y_state), yaw_(yaw), v_(velocity) {
    state_ << x_, y_, yaw_, v_;
}

void State::updateState(double acc, double steer_angle, double dt, double W_base) {
    x_ += v_ * std::cos(yaw_) * dt;
    y_ += v_ * std::sin(yaw_) * dt;
    yaw_ += (v_ / W_base) * std::tan(steer_angle) * dt;
    v_ += acc * dt;
}

Eigen::Vector4d State::getState() const { return state_; }

#include "mpc_driving/model/State.hpp"

State::State(double x_state, double y_state, double yaw, double velocity)
    : x_(x_state), y_(y_state), yaw_(yaw), velocity_(velocity) {
    state_vector_ << x_, y_, yaw_, velocity_;
}
State::State(const State& other)
    : x_(other.x_), y_(other.y_), yaw_(other.yaw_), velocity_(other.velocity_) {
    state_vector_ = other.state_vector_;
}
State::State(const Eigen::Vector4d& state_vector)
    : x_(state_vector[0])
    , y_(state_vector[1])
    , yaw_(state_vector[2])
    , velocity_(state_vector[3])
    , state_vector_(state_vector) {}

void State::updateState(double steer_angle, double acc, double dt, double wheel_base) {
    x_ += velocity_ * std::cos(yaw_) * dt;
    y_ += velocity_ * std::sin(yaw_) * dt;
    yaw_ += (velocity_ / wheel_base) * std::tan(steer_angle) * dt;
    velocity_ += acc * dt;
    state_vector_ << x_, y_, yaw_, velocity_;
}

Eigen::Vector4d State::getState() const { return state_vector_; }

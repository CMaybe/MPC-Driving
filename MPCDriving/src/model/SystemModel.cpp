#include "mpc/model/SystemModel.hpp"

#include <iostream>

SystemModel::SystemModel(const double& dt, const double& wheel_base)
    : dt_(dt), wheel_base_(wheel_base) {}
SystemModel::SystemModel(const SystemModel& other)
    : dt_(other.dt_), wheel_base_(other.wheel_base_), state_(other.state_) {}

void SystemModel::setState(double x, double y, double yaw, double velocity) {
    state_ = State(x, y, yaw, velocity);
}
void SystemModel::setState(const State& other) { state_ = other; }
void SystemModel::setState(const Eigen::Vector4d& state_vector) { state_ = State(state_vector); }

void SystemModel::updateModel(double yaw, double velocity, double steer) {
    A_.row(0) << 0.0, 0.0, -velocity * std::sin(yaw), std::cos(yaw);
    A_.row(1) << 0.0, 0.0, velocity * std::cos(yaw), std::sin(yaw);
    A_.row(2) << 0.0, 0.0, 0.0, std::tan(steer) / wheel_base_;
    A_.row(3) << 0.0, 0.0, 0.0, 0.0;

    B_.row(0) << 0.0, 0.0;
    B_.row(1) << 0.0, 0.0;
    B_.row(2) << velocity / (wheel_base_ * std::cos(steer) * std::cos(steer)), 0.0;
    B_.row(3) << 0.0, 1;

    C_[0] = velocity * std::sin(yaw) * yaw;
    C_[1] = -velocity * std::cos(yaw) * yaw;
    C_[2] = -velocity * steer / (wheel_base_ * std::cos(steer) * std::cos(steer));
    C_[3] = 0.0;

    updateDiscretizedModel();
}

void SystemModel::updateDiscretizedModel() {
    Ad_ = (Eigen::Matrix4d::Identity() + 0.5 * dt_ * A_) *
          (Eigen::Matrix4d::Identity() - 0.5 * dt_ * A_).inverse();
    Bd_ = B_ * dt_;
    Cd_ = C_ * dt_;
}

Eigen::Vector4d SystemModel::updateState(const Eigen::Vector4d& state,
                                         const Eigen::Vector2d& input) {
    Eigen::Vector4d result = Ad_ * state_.getState() + Bd_ * input + Cd_;
    state_ = result;
    return state_.getState();
}

Eigen::Matrix<double, 4, 4> SystemModel::getA() const { return A_; }
Eigen::Matrix<double, 4, 4> SystemModel::getAd() const { return Ad_; }
Eigen::Matrix<double, 4, 2> SystemModel::getB() const { return B_; }
Eigen::Matrix<double, 4, 2> SystemModel::getBd() const { return Bd_; }
Eigen::Vector4d SystemModel::getC() const { return C_; }
Eigen::Vector4d SystemModel::getCd() const { return Cd_; }

State SystemModel::getState() const { return state_; }
double SystemModel::getWheelBase() const { return wheel_base_; }
double SystemModel::getDt() const { return dt_; }

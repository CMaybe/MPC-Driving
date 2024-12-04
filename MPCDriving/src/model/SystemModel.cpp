#include "mpc/model/SystemModel.hpp"
#include "SystemModel.hpp"

SystemModel::SystemModel(double dt) : dt_(dt) {}
SystemModel::SystemModel(const SystemModel& other)
    : dt_(other.dt_), Bd_(other.Bd_), state_(other.state_) {}

void SystemModel::setState(double x, double y, double yaw, double velocity) {
    state_ = State(x, y, yaw, velocity);
}
void SystemModel::setState(const State& other) { state_ = State(other); }
void SystemModel::setState(const Eigen::Vector4d& state_vector) { state_ = State(state_vector); }

void SystemModel::updateModel(double yaw, double velocity, double steer) {
    A_.row(0) << 0.0, 0.0, -velocity * std::sin(yaw), std::cos(yaw);
    A_.row(1) << 0.0, 0.0, velocity * std::cos(yaw), std::sin(yaw);
    A_.row(2) << 0.0, 0.0, 0.0, std::tan(steer) / wheel_base_;
    A_.row(3) << 0.0, 0.0, 0.0, 0.0;

    B_.row(0) << 0.0, 0.0;
    B_.row(1) << 0.0, 0.0;
    B_.row(2) << velocity * wheel_base_ / (std::cos(steer) * std::cos(steer)), 0.0;
    B_.row(3) << 0.0, 1;

    C_.row(0) << 1.0, 0.0, 0.0, 0.0;
    C_.row(1) << 0.0, 1.0, 0.0, 0.0;
    C_.row(2) << 0.0, 0.0, 1.0, 0.0;

    updateDiscretizedModel();
}

void SystemModel::updateDiscretizedModel() {
    Ad_ = (Eigen::Matrix4d::Identity() + 0.5 * dt_ * A_) *
          (Eigen::Matrix4d::Identity() - 0.5 * dt_ * A_).inverse();
    Bd_ = B_ * dt_;
}

Eigen::Vector4d SystemModel::updateState(const Eigen::Vector4d& state,
                                         const Eigen::Vector2d& input) {
    // return Ad_ * state + Bd_ * input;
    state_.updateState(input[0], input[1], dt_, wheel_base_);
    return state_.getState();
    // (x0 + vel0 * CppAD::cos(yaw0) * dt_);
    // fg[1 + y_idx_ + t] = y1 - (y0 + vel0 * CppAD::sin(yaw0) * dt_);
    // fg[1 + yaw_idx_ + t] = yaw1 - (yaw0 + vel0 / wheel_base_ * CppAD::tan(steer0) * dt_);
    // fg[1 + velocity_idx_ + t] = vel1 - (vel0 + acc0 * dt_);
}

Eigen::Matrix4d SystemModel::getA() const { return A_; }
Eigen::Matrix4d SystemModel::getB() const { return B_; }
Eigen::Matrix4d SystemModel::getC() const { return C_; }
Eigen::Matrix4d SystemModel::getAd() const { return Ad_; }
Eigen::Matrix4d SystemModel::getBd() const { return Bd_; }

State SystemModel::getState() const { return state_; }
double SystemModel::getWheelBase() const { return wheel_base_; }
double SystemModel::getDt() const { return dt_; }

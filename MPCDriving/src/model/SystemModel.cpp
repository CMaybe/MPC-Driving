#include "SystemModel.hpp"

SystemModel::SystemModel(double dt, double wheel_base) : dt_(dt), wheel_base_(wheel_base) {}

void SystemModel::updateModel(double velocity, double yaw, double steer) {
    A_ = Eigen::MatrixXd(4, 4);
    A_ << 1.0, 0.0, -dt_ * velocity * std::sin(yaw), dt_ * std::cos(yaw), 0.0, 1.0,
        dt_ * velocity * std::cos(yaw), dt_ * std::sin(yaw), 0.0, 0.0, 1.0,
        dt_ * std::tan(steer) / wheel_base_, 0.0, 0.0, 0.0, 1.0;

    B_ = Eigen::MatrixXd(4, 2);
    B_ << 0.0, 0.0, 0.0, 0.0, 0.0,
        dt_ * velocity / (wheel_base_ * std::cos(steer) * std::cos(steer)), dt_, 0.0;

    C_ = Eigen::VectorXd(4);
    C_ << dt_ * velocity * std::sin(yaw) * yaw, -dt_ * velocity * std::cos(yaw) * yaw,
        -dt_ * velocity * steer / (wheel_base_ * std::cos(steer) * std::cos(steer)), 0.0;
}

Eigen::MatrixXd SystemModel::getA() const { return A_; }
Eigen::MatrixXd SystemModel::getB() const { return B_; }
Eigen::VectorXd SystemModel::getC() const { return C_; }
double SystemModel::getWheelBase() const { return wheel_base_; }

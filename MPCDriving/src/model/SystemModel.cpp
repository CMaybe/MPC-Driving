#include "mpc/model/SystemModel.hpp"

SystemModel::SystemModel(double dt, double wheel_base) : dt_(dt), wheel_base_(wheel_base) {}

void SystemModel::updateModel(double yaw, double velocity, double steer) {
    A_ = Eigen::MatrixXd(4, 4);
    A_.row(0) << 1.0, 0.0, -dt_ * velocity * std::sin(yaw), dt_ * std::cos(yaw);
    A_.row(1) << 0.0, 1.0, dt_ * velocity * std::cos(yaw), dt_ * std::sin(yaw);
    A_.row(2) << 0.0, 0.0, 1.0, dt_ * std::tan(steer) / wheel_base_;
    A_.row(3) << 0.0, 0.0, 0.0, 1.0;

    B_ = Eigen::MatrixXd(4, 2);
    B_.row(0) << 0.0, 0.0;
    B_.row(1) << 0.0, 0.0;
    B_.row(2) << 0.0, dt_ * velocity / (wheel_base_ * std::cos(steer) * std::cos(steer));
    B_.row(3) << dt_, 0.0;

    C_ = Eigen::MatrixXd(3, 4);
    C_.row(0) << 1.0, 0.0, 0.0, 0.0;
    C_.row(1) << 0.0, 1.0, 0.0, 0.0;
    C_.row(2) << 0.0, 0.0, 1.0, 0.0;

    Ad_ = Eigen::VectorXd(4);
    Ad_ << dt_ * velocity * std::sin(yaw) * yaw, -dt_ * velocity * std::cos(yaw) * yaw,
        -dt_ * velocity * steer / (wheel_base_ * std::cos(steer) * std::cos(steer)), 0.0;
}

Eigen::MatrixXd SystemModel::getA() const { return A_; }
Eigen::MatrixXd SystemModel::getB() const { return B_; }
Eigen::MatrixXd SystemModel::getC() const { return C_; }
Eigen::VectorXd SystemModel::getAd() const { return Ad_; }
double SystemModel::getWheelBase() const { return wheel_base_; }

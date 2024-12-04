#include "mpc/model/SystemModel.hpp"
#include "SystemModel.hpp"

SystemModel::SystemModel(double dt, double wheel_base) : dt_(dt), wheel_base_(wheel_base) {}

void SystemModel::updateModel(double yaw, double velocity, double steer) {
    A_ = Eigen::MatrixXd(4, 4);
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
                                         const Eigen::Vector4d& input) {
    return Ad_ * state + Bd_ * input;
}

Eigen::Matrix4d SystemModel::getA() const { return A_; }
Eigen::Matrix4d SystemModel::getB() const { return B_; }
Eigen::Matrix4d SystemModel::getC() const { return C_; }
Eigen::Matrix4d SystemModel::getAd() const { return Ad_; }
Eigen::Matrix4d SystemModel::getBd() const { return Bd_; }
double SystemModel::getWheelBase() const { return wheel_base_; }

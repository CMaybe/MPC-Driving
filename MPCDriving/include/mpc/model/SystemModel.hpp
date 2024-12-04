#ifndef System_MODEL_HPP
#define System_MODEL_HPP

#include "mpc/model/State.hpp"

#include <Eigen/Dense>
#include <cmath>

class SystemModel {
public:
    SystemModel() = default;
    SystemModel(double dt, double wheel_base);
    SystemModel(const SystemModel& other);

    void setState(double x, double y, double yaw, double velocity);
    void setState(const State& other);
    void setState(const Eigen::Vector4d& state_vector);
    void updateModel(double yaw, double velocity, double steer);

    Eigen::Matrix4d getA() const;
    Eigen::Matrix4d getB() const;
    Eigen::Matrix4d getC() const;
    Eigen::Matrix4d getAd() const;
    Eigen::Matrix4d getBd() const;

    double getWheelBase() const;

private:
    double dt_;
    double wheel_base_;

    void updateDiscretizedModel();
    Eigen::Vector4d updateState(const Eigen::Vector4d& state, const Eigen::Vector4d& input);
    Eigen::Matrix4d A_, B_, C_;
    Eigen::Matrix4d Ad_, Bd_;

    State state_;
};

#endif  // System_MODEL_HPP

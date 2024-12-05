#ifndef System_MODEL_HPP
#define System_MODEL_HPP

#include "mpc/model/State.hpp"

#include <Eigen/Dense>
#include <cmath>

class SystemModel {
public:
    SystemModel() = default;
    SystemModel(const double& dt, const double& wheel_base_ = 3.5);
    SystemModel(const SystemModel& other);

    void setState(double x, double y, double yaw, double velocity);
    void setState(const State& other);
    void setState(const Eigen::Vector4d& state_vector);
    void updateModel(double yaw, double velocity, double steer);
    Eigen::Vector4d updateState(const Eigen::Vector4d& state, const Eigen::Vector2d& input);

    Eigen::Matrix<double, 4, 4> getA() const;
    Eigen::Matrix<double, 4, 4> getAd() const;
    Eigen::Matrix<double, 4, 2> getB() const;
    Eigen::Matrix<double, 4, 2> getBd() const;
    Eigen::Vector4d getC() const;
    Eigen::Vector4d getCd() const;
    State getState() const;

    double getDt() const;
    double getWheelBase() const;

private:
    void updateDiscretizedModel();

    double dt_, wheel_base_;

    Eigen::Matrix<double, 4, 4> A_, Ad_;
    Eigen::Matrix<double, 4, 2> B_, Bd_;
    Eigen::Vector4d C_, Cd_;

    State state_;
};

#endif  // System_MODEL_HPP

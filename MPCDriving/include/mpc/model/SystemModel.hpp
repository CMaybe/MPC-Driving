#ifndef System_MODEL_HPP
#define System_MODEL_HPP

#include <Eigen/Dense>
#include <cmath>

class SystemModel {
public:
    SystemModel() = default;
    SystemModel(double dt, double wheel_base);

    void updateModel(double velocity, double yaw, double steer);

    Eigen::MatrixXd getA() const;
    Eigen::MatrixXd getB() const;
    Eigen::VectorXd getC() const;

    double getWheelBase() const;

private:
    double dt_;
    double wheel_base_;

    Eigen::MatrixXd A_, B_, C_;
    Eigen::MatrixXd Ad_;
};

#endif  // System_MODEL_HPP

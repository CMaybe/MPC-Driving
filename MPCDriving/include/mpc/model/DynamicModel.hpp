#ifndef DYNAMIC_MODEL_HPP
#define DYNAMIC_MODEL_HPP

#include <Eigen/Dense>
#include <cmath>

class DynamicModel {
public:
    DynamicModel(){};
    DynamicModel(double dt, double wheel_base);

    void computeModel(double velocity, double yaw, double steer);

    Eigen::MatrixXd getA() const;
    Eigen::MatrixXd getB() const;
    Eigen::VectorXd getC() const;

    double getWheelBase() const;

private:
    double dt_;
    double wheel_base_;

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::VectorXd C_;
};

#endif  // DYNAMIC_MODEL_HPP

#ifndef STATE_HPP
#define STATE_HPP

#include <Eigen/Dense>
class State {
public:
    State(double x_state, double y_state, double yaw, double velocity);

    void updateState(double acc, double steer_angle, double dt, double W_base);

    Eigen::Vector4d getState() const;

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getYaw() const { return yaw_; }
    double getVelocity() const { return v_; }

private:
    double x_;
    double y_;
    double yaw_;
    double v_;

    Eigen::Vector4d state_;
};

#endif  // STATE_HPP

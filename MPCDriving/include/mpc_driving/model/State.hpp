#ifndef STATE_HPP
#define STATE_HPP

#include <Eigen/Dense>
class State {
public:
    State() = default;
    State(double x, double y, double yaw, double velocity);
    State(const State& other);
    State(const Eigen::Vector4d& state_vector);

    void updateState(double steer_angle, double acc, double dt, double wheel_base);

    Eigen::Vector4d getState() const;

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getYaw() const { return yaw_; }
    double getVelocity() const { return velocity_; }

private:
    double x_;
    double y_;
    double yaw_;
    double velocity_;

    Eigen::Vector4d state_vector_;
};

#endif  // STATE_HPP

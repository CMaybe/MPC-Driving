#ifndef STATE_HPP
#define STATE_HPP

#include <Eigen/Dense>
class State {
public:
    State() = default;
    State(double x, double y, double yaw, double velocity);
    State(const State& other)
        : x_(other.x_), y_(other.y_), yaw_(other.yaw_), velocity_(other.velocity_) {
        state_vector_ = other.state_vector_;
    }
    State(const Eigen::Vector4d& vector)
        : x_(vector[0]), y_(vector[1]), yaw_(vector[2]), velocity_(vector[3]) {
        state_vector_ = vector;
    }

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

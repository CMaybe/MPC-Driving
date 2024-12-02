#ifndef STATE_HPP
#define STATE_HPP

#include <Eigen/Dense>
class State {
public:
    State() = default;
    State(double x, double y, double yaw, double velocity);
    State(const State& other) : x_(other.x_), y_(other.y_), yaw_(other.yaw_), v_(other.v_) {
        state_vector_ = other.state_vector_;
    }

    void updateState(double acc, double steer_angle, double dt, double wheel_base);

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

    Eigen::Vector4d state_vector_;
};

#endif  // STATE_HPP

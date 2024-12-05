#ifndef MPC_HPP
#define MPC_HPP

#include <Eigen/Dense>
#include <vector>

#include "mpc/model/SystemModel.hpp"

class MPC {
public:
    // Constructor to initialize parameters
    MPC() = delete;
    MPC(const SystemModel& system,
        const Eigen::Vector4d& state_weight,
        const Eigen::Vector2d& input_weight,
        const size_t& prediction_horizon,
        const double& dt);

    void run(const std::vector<double>& path_x,
             const std::vector<double>& path_y,
             const std::vector<double>& path_yaw,
             const std::vector<double>& path_velocity);

    std::vector<double> getPredictedX() const;
    std::vector<double> getPredictedY() const;
    std::vector<double> getPredictedYaw() const;
    std::vector<double> getPredictedVelocoity() const;
    std::vector<double> getPredictedSteer() const;
    std::vector<double> getPredictedAcc() const;
    Eigen::Vector4d getState() const;
    Eigen::Vector2d getInput() const;

private:
    Eigen::Vector4d state_;
    Eigen::Vector2d input_;
    size_t num_state_, num_input_, num_output_, prediction_horizon_;
    double dt_;

    SystemModel system_;

    Eigen::Vector4d state_weight_;
    Eigen::Vector2d input_weight_;

    std::vector<double> predicted_x_;
    std::vector<double> predicted_y_;
    std::vector<double> predicted_yaw_;
    std::vector<double> predicted_velocity_;
    std::vector<double> predicted_steer_;
    std::vector<double> predicted_acc_;
};
#endif  // MPC_HPP

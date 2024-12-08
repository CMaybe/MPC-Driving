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
        const Eigen::Vector4d& state_lowerbound,
        const Eigen::Vector4d& state_upperbound,
        const Eigen::Vector2d& input_lowerbound,
        const Eigen::Vector2d& input_upperbound,
        const size_t& prediction_horizon,
        const double& dt);

    void run(const std::vector<double>& path_x,
             const std::vector<double>& path_y,
             const std::vector<double>& path_yaw,
             const std::vector<double>& path_velocity);
    void update(const Eigen::Vector4d& state, const Eigen::Vector2d& input);
    void update();

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

    Eigen::Vector4d state_weight_, state_lowerbound_, state_upperbound_;
    Eigen::Vector2d input_weight_, input_lowerbound_, input_upperbound_;

    std::vector<double> predicted_x_, predicted_y_, predicted_yaw_, predicted_velocity_,
        predicted_steer_, predicted_acc_;
};
#endif  // MPC_HPP

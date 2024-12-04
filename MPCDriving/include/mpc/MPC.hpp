#ifndef MPC_HPP
#define MPC_HPP

#include <Eigen/Dense>
#include <vector>

#include "mpc/model/SystemModel.hpp"

class MPC {
public:
    // Constructor to initialize parameters
    MPC() = delete;
    MPC(const SystemModel& system, const size_t& prediction_horizon, const double& dt);

    void run(const std::vector<double>& path_x,
             const std::vector<double>& path_y,
             const std::vector<double>& path_yaw,
             const std::vector<double>& path_velocity);

    Eigen::Vector4d getOutput() const;

private:
    size_t num_state_, num_input_, num_output_, prediction_horizon_;
    double dt_;
    Eigen::Vector4d output_;

    SystemModel system_;
};
#endif  // MPC_HPP

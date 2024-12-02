#ifndef MPC_HPP
#define MPC_HPP

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <vector>

#include "mpc/model/State.hpp"
#include "mpc/model/SystemModel.hpp"

class MPC {
public:
    // Constructor to initialize parameters
    MPC() = delete;
    MPC(int num_state, int num_input, int prediction_horizon, double dt);

    void optimize(const Eigen::VectorXd& cur_state_vec, const Eigen::MatrixXd& state_des);
    void setSystemModel(double velocity, double yaw, double steer);
    void updateDiscretizedMatrices();
    void updateArgumentedMatrices();

    Eigen::MatrixXd calcPredictedState(const Eigen::VectorXd& acc, const Eigen::VectorXd& steer);

private:
    int num_state_, num_input_, num_output_, prediction_horizon_;
    double dt_;
    double max_steer_rate, max_speed, max_reverse_speed;
    double max_steer_angle, max_acc;
    Eigen::MatrixXd x, u;
    Eigen::Vector4d x_max_, x_min_;
    Eigen::Vector4d u_max_, u_min_;
    Eigen::MatrixXd Q_, R_;
    Eigen::MatrixXd Q_augmented_, R_augmented_;
    Eigen::VectorXd mpc_acc, mpc_steer;

    Eigen::MatrixXd C_, D_, F_, G_, W_;
    Eigen::MatrixXd C_augmented_, D_augmented_, F_augmented_, G_augmented_;
    Eigen::MatrixXd state_constraint_, input_constraint_;
    Eigen::MatrixXd state_augmented_constraint_, output_augmented_constraint_;

    Eigen::MatrixXd H_, constraint_matrix_;
    Eigen::VectorXd f_, constraint_bound_vector_;

    OsqpEigen::Solver solver_;
    SystemModel system_;
    State state_;
};
#endif  // MPC_HPP

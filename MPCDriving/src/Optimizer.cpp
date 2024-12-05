#define _USE_MATH_DEFINES

#include "mpc/Optimizer.hpp"

FG_eval::FG_eval(const SystemModel& system,
                 const Eigen::Vector4d& state_weight,
                 const Eigen::Vector2d& input_weight,
                 const std::vector<double>& x_ref,
                 const std::vector<double>& y_ref,
                 const std::vector<double>& yaw_ref,
                 const std::vector<double>& velocity_ref,
                 const size_t& prediction_horizon,
                 const size_t& x_idx,
                 const size_t& y_idx,
                 const size_t& yaw_idx,
                 const size_t& velocity_idx,
                 const size_t& steer_idx,
                 const size_t& acc_idx)
    : system_(system)
    , state_weight_(state_weight)
    , input_weight_(input_weight)
    , x_ref_(x_ref)
    , y_ref_(y_ref)
    , yaw_ref_(yaw_ref)
    , velocity_ref_(velocity_ref)
    , wheel_base_(system.getWheelBase())
    , dt_(system.getDt())
    , prediction_horizon_(prediction_horizon)
    , x_idx_(x_idx)
    , y_idx_(y_idx)
    , yaw_idx_(yaw_idx)
    , velocity_idx_(velocity_idx)
    , steer_idx_(steer_idx)
    , acc_idx_(acc_idx){};

void FG_eval::operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < prediction_horizon_; ++t) {
        fg[0] += state_weight_[0] * CppAD::pow(vars[x_idx_ + t] - x_ref_[t], 2);
        fg[0] += state_weight_[1] * CppAD::pow(vars[y_idx_ + t] - y_ref_[t], 2);
        fg[0] += state_weight_[2] * CppAD::pow(vars[yaw_idx_ + t] - yaw_ref_[t], 2);
        fg[0] += state_weight_[3] * CppAD::pow(vars[velocity_idx_ + t] - velocity_ref_[t], 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < prediction_horizon_ - 1; ++t) {
        fg[0] += input_weight_[0] * CppAD::pow(vars[steer_idx_ + t], 2);
        fg[0] += input_weight_[1] * CppAD::pow(vars[acc_idx_ + t], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.

    fg[1 + x_idx_] = vars[x_idx_];
    fg[1 + y_idx_] = vars[y_idx_];
    fg[1 + yaw_idx_] = vars[yaw_idx_];
    fg[1 + velocity_idx_] = vars[velocity_idx_];

    // The rest of the constraints
    for (size_t t = 1; t < prediction_horizon_; ++t) {
        // The state at time t+1 .
        AD<double> x1 = vars[x_idx_ + t];
        AD<double> y1 = vars[y_idx_ + t];
        AD<double> yaw1 = vars[yaw_idx_ + t];
        AD<double> vel1 = vars[velocity_idx_ + t];

        // The state at time t.
        AD<double> x0 = vars[x_idx_ + t - 1];
        AD<double> y0 = vars[y_idx_ + t - 1];
        AD<double> yaw0 = vars[yaw_idx_ + t - 1];
        AD<double> vel0 = vars[velocity_idx_ + t - 1];

        // Only consider the actuation at time t.
        AD<double> steer0 = vars[steer_idx_ + t - 1];
        AD<double> acc0 = vars[acc_idx_ + t - 1];

        fg[1 + x_idx_ + t] = x1 - (x0 + vel0 * CppAD::cos(yaw0) * dt_);
        fg[1 + y_idx_ + t] = y1 - (y0 + vel0 * CppAD::sin(yaw0) * dt_);
        fg[1 + yaw_idx_ + t] = yaw1 - (yaw0 + vel0 / wheel_base_ * CppAD::tan(steer0) * dt_);
        fg[1 + velocity_idx_ + t] = vel1 - (vel0 + acc0 * dt_);
    }
}

Optimizer::Optimizer(const SystemModel& system,
                     const Eigen::Vector4d& state_weight,
                     const Eigen::Vector2d& input_weight,
                     const size_t& prediction_horizon)
    : system_(system)
    , state_weight_(state_weight)
    , input_weight_(input_weight)
    , prediction_horizon_(prediction_horizon) {
    x_idx_ = 0;
    y_idx_ = x_idx_ + prediction_horizon;
    yaw_idx_ = y_idx_ + prediction_horizon;
    velocity_idx_ = yaw_idx_ + prediction_horizon;
    steer_idx_ = velocity_idx_ + prediction_horizon;
    acc_idx_ = steer_idx_ + prediction_horizon - 1;
}

std::vector<double> Optimizer::Solve(const std::vector<double>& x_ref,
                                     const std::vector<double>& y_ref,
                                     const std::vector<double>& yaw_ref,
                                     const std::vector<double>& velocity_ref) {
    typedef CPPAD_TESTVECTOR(double) Dvector;

    State state0 = system_.getState();
    double x = state0.getX();
    double y = state0.getY();
    double yaw = state0.getYaw();
    double velocity = state0.getVelocity();

    // number of independent variables
    // prediction_horizon_ timesteps == prediction_horizon_ - 1 actuations
    size_t n_vars = prediction_horizon_ * 4 + (prediction_horizon_ - 1) * 2;
    // Number of constraints
    size_t n_constraints = prediction_horizon_ * 4;

    // Initial value of the independent variables.
    // Should be 0 except for the initial values.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; ++i) {
        vars[i] = 0.0;
    }
    // Set the initial variable values
    vars[x_idx_] = x;
    vars[y_idx_] = y;
    vars[yaw_idx_] = yaw;
    vars[velocity_idx_] = velocity;
    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = x_idx_; i < y_idx_; ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    for (int i = y_idx_; i < yaw_idx_; ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    for (int i = yaw_idx_; i < velocity_idx_; ++i) {
        vars_lowerbound[i] = -M_PI;
        vars_upperbound[i] = M_PI;
    }
    for (int i = velocity_idx_; i < steer_idx_; ++i) {
        vars_lowerbound[i] = 0;
        vars_upperbound[i] = 20;
    }

    for (int i = steer_idx_; i < acc_idx_; ++i) {
        vars_lowerbound[i] = -M_PI_4;
        vars_upperbound[i] = M_PI_4;
    }
    for (int i = acc_idx_; i < n_vars; ++i) {
        vars_lowerbound[i] = -4.0;
        vars_upperbound[i] = 1.0;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; ++i) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_idx_] = x;
    constraints_lowerbound[y_idx_] = y;
    constraints_lowerbound[yaw_idx_] = yaw;
    constraints_lowerbound[velocity_idx_] = velocity;

    constraints_upperbound[x_idx_] = x;
    constraints_upperbound[y_idx_] = y;
    constraints_upperbound[yaw_idx_] = yaw;
    constraints_upperbound[velocity_idx_] = velocity;

    // Object that computes objective and constraints
    FG_eval fg_eval(system_,
                    state_weight_,
                    input_weight_,
                    x_ref,
                    y_ref,
                    yaw_ref,
                    velocity_ref,
                    prediction_horizon_,
                    x_idx_,
                    y_idx_,
                    yaw_idx_,
                    velocity_idx_,
                    steer_idx_,
                    acc_idx_);

    // options
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FG_eval>(options,
                                          vars,
                                          vars_lowerbound,
                                          vars_upperbound,
                                          constraints_lowerbound,
                                          constraints_upperbound,
                                          fg_eval,
                                          solution);

    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    auto cost = solution.obj_value;
    std::vector<double> result(solution.x.size());
    for (size_t i = 0; i < solution.x.size(); i++) {
        result[i] = solution.x[i];
    }
    return result;
}

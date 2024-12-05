#include "mpc/Utils.hpp"
#include <algorithm>
#include <cmath>

int getClosestPointOnPath(const std::vector<double>& path_x,
                          const std::vector<double>& path_y,
                          const Eigen::Vector4d& cur_state_vec,
                          int point,
                          int N_search) {
    // Extract the next N_search points from the path
    std::vector<double> next_x(path_x.begin() + point, path_x.begin() + point + N_search);
    std::vector<double> next_y(path_y.begin() + point, path_y.begin() + point + N_search);

    // Compute the squared differences for each point
    std::vector<double> dist_sq(N_search);
    for (int i = 0; i < N_search; ++i) {
        Eigen::Vector2d diff(next_x[i] - cur_state_vec[0], next_y[i] - cur_state_vec[1]);
        dist_sq[i] = diff.squaredNorm();  // Using Eigen's squaredNorm() to compute squared distance
    }

    // Find the minimum distance squared
    double min_d = *std::min_element(dist_sq.begin(), dist_sq.end());

    // Find the index of the minimum distance squared
    auto it = std::find(dist_sq.begin(), dist_sq.end(), min_d);
    int target_pt = std::distance(dist_sq.begin(), it) + point;

    return target_pt;
}

Eigen::MatrixXd calDesiredTrajectory(const Eigen::Vector4d& cur_state_vec,
                                     const std::vector<double>& path_x,
                                     const std::vector<double>& path_y,
                                     const std::vector<double>& path_yaw,
                                     double dist_step,
                                     int target_pt,
                                     int Nx,
                                     int H,
                                     double dt,
                                     double desired_speed) {
    Eigen::MatrixXd traj_des(Nx, H + 1);  // Trajectory matrix

    double distance = 0;
    int total_pts = path_x.size();

    traj_des(0, 0) = path_x[target_pt];
    traj_des(1, 0) = path_y[target_pt];
    traj_des(2, 0) = path_yaw[target_pt];
    traj_des(3, 0) = desired_speed;

    for (int i = 0; i < H; ++i) {
        distance += std::abs(cur_state_vec[1]) * dt;
        int pts_travelled = std::round(distance / dist_step);

        if ((target_pt + pts_travelled) < total_pts) {
            traj_des(0, i + 1) = path_x[target_pt + pts_travelled];
            traj_des(1, i + 1) = path_y[target_pt + pts_travelled];
            traj_des(2, i + 1) = path_yaw[target_pt + pts_travelled];
            if ((target_pt + pts_travelled) == total_pts - 1) {
                traj_des(3, i + 1) = 0.0;
            } else {
                traj_des(3, i + 1) = desired_speed;
            }
        } else {
            traj_des(0, i + 1) = path_x[total_pts - 1];
            traj_des(1, i + 1) = path_y[total_pts - 1];
            traj_des(2, i + 1) = path_yaw[total_pts - 1];
            traj_des(3, i + 1) = 0.0;
        }
    }

    if (traj_des(3, 1) == 0.0) {
        traj_des(3, 0) = 0.0;
    }

    return traj_des;
}

std::vector<double> slice(const std::vector<double>& vec, size_t start, size_t end) {
    if (start >= vec.size() || start > end) {
        return {};
    }
    end = std::min(end, vec.size());
    return std::vector<double>(vec.begin() + start, vec.begin() + end);
}
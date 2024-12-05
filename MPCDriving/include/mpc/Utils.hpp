#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include <vector>

int getClosestPointOnPath(const std::vector<double>& path_x,
                          const std::vector<double>& path_y,
                          const Eigen::Vector4d& cur_state_vec,
                          int point,
                          int N_search);

Eigen::MatrixXd calDesiredTrajectory(const Eigen::Vector4d& cur_state_vec,
                                     const std::vector<double>& path_x,
                                     const std::vector<double>& path_y,
                                     const std::vector<double>& path_yaw,
                                     double dist_step,
                                     int target_pt,
                                     int Nx,
                                     int H,
                                     double dt,
                                     double desired_speed);

std::vector<double> slice(const std::vector<double>& vec, size_t start, size_t end);
#endif  // UTILS_HPP
#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include <vector>

int getClosestPointOnPath(const std::vector<double>& path_x,
                          const std::vector<double>& path_y,
                          const Eigen::Vector2d& cur_state_vec,
                          int point,
                          int N_search);

Eigen::MatrixXd calDesiredTrajectory(const Eigen::Vector2d& cur_state_vec,
                                     const std::vector<double>& path_x,
                                     const std::vector<double>& path_y,
                                     double dist_step,
                                     const std::vector<double>& path_yaw,
                                     int target_pt,
                                     int Nx,
                                     int H,
                                     double dt,
                                     double desired_speed);
#endif  // UTILS_HPP
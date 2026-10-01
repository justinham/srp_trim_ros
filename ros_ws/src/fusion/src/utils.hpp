#ifndef UTILS_HPP
#define UTILS_HPP
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include <Eigen/Dense>
#include <array>

double time_to_double(builtin_interfaces::msg::Time t);

double time_dif_double(builtin_interfaces::msg::Time t1, builtin_interfaces::msg::Time t2);

builtin_interfaces::msg::Time double_to_time(double t);

double dist_between_two_points_gps(double lat1, double lon1, double lat2, double lon2);

void cov_array_to_matrix(std::array<double, 16> &array, Eigen::Matrix<double, 4, 4> &mat);

void cov_matrix_to_array(std::array<double, 16> &array, Eigen::Matrix<double, 4, 4> &mat);

void log_2Dmatrix(rclcpp::Node *node, Eigen::MatrixXd mat);
#endif
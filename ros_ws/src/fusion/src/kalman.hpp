#ifndef KALMAN_HPP
#define KALMAN_HPP 1

#include <Eigen/Dense>
#include <vector>
#include "rclcpp/rclcpp.hpp"

struct kalman_pair{
    Eigen::MatrixXd state;
    Eigen::MatrixXd covariance;
};

class KalmanFilter {

    public:
        KalmanFilter();
        KalmanFilter(long int state_size, Eigen::MatrixXd Q);
        ~KalmanFilter();
        kalman_pair predict(kalman_pair input, Eigen::MatrixXd F, rclcpp::Node *node);
        void refine(kalman_pair &input, std::vector<kalman_pair> observations, Eigen::MatrixXd sensor_transform, rclcpp::Node *node);

    private:
        bool initalized;
        long int state_size;
        Eigen::MatrixXd Q;
};

#endif
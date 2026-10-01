#include "kalman.hpp"
#include "utils.hpp"

#include <stdexcept>
#include <Eigen/Dense>
#include <vector>

// Needed because the Node creates a blank one on construction
KalmanFilter::KalmanFilter()
{
    initalized = false;
}

KalmanFilter::KalmanFilter(long int state_size, Eigen::MatrixXd Q)
{
    initalized = true;
    this->state_size = state_size;
    this->Q = Q;
    if (Q.cols() != state_size && Q.rows() != state_size)
    {
        throw std::invalid_argument("Q is not a State_Size x State_Size matrix!");
    }
}

KalmanFilter::~KalmanFilter()
{
}

kalman_pair KalmanFilter::predict(kalman_pair input, Eigen::MatrixXd F, rclcpp::Node *node)
{
    if (!initalized)
    {
        throw std::invalid_argument("Kalman was not properly created");
    }
    Eigen::MatrixXd state = input.state;
    Eigen::MatrixXd covariance = input.covariance;

    // Error checking
    if (state.cols() != 1 && state.rows() != state_size)
    {
        throw std::invalid_argument("State is not a State_Size x 1 matrix!");
    }
    if (covariance.cols() != state_size && covariance.rows() != state_size)
    {
        throw std::invalid_argument("Covariance is not a State_Size x State_Size matrix!");
    }

    if (F.cols() != state_size && F.rows() != state_size)
    {
        throw std::invalid_argument("Covariance is not a State_Size x State_Size matrix!");
    }
    // RCLCPP_INFO(node->get_logger(), "Updating State");
    state = F * state;
    // RCLCPP_INFO(node->get_logger(), "Updating Covariance");
    covariance = ((F * covariance) * F.transpose()) + Q;

    // RCLCPP_INFO(node->get_logger(), "Updating Outputs");
    kalman_pair output;
    output.state = state;
    output.covariance = covariance;

    return output;
}

void KalmanFilter::refine(kalman_pair &input, std::vector<kalman_pair> observations, Eigen::MatrixXd sensor_transform, rclcpp::Node *node)
{
    if (!initalized)
    {
        throw std::invalid_argument("Kalman was not properly created");
    }
    Eigen::MatrixXd X = input.state;
    Eigen::MatrixXd P = input.covariance;
    // Error Checking
    if (X.cols() != 1 && X.rows() != state_size)
    {
        throw std::invalid_argument("X is not a State_Size x 1 matrix!");
    }
    if (P.cols() != state_size && P.rows() != state_size)
    {
        throw std::invalid_argument("P is not a State_Size x State_Size matrix!");
    }
    if (sensor_transform.cols() != state_size && sensor_transform.rows() != state_size)
    {
        throw std::invalid_argument("P is not a State_Size x State_Size matrix!");
    }
    for (unsigned long int i = 0; i < observations.size(); i++)
    {
        if (observations[i].state.cols() != 1 && observations[i].state.rows() != state_size)
        {
            throw std::invalid_argument("An X_obs is not a State_Size x 1 matrix!");
        }
        if (observations[i].covariance.cols() != state_size && observations[i].covariance.rows() != state_size)
        {
            throw std::invalid_argument("An P_obs is not a State_Size x State_Size matrix!");
        }
    }
    
    unsigned long int num_obs = observations.size();
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(state_size, num_obs * state_size);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(num_obs * state_size, num_obs * state_size); // Observed Covariance
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_obs * state_size, state_size);           // Sensor Transform Matrix
    Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(num_obs * state_size, 1);                    // Observed States

    // Populate Matrices with Required Data
    for (unsigned long int i = 0; i < num_obs; i++)
    {
        Z.block(i * state_size, 0, state_size, 1) = observations[i].state;
        R.block(state_size * i, state_size * i, state_size, state_size) = observations[i].covariance;
        H.block(state_size * i, 0, state_size, state_size) = sensor_transform;
    }
    // RCLCPP_INFO(node->get_logger(), "Data Populated");
    // Refine State
    Eigen::MatrixXd H_t = H.transpose();
    // RCLCPP_INFO(node->get_logger(), "K");
    Eigen::MatrixXd S = (((H * P) * H_t) + R);
    Eigen::MatrixXd S_inv = S.inverse();
    K = (P * H_t) * S_inv;


    //log_2Dmatrix(node, P * H_t);
    //log_2Dmatrix(node, (H * P));
    //log_2Dmatrix(node, (H * P) * H_t);
    //log_2Dmatrix(node, S);
    //log_2Dmatrix(node, P);
    //log_2Dmatrix(node, H);
    //log_2Dmatrix(node, R);
    //log_2Dmatrix(node, S_inv);
    //RCLCPP_INFO(node->get_logger(), "Below Should be Identity Matrix");
    //log_2Dmatrix(node, ((((H * P) * H_t) + R).inverse()) * (((H * P) * H_t) + R));
    
    //log_2Dmatrix(node, K);
    //RCLCPP_INFO(node->get_logger(), "(Z - (H * X))");
    //log_2Dmatrix(node, (Z - (H * X)));
    //RCLCPP_INFO(node->get_logger(), "K * (Z - (H * X))");
    //log_2Dmatrix(node, (K * (Z - (H * X))));
    //RCLCPP_INFO(node->get_logger(), "X");
    X = X + (K * (Z - (H * X)));
    //log_2Dmatrix(node, X);
    //RCLCPP_INFO(node->get_logger(), "P");
    P = (Eigen::MatrixXd::Identity(state_size, state_size) - (K * H)) * P;
    //log_2Dmatrix(node ,P);

    // Ensure Covariance Matrix does not converge
    for (int i = 0; i < state_size; i++)
    {
        for (int j = 0; j < state_size; j++)
        {
            if (P(i, j) < 0.000001)
            {
                P(i, j) += 0.000001;
            }
        }
    }
    
    // RCLCPP_INFO(node->get_logger(), "Updating Outputs");
    input.state = X;
    input.covariance = P;
    // RCLCPP_INFO(node->get_logger(), "Updated Outputs");
}
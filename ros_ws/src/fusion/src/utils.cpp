#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"
#include <Eigen/Dense>

double time_to_double(builtin_interfaces::msg::Time t){
    return (double)t.sec + t.nanosec/((double)1000000000);
}

double time_dif_double(builtin_interfaces::msg::Time t1, builtin_interfaces::msg::Time t2){
    return time_to_double(t1) - time_to_double(t2);
}

builtin_interfaces::msg::Time double_to_time(double t){
    int s = (int)t;
    int n = ((t - ((int)t))*1000000000);
    builtin_interfaces::msg::Time newTime;
    newTime.sec = s;
    newTime.nanosec = n;
    return newTime;
}

// https://en.wikipedia.org/wiki/Geographical_distance
double dist_between_two_points_gps(double lat1, double lon1, double lat2, double lon2){
    double R = 6371009; // Radius of Earth in Meters
    double PI = 3.14159265359;
    double deltaLat = (lat1 - lat2) / 180 * PI;
    double deltaLon = (lon1 - lon2) / 180 * PI;

    double avgLat = (lat1 +lat2)/2 / 180 * PI;
    double res1 = pow(2*sin(deltaLat/2)*cos(deltaLon/2),2);
    double res2 = pow(2*cos(avgLat)*sin(deltaLon/2),2);


    return R * pow(res1+res2, 0.5);

}

void cov_array_to_matrix(std::array<double, 16> &array, Eigen::Matrix<double, 4, 4> &mat){
     for(int i = 0; i < 16; i++){
          (mat)(i/4, i%4) = (array)[i];
     }
}

void cov_matrix_to_array(std::array<double, 16> &array, Eigen::Matrix<double, 4, 4> &mat){
     for(int i = 0; i < 16; i++){
          (array)[i] = (mat)(i/4, i%4);
     }
}

void log_2Dmatrix(rclcpp::Node *node, Eigen::MatrixXd mat){
    std::ostringstream ss;
    ss << std::endl << mat << std::endl;
    RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
}
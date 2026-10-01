#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <cmath>
#include <chrono>

#include <Eigen/Dense>

#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "fusion/msg/track.hpp"
#include "fusion/msg/vehicle_dynamics.hpp"
#include "fusion/msg/remote_object_state.hpp"
#include "fusion/msg/tracked_object.hpp"
#include "fusion/msg/tracked_object_list.hpp"
#include "fusion/msg/gps.hpp"
#include "fusion/msg/vehicle_command.hpp"
#include "fusion/msg/triple_vector_wps.hpp"

#include "std_msgs/msg/u_int32.hpp"

#include "rclcpp/time.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "utils.hpp"
#include "kalman.hpp"
#include "./codegen/lib/WayPtsAhead2/spline.h"
#include "./codegen/lib/WayPtsAhead2/WayPtsAhead2.h"
#include "./codegen/lib/NonlinCircleFit/NonlinCircleFit.h"

#include <vector>      
#include <string>     
#include <fstream>   
#include <sstream>   
#include <iostream>   
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/float64.hpp>
#define _USE_MATH_DEFINES
#include <iomanip>   // For setting precision (setprecision)

using std::placeholders::_1;
using namespace std::chrono_literals;

int object_count = 0;

builtin_interfaces::msg::Time last_time;
unsigned long int track_ids = 0;

static bool USE_SIM_TIME = false;

std::vector<double> plan_x;
std::vector<double> plan_y;
std::vector<double> plan_h;
bool path_valid = true;

bool use_path_file = true;
std::string path_file_name = "closed_track_speed_test_traj";

#include <cmath>

std::vector<double> convert_xy_to_lat_lon(double ref_lat_rad, double ref_lon_rad, double ref_heading_rad, double x_m, double y_m) {
    const double f = 0.003353;
    const double a = 6378137.0;
    double f1 = std::sqrt(f * (2.0 - f));
    double sin_lat = std::sin(ref_lat_rad);
    double denom = std::sqrt(1.0 - (f1 * f1) * (sin_lat * sin_lat));
    double f2 = a * (1.0 - f1 * f1) / (denom * denom * denom);
    double f3 = a / denom;
    double N = x_m * std::cos(ref_heading_rad) - y_m * std::sin(ref_heading_rad);
    double E = x_m * std::sin(ref_heading_rad) + y_m * std::cos(ref_heading_rad);
    double lat_rad = ref_lat_rad + N / f2;
    double lon_rad = ref_lon_rad + E / (f3 * std::cos(ref_lat_rad));
    return {lat_rad, lon_rad};;
}


std::vector<double> convert_lat_lon_to_xy(double ref_lat_rad, double ref_lon_rad, double ref_heading_rad, double lat_rad, double lon_rad) {
    /**
     * Computes xy distances between two sets of lat/lon
     */
    double f = 0.003353;
    double a = 6378137;
    double f1 = std::sqrt(f * (2 - f));
    double f2 = a * (1 - std::pow(f1, 2)) / std::pow((1 - std::pow(f1, 2) * std::pow(std::sin(ref_lat_rad), 2)), 1.5);
    double f3 = a / std::sqrt(1 - std::pow(f1, 2) * std::pow(std::sin(ref_lat_rad), 2));
    double E = f3 * std::cos(ref_lat_rad) * (lon_rad - ref_lon_rad);
    double N = f2 * (lat_rad - ref_lat_rad);
    double x_m = N * std::cos(ref_heading_rad) + E * std::sin(ref_heading_rad);
    double y_m = -N * std::sin(ref_heading_rad) + E * std::cos(ref_heading_rad);
    double d2d_m = std::sqrt(std::pow(x_m, 2) + std::pow(y_m, 2));
    
    return {x_m, y_m, d2d_m};
}

double wrapAngleTo2Pi(double angle) {
    const double twoPi = 2.0 * M_PI;
    double wrappedAngle = fmod(angle, twoPi);
    if (wrappedAngle < 0) {
        wrappedAngle += twoPi;
    }
    return wrappedAngle;
}

double wrapAngleToPi(double angle) {
    angle = fmod(angle, 2 * M_PI);

    if (angle >= M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

void printArray(double arr[], int size) {
    for (int i = 0; i < size; ++i) {
        std::cout << arr[i] << " ";
    }
    std::cout << std::endl;
}

class ControlGenerator : public rclcpp::Node
{

public:
    ControlGenerator() : Node("ControlGen2")
    {
        this->declare_parameter<std::string>("mode_arg", "real");
        std::string mode_arg = this->get_parameter("mode_arg").as_string();
        if (mode_arg == "real") {
            RCLCPP_INFO(this->get_logger(), "Mode is real2");
            USE_SIM_TIME = false;
        }
        if (use_path_file == true) {
            load_path_from_csv("/home/connau/ConnAu/Data/offline_path_files/" + path_file_name + ".csv");
            std::ifstream f("/home/connau/ConnAu/Data/offline_path_files/" + path_file_name + "_info.json");
            if (!f.is_open()) {
                std::cerr << "Error: Could not open path info json file." << std::endl;
                return;
            }
            nlohmann::json data = nlohmann::json::parse(f);
            std::cout << "lat from info file: " << data["p1_lat"] << std::endl;
            p1_lat = data["p1_lat"];
            p1_lon = data["p1_lon"];
        } else {
            subscription_ = this->create_subscription<fusion::msg::TripleVectorWps>("interpolated_wps", 100, 
            std::bind(&ControlGenerator::app_wp_callback, this, _1));
        }


        // rclcpp::on_shutdown(std::bind(&ControlGenerator::cleanup_loop, this)); 

        objects = new std::map<int, fusion::msg::TrackedObject>;
        gps_sub = this->create_subscription<fusion::msg::GPS>("gps", 100, std::bind(&ControlGenerator::gps_callback, this, _1));
        gps_sub_can = this->create_subscription<fusion::msg::GPS>("gps_can", 100, std::bind(&ControlGenerator::gps_can_callback, this, _1));
        // path_sub = this->create_subscription<std_msgs::msg::UInt32>("PathID", 100, std::bind(&ControlGenerator::update_path, this, _1));
        object_sub = this->create_subscription<fusion::msg::TrackedObjectList>("FinalObjectList", 100, std::bind(&ControlGenerator::update_objects, this, _1));
        ego_cmd_pub = this->create_publisher<fusion::msg::VehicleCommand>("vehicle_command", 100);
        speed_cmd_sub = this->create_subscription<std_msgs::msg::Float64>("ego_speed_cmd", 10, std::bind(&ControlGenerator::speed_cmd_callback, this, _1)); //
        // speed_cmd_sub = this->create_subscription<std_msgs::msg::Float64>(
        // "ego_speed_cmd", 10,
        // [this](const std_msgs::msg::Float64::SharedPtr msg) {
        //     speed_cmd_callback(msg);
        // });
        tick_timer = this->create_wall_timer(100ms, std::bind(&ControlGenerator::tick, this));
        curGPS = fusion::msg::GPS();
        curGPS_CAN = fusion::msg::GPS();
    }

private:

    void gps_callback(fusion::msg::GPS g){
        curGPS = g;
        // RCLCPP_INFO(this->get_logger(), "lat lon is: %f, %f", curGPS.latitude, curGPS.longitude);
        if(!curGPS.valid){
            RCLCPP_INFO(this->get_logger(), "Invalid GPS position received, please investigate, exiting program!");
        }
        gps_speed = curGPS.speed_f;
        // exit(1);
    }   

    void gps_can_callback(fusion::msg::GPS g){
        curGPS_CAN = g;
        // RCLCPP_INFO(this->get_logger(), "lat lon is: %f, %f", curGPS.latitude, curGPS.longitude);
        if(!curGPS_CAN.valid){
            RCLCPP_INFO(this->get_logger(), "Invalid CAN GPS position received, please investigate, exiting program!");
        }
        can_speed = curGPS_CAN.speed;
        // exit(1);
    }

    void update_objects(fusion::msg::TrackedObjectList list){
        /*for(long unsigned int i = 0; i < list.objs.size(); i++){
            if(list.objs.find(list.objs.begin(), list.objs.end(), list.objs[i].object_id) != list.objs.end()){
                // Create a new object in the map
            }else{
                // Update the object map
            }
        }*/
    }

     void speed_cmd_callback(std_msgs::msg::Float64::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received float data: '%f'", msg->data); //
        speed_cmd = msg->data;
    }
    
    /**
     * Takes in position, and heading in combination with the map and calculates the required road wheel angle
     * @param time_from - double,  the current time
     * @param time_to - double, the future time stamp to take action through
     * @param pos_x - double, Current position x
     * @param pos_y - double, Current position y
     * @param heading - double, Current heading
     * @param speed - double, Current Speed
     * @return swa_d - float, The required steering wheel angle for a Hummer EV SUV
     */
    double steering_wheel_angle_generation(double time_from, double time_to, double pos_x, double pos_y, double heading, double speed){
        
        // Initalize variables as required by WayPointsAhead
        double hwp[50];
        double hwp2[50];
        double xwp[50];
        double xwp2[50];
        double ywp[50];
        double ywp2[50];
        double dis;
        double imode;
        double localx = pos_x;
        double localy = pos_y;
        double localh = heading;
        double delta = speed * (time_to - time_from);
        unsigned short b_index;

        // Some known values that can be used to compare against the Matlab version
        //double localx = 20.2707;
        //double localy = 79.2488;
        //double localh = 1.5780;
        //double delta = 1.0;
        WayPtsAhead2(localx, localy, localh, delta, plan_x.data(), plan_y.data(), plan_h.data(), int(plan_x.size()), &dis, &b_index, &imode, xwp, ywp, hwp, xwp2, ywp2, hwp2);
        // printArray(ywp2, 50);
        // Configure parameters for NonLinCircFit
        
        double* desPathX = xwp2;
        double* desPathY = ywp2;
        double* desPhi = hwp2;

        double weight[50];
        for(int i = 0; i < 50; i++){
            weight[i] = 1.0;
        }

        //double a = 1.2856; // Distance CG-front Axel in Meters - Maybe for Volt - Check with Nikolai
        //double b = 1.5244; // Distance CG-rear axel in Meters - Maybe for Volt - Check with Nikolai

        double a=1.67; //%[m] CG to front axle for Hummer
        double b=1.51; //%[m] CG to rear axle for Hummer

        double veh_param[2];
        veh_param[0] = a;
        veh_param[1] = b;
        double MnvrActive = 1.0;
        double TimeInMnvr = 1.0;
        double tanRWA = 0.0;
        double tanRWA_prev = 0.0;
        double PPursuit_tun1 = 0.6;
        double L_preview = 2.0 * 2.5;
        double brent_tol = 0.0025;
        double brent_itmax = 10;
        double brent_zeps = 1.0e-6;
        double wdist = 1.0;
        double wangle = 1.0;
        double method4ini = 1.0;

        double Nwp1;
        double brent_valid;
        double bx;
        double cost;
        double cost_dd0_2;
        double cost_deriv;
        double costfun_value;
        double j_preview;
        double number_iter;
        double tandelta_NF;
        double tandelta_PP;
        double tandelta_PP2;
        
        NonlinCircleFit(
            MnvrActive, TimeInMnvr, tanRWA, tanRWA_prev,
            desPathX, desPathY, desPhi, PPursuit_tun1, L_preview,
            veh_param, brent_tol, brent_itmax, brent_zeps, wdist,
            wangle, weight, method4ini, &tandelta_NF, &costfun_value,
            &number_iter, &brent_valid, &j_preview, &delta, &tandelta_PP, &cost,
            &cost_deriv, &Nwp1, &bx, &cost_dd0_2, &tandelta_PP2);
        
        double rwa_f = atan(tandelta_NF) * 1.0; // Road Wheel Angle

        
        //Outputs
        //RCLCPP_INFO(this->get_logger(), "RWA: %lf\n", rwa_f);
        // std::cout << "rwa (rad): " << -rwa_f << std::endl;

        double swa_rad = (rwa_f * 18.33);
        // RCLCPP_INFO(this->get_logger(), "rv angle rad: %f", rwa_f);

        double swa_deg = swa_rad * 180 / 3.1415926535;
        return swa_deg;
    }


    
    // void update_path(std_msgs::msg::UInt32 path_id)
    // {
    //     if(path_id.data == 0){
    //         std::memcpy(plan_x, plan_sl_x, 298*sizeof(double));
    //         std::memcpy(plan_y, plan_sl_y, 298*sizeof(double));
    //         std::memcpy(plan_h, plan_sl_h, 298*sizeof(double));
    //         path_valid = true;
    //     }else{
    //         path_valid = false;
    //         RCLCPP_INFO(this->get_logger(), "ERROR: Path plan id is not valid!");
    //     }
    // }

    void tick()
    {   

        if(curGPS_CAN.valid){
            if (curGPS_CAN.heading_valid) {
                // localh = wrapAngleToPi(curGPS_CAN.heading-M_PI);
                localh = curGPS_CAN.heading_f;
                localh = wrapAngleTo2Pi(-localh + M_PI/2);
            } else {
                if (localh == 0.0) {
                    localh = M_PI;
                }
            }
        } 
        // else{
        //     return;
        // }

        // If plan not defined, we should stop the vehicle
        // Plan is valid if all var's size are equal and greater than 1
        //plan_valid = plan_x.size() == plan_h.size() && plan_h.size() == plan_y.size() && plan_x.size() > 1;
        if (!path_valid){
            
                RCLCPP_INFO(this->get_logger(), "ERROR: Path plan either is not yet established or is not valid!");
        }
        else{
            localx = curGPS.latitude;
            localy = curGPS.longitude;
            // localx = 42.5201539119625;
            // localy = -83.04390085925229;
            if (p1_lat == -1 || p1_lon == -1) {return;}
            // std::vector<double> ego_c_latlon = convert_xy_to_lat_lon(curGPS.latitude * (M_PI)/180, curGPS.longitude * (M_PI)/180, curGPS_CAN.heading, ego_x_gps_offset, 0);
            // localx = ego_c_latlon[0] * 180/M_PI;
            // localy = ego_c_latlon[1] * 180/M_PI;
            std::vector<double> ego_pos = convert_lat_lon_to_xy(p1_lat * (M_PI)/180, p1_lon * (M_PI)/180, 0, localx * (M_PI)/180, localy * (M_PI)/180);
            // ego_pos[0] += ego_x_gps_offset*std::cos(localh);
            // ego_pos[1] += ego_x_gps_offset*std::sin(localh);
            speed = std::max(1.0, gps_speed);
            swa_deg = steering_wheel_angle_generation(0.0,0.1,ego_pos[1], ego_pos[0], localh, speed);
            // swa_deg = -swa_deg;
            RCLCPP_INFO(this->get_logger(), "steering cmd deg: %f", swa_deg);
            // std::cout << "steer (deg): " << swa_deg << std::endl;
            // std::cout << localx << ", " << localy << ", " << localh << ", " << swa_deg << ", " << std::endl;
            RCLCPP_INFO(this->get_logger(), "ego pos x&y: %f, %f, %f, %f", ego_pos[1], ego_pos[0], localh, curGPS.heading);
            // RCLCPP_INFO(this->get_logger(), ", %f, %f, %f", ego_pos[0], ego_pos[1], localh);
            // RCLCPP_INFO(this->get_logger(), "ego lat lon: %f, %f", localx, localy);
            int shifter_val = 1;
            // double dist_to_end_wp = std::sqrt(std::pow(wp_end_x - ego_pos[0], 2) + std::pow(wp_end_y - ego_pos[1], 2));
            // if (dist_to_end_wp < 0.3 || is_justin_wp_end) {
            //     speed = 0.0;
            //     swa_deg = 0.0;
            //     shifter_val = 0;
            //     is_justin_wp_end = true;
            // }
            // RCLCPP_INFO(this->get_logger(), "dist to end wp: %f", dist_to_end_wp);
            fusion::msg::VehicleCommand cmd_msg;
            cmd_msg.long_dir_rq = shifter_val;
            cmd_msg.strgwhlang_rq = swa_deg;
            cmd_msg.velocity_rq = speed_cmd;
            cmd_msg.brake_rq = 0;
            cmd_msg.brake_hold_rq = false;
            ego_cmd_pub->publish(cmd_msg);
        }
    }

    bool load_path_from_csv(const std::string &filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            // RCLCPP_ERROR(this->get_logger(), "failed to open CSV file: %s", filepath.c_str());
            return false;
        }
        plan_x.clear();
        plan_y.clear();
        plan_h.clear();
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str, h_str;

            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, h_str, ',');

            try {
                plan_x.push_back(std::stod(x_str));
                plan_y.push_back(std::stod(y_str));
                plan_h.push_back(std::stod(h_str));
            } catch (const std::exception &e) {
                // RCLCPP_WARN(this->get_logger(), "skipping malformed line: %s", line.c_str());
                continue;
            }
        }
        if (plan_x.size() > 1 && plan_x.size() == plan_y.size() && plan_x.size() == plan_h.size()) {
            path_valid = true;
            // RCLCPP_INFO(this->get_logger(), "successfully loaded %zu path points", plan_x.size());
            return true;
        }
        // RCLCPP_ERROR(this->get_logger(), "invalid or mismatched data in CSV");
        path_valid = false;
        return false;
    }

    void app_wp_callback(const fusion::msg::TripleVectorWps::SharedPtr msg)
    {
        plan_x = msg->plan_x;
        plan_y = msg->plan_y;
        plan_h = msg->plan_h;

        wp_end_x = plan_x.back();
        wp_end_y = plan_y.back();

        p1_lat = curGPS.latitude;
        p1_lon = curGPS.longitude;

        std::ifstream inputFile("/home/connau/srp/birdview/hummer_path/gps_ref_p2.txt");

        if (!inputFile.is_open()) {
            std::cerr << "Error: Could not open the file 'gps_ref_p2.txt'\n";
        }

        std::string line; 
        double latitude, longitude; 

        if (std::getline(inputFile, line)) {
            line.erase(0, line.find('[') + 1); 
            line.erase(line.find(']'));

            std::stringstream ss(line);
            std::string latitude_str, longitude_str;

            if (std::getline(ss, latitude_str, ',') && (ss >> longitude)) { 
                latitude = std::stod(latitude_str);

                std::cout << "Latitude: " << std::fixed << std::setprecision(6) << latitude << std::endl; 
                std::cout << "Longitude: " << std::fixed << std::setprecision(6) << longitude << std::endl;
            } else {
                std::cerr << "Error: Invalid format in the file.\n";
            }
        } else {
            std::cerr << "Error: Empty file or unable to read a line.\n";
        }
        p1_lat = latitude;
        p1_lon = longitude;

        inputFile.close();

        // std::vector<double> ego_c_latlon = convert_xy_to_lat_lon(curGPS.latitude * (M_PI)/180, curGPS.longitude * (M_PI)/180, curGPS_CAN.heading, ego_x_gps_offset, 0);
        // p1_lat = ego_c_latlon[0] * 180/M_PI;
        // p1_lon = ego_c_latlon[1] * 180/M_PI;
        
        // RCLCPP_INFO(this->get_logger(), "Received list1[0]=%f", plan_x[0]);
    }

    
    std::map<int, fusion::msg::TrackedObject> *objects;
    
    // justin ## make it slow for temp test 0.5m/s
    // double localx, localy, localh=0.0, speed=3.0, swa_deg, p1_lat=-1, p1_lon=-1;

    double localx, localy, localh=0.0, speed=1.0, speed_cmd = 2.0, gps_speed = 0.0, can_speed = 0.0, prev_speed=0, swa_deg, p1_lat=-1, p1_lon=-1, wp_end_x = 0, wp_end_y = 0;
    bool is_justin_wp_end = false;
    
    fusion::msg::GPS curGPS, curGPS_CAN;
    // double ego_x_gps_offset = 2.39649;
    int ego_x_gps_offset = 0.0;

    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr path_sub;
    rclcpp::Subscription<fusion::msg::TrackedObjectList>::SharedPtr object_sub;
    rclcpp::Subscription<fusion::msg::GPS>::SharedPtr gps_sub, gps_sub_can;
    rclcpp::Subscription<fusion::msg::TripleVectorWps>::SharedPtr subscription_;
    rclcpp::Publisher<fusion::msg::VehicleCommand>::SharedPtr ego_cmd_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_cmd_sub;
    rclcpp::TimerBase::SharedPtr tick_timer;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
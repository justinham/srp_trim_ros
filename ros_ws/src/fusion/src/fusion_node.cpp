#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <cmath>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "fusion/msg/track.hpp"
#include "fusion/msg/vehicle_dynamics.hpp"
#include "fusion/msg/remote_object_state.hpp"
#include "fusion/msg/tracked_object.hpp"
#include "fusion/msg/tracked_object_list.hpp"

#include "rclcpp/time.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "utils.hpp"
#include "kalman.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

int object_count = 0;

const static int NUM_SENSORS = 4;
const static double COST_THRESHOLD = 5.0;
const static double MERGE_THRESHOLD = 5.0;
const static long int STATE_SIZE = 4;

static bool USE_SIM_TIME = false;
static bool LOG_GT = true;

builtin_interfaces::msg::Time last_time;
unsigned long int track_ids = 0;

typedef Eigen::Matrix<double, STATE_SIZE, 1> State;
typedef Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Covariance;

double compute_cost(fusion::msg::RemoteObjectState s, fusion::msg::Track obs)
{
    double dist = sqrt(pow(s.lat - obs.lat, 2) + pow(s.lon - obs.lon, 2));
    return dist;
}

class Fusion : public rclcpp::Node
{

public:
    Fusion() : Node("Fusion")
    {   
        // Internal data storage
        track_buf = new std::vector<fusion::msg::Track>;
        objects = new std::map<int, fusion::msg::TrackedObject>;
        
        // Sets up the ROS pubs, subs, and timer
        fusion_timer = this->create_wall_timer(100ms, std::bind(&Fusion::fusion_callback, this));
        track_subscription = this->create_subscription<fusion::msg::Track>("track", 100, std::bind(&Fusion::track_callback, this, _1));
        visual_object_pub = this->create_publisher<visualization_msgs::msg::Marker>("object_marker", 50);
        tracked_object_list_pub = this->create_publisher<fusion::msg::TrackedObjectList>("tracked_object_list", 100);
        sim_ego_state_sub = this->create_subscription<fusion::msg::RemoteObjectState>("ego", 100, std::bind(&Fusion::SIM_EGO_STATE_callback, this, _1));
        gt_state_sub = this->create_subscription<fusion::msg::TrackedObjectList>("GT", 100, std::bind(&Fusion::GT_callback, this, _1));
        
        // Sets up Kalman Filters 
        Q_theta = Eigen::MatrixXd(1,1);
        Q_theta(0,0) = 1.0;
        theta_kalman = KalmanFilter(1, Q_theta);
        Eigen::MatrixXd Q_state(4,4);
        for (int i = 0; i < STATE_SIZE; i++)
        {
            for (int j = 0; j < STATE_SIZE; j++)
            {
                Q_state(i, j) = 0.2;
            }
        }
        Q_state += Eigen::MatrixXd::Identity(4,4);
        Q_state.block(2,2,2,2) += 2 * Eigen::MatrixXd::Identity(2,2);
        state_kalman = KalmanFilter(STATE_SIZE, Q_state);

        RCLCPP_INFO(this->get_logger(), ",time,Obj id,Obj type,Lat,Lon,Theta,Lat_Vel,Lon_Vel,Lat Cov,Lon Cov,Lat_Vel Cov,Lon_Vel Cov,GT Lat,GT Lon,GT Lat_Vel,GT Lon_Vel,Error Lat,Error Lon,Error Lat_Vel,Error Lon_Vel,Theta,Error Theta,Theta Valid");

    }

private:


    /**
     * Kalman Predict
     * Predicts an objects state at a specified timestamp
     * @param s, RemoteObjectState - Object to have its state predicted
     * @param req_t, Time - Time Object containing the time information
     * @return RemoteObjectState - the Updated state information for the object
     */
    fusion::msg::RemoteObjectState kalman_predict(fusion::msg::RemoteObjectState s, builtin_interfaces::msg::Time req_t)
    {
        State old_state(s.lat, s.lon, s.lat_vel, s.lon_vel);
        Covariance old_cov;
        cov_array_to_matrix(s.covariance, old_cov);
        double delta_t = time_to_double(req_t) - time_to_double(s.timestamp);
        Eigen::Matrix<double, 4, 4> Fx;
        Fx << 1, 0, delta_t, 0,
            0, 1, 0, delta_t,
            0, 0, 1, 0,
            0, 0, 0, 1;

        kalman_pair input;
        input.state = old_state;
        input.covariance = old_cov;

        kalman_pair res = state_kalman.predict(input, Fx, this);

        fusion::msg::RemoteObjectState response;
        Covariance r = res.covariance;
        cov_matrix_to_array(response.covariance, r);
        response.lat = res.state(0);
        response.lon = res.state(1);
        response.lat_vel = res.state(2);
        response.lon_vel = res.state(3);


        // Handle Theta Seperatly
        response.theta = s.theta;
        response.theta_cov = s.theta_cov;
        response.theta_valid = s.theta_valid;

        response.timestamp = req_t;
        response.object_id = s.object_id;
        response.object_type = s.object_type;
        return response;
    }

    /**
     * Kalman Refine
     * Refines the state estimation of the object using the provided observations
     * @param s, RemoteObjectState - Object to have its state refined by the observations
     * @param observations, vector<Track> - The tracks that will be used to refine the state estimation of the object
     */
    void kalman_refine(fusion::msg::RemoteObjectState &s, std::vector<fusion::msg::Track> &observations)
    {
        Covariance P;
        cov_array_to_matrix(s.covariance, P); // Predicted Covariance
        State X(s.lat, s.lon, s.lat_vel, s.lon_vel); // Predicted State
        kalman_pair input;
        input.state = X;
        input.covariance = P;
        unsigned long int num_obs = observations.size();
        std::vector<kalman_pair> obs;
        std::vector<kalman_pair> theta_obs;
        for (unsigned long int i = 0; i < num_obs; i++)
        {
            State o;
            o(0,0) = observations[i].lat;
            o(1,0) = observations[i].lon;
            o(2,0) = observations[i].lat_vel;
            o(3,0) = observations[i].lon_vel;

            Covariance o_cov;
            cov_array_to_matrix(observations[i].covariance, o_cov);
            kalman_pair o_pair;
            o_pair.state = o;
            o_pair.covariance = o_cov;
            obs.push_back(o_pair);

            // If theta is valid create an pair to refine our estimate
            if(observations[i].theta_valid){
                Eigen::MatrixXd t(1,1);
                t(0,0) = observations[i].theta;
                

                int factor = int(t(0,0)/(M_PI*2));
                if(t(0,0)< 0){
                    factor--;
                }
                // Brings it to within (0-2PI)
                t(0,0) -= factor*M_PI*2;
                //RCLCPP_INFO(this->get_logger(), "Theta Orig: %lf, Theata after bringing between 0-2Pi: %lf", observations[i].theta, t(0,0));
                // Ensure we do not mess-up theta by having 0 and 350 result in 175
                if(s.theta_valid){
                    if(s.theta - t(0,0) > M_PI){
                        t(0,0) += (2*M_PI);
                    }else if(s.theta - t(0,0) < -M_PI){
                        t(0,0) -= (2*M_PI);
                    }
                }
                //RCLCPP_INFO(this->get_logger(), "Theta State: %lf, Theta Orig: %lf, Theata Used after conversion: %lf, Sensor %d, Theta_Valid %d, Theta_Cov %lf", s.theta, observations[i].theta, t(0,0), observations[i].sensor_id, observations[i].theta_valid, observations[i].theta_cov);
                Eigen::MatrixXd t_cov(1,1);
                t_cov(0,0) = observations[i].theta_cov;
                kalman_pair p;
                p.state = t;
                p.covariance = t_cov;
                theta_obs.push_back(p);
            }
        }
        state_kalman.refine(input, obs, Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE), this);
        s.lat = input.state(0);
        s.lon = input.state(1);
        s.lat_vel = input.state(2);
        s.lon_vel = input.state(3);
        Covariance i = input.covariance;
        cov_matrix_to_array(s.covariance, i);

        // Update our theta
        if(theta_obs.size() > 0){
            kalman_pair theta_input;

            Eigen::MatrixXd t(1,1);
            t(0,0) = s.theta;
            Eigen::MatrixXd t_cov(1,1);
            t_cov(0,0) = s.theta_cov;

            theta_input.state = t;

            // Trust our previous estimates less
            theta_input.covariance = t_cov + Q_theta;
            //RCLCPP_INFO(this->get_logger(), "Before Kalman Theta State: %lf, Before Kalman Theta Cov %lf, state cov: %lf", theta_input.state(0,0), theta_input.covariance(0,0), s.theta_cov);
            theta_kalman.refine(theta_input, theta_obs, Eigen::MatrixXd::Identity(1, 1), this);
            //RCLCPP_INFO(this->get_logger(), "Updated Theta State: %lf, Updated Theta Cov %lf", theta_input.state(0,0), theta_input.covariance(0,0));
            s.theta = theta_input.state(0,0);
            s.theta_cov = theta_input.covariance(0,0);
            s.theta_valid = true;
        }
    }

    /**
     * Visualize Sensor Track
     * Takes a track and creates a set of RVIZ visualization messages to display the track and its relevant data in RVIZ
     * @param t, track - The track to be visualized
     */
    void visualize_sensor_track(const fusion::msg::Track &t) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "/my_frame";
        marker.header.stamp = t.timestamp;

        marker.ns = std::to_string(t.sensor_id);
        marker.id = t.track_id;
        if (t.object_type)
        {
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
        }
        else
        {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 2.0;
            marker.scale.y = 3.0;
            marker.scale.z = 1.0;
        }

        marker.action = 0;

        marker.pose.position.x = -t.lat;
        marker.pose.position.y = t.lon;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        tf2::Quaternion q;

        if (t.theta_valid)
        {
            q.setRPY(0.0, 0.0, t.theta);
        }
        else
        {
            q.setRPY(0.0, 0.0, 0.0);
        }

        marker.pose.orientation.x = q.getX();
        marker.pose.orientation.y = q.getY();
        marker.pose.orientation.z = q.getZ();
        marker.pose.orientation.w = q.getW();

        switch (t.sensor_id)
        {
        case 0:
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            break;
        case 1:
            marker.color.r = 1.0;
            break;
        case 2:
            marker.color.b = 1.0;
            break;
        case 3:
            marker.color.g = 1.0;
            break;
        case 4:
            marker.color.r = 1.0;
            marker.color.b = 1.0;
            break;
        default:
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        }
        marker.color.a = 0.5;

        visual_object_pub->publish(marker);

        visualization_msgs::msg::Marker arrow2;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        arrow2.header.frame_id = "/my_frame";
        arrow2.header.stamp = t.timestamp;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        arrow2.ns = std::to_string(t.sensor_id+10);
        arrow2.id = t.track_id;

        // Set the marker action.  Options are ADD, DELETE, and new in rclcpp Indigo: 3 (DELETEALL)
        arrow2.action = visualization_msgs::msg::Marker::ADD;

        arrow2.lifetime = rclcpp::Duration::from_seconds(0.5);

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        geometry_msgs::msg::Point p1;
        p1.x = t.lat;
        p1.y = t.lon;
        p1.z = 0.0;
        geometry_msgs::msg::Point p2;
        p2.x = t.lat + t.lat_vel;
        p2.y = t.lon+t.lon_vel;
        p2.z = 0.0;
        arrow2.points.push_back(p1);
        arrow2.points.push_back(p2);

        arrow2.scale.x = 1;
        arrow2.scale.y = 1;
        arrow2.scale.z = 0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        arrow2.type = visualization_msgs::msg::Marker::ARROW;
        // RCLCPP_INFO(this->get_logger(), "Arrow: p1.x %lf, p1.y %lf -> p2.x %lf, p2.y %lf", p1.x, p1.y, p2.x, p2.y);
        // Set the color -- be sure to set alpha to something non-zero!
        switch (t.sensor_id)
        {
        case 0:
            arrow2.color.g = 1.0;
            arrow2.color.b = 1.0;
            break;
        case 1:
            arrow2.color.r = 1.0;
            break;
        case 2:
            arrow2.color.b = 1.0;
            break;
        case 3:
            arrow2.color.g = 1.0;
            break;
        case 4:
            arrow2.color.r = 1.0;
            arrow2.color.b = 1.0;
            break;
        default:
            arrow2.color.r = 1.0;
            arrow2.color.g = 1.0;
            arrow2.color.b = 1.0;
        }

        arrow2.color.a = 1.0;

        //visual_object_pub->publish(arrow2);
    }

    /**
     * Track Callback
     * Receives and buffers the track messages
     * @param msg, Track - Track message received from data input sources
     */
    void track_callback(const fusion::msg::Track &msg) const
    {
         //RCLCPP_INFO(this->get_logger(), "Track %d.%d at %d.%09d: Lat: %lf, Lon: %lf, Theta: %lf, Lat_Vel: %lf, Lon_Vel: %lf", msg.sensor_id, msg.track_id, msg.timestamp.sec, msg.timestamp.nanosec, msg.lat, msg.lon, msg.theta, msg.lat_vel, msg.lon_vel);

        // Bring Theta to within set range
        visualize_sensor_track(msg);
        // RCLCPP_INFO(this->get_logger(), "Received Ego Message with timestamp %d.%09d", msg.timestamp.sec, msg.timestamp.nanosec);
            
        (*track_buf).push_back(msg);
        if (USE_SIM_TIME)
        {
            last_time = msg.timestamp;

        }
    }

    void SIM_EGO_STATE_callback(const fusion::msg::RemoteObjectState &s)
    {
        SIM_EGO_STATE = s;
    }

    void GT_callback(const fusion::msg::TrackedObjectList &s){
        GT_STATE = s.objs[0];
    }

    void log_cov(std::array<double, 16> a)
    {

        std::ostringstream ss;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                ss << a[i * 4 + j] << " ";
            }
            ss << std::endl;
        }
        RCLCPP_INFO(this->get_logger(), "Covariance:\n %s", ss.str().c_str());
    }

    void log_state(fusion::msg::RemoteObjectState s)
    {
        RCLCPP_INFO(this->get_logger(), "State: Obj:%d Lat: %lf, Lon: %lf, Theta: %lf, Lat_Vel: %lf, Lon_Vel: %lf", s.object_id, s.lat, s.lon, s.theta, s.lat_vel, s.lon_vel);
        log_cov(s.covariance);
    }

    void log_track(fusion::msg::Track t)
    {
        //RCLCPP_INFO(this->get_logger(), "Track %d.%d at %d.%09d: Lat: %lf, Lon: %lf, Theta: %lf, Lat_Vel: %lf, Lon_Vel: %lf", t.sensor_id, t.track_id, t.timestamp.sec, t.timestamp.nanosec, t.lat, t.lon, t.theta, t.lat_vel, t.lon_vel);
        //log_cov(t.covariance);
    }

    fusion::msg::TrackedObject merge_objects(fusion::msg::TrackedObject obj1, fusion::msg::TrackedObject obj2)
    {
        fusion::msg::TrackedObject result;
        if(USE_SIM_TIME){
        // Merges Ids
            for(unsigned long int i = 0; i < obj1.carla_ids.size(); i++){
                result.carla_ids.push_back(obj1.carla_ids[i]);
            }

            for(unsigned long int i = 0; i < obj2.carla_ids.size(); i++){
                bool found = false;
                for(unsigned long int ind = 0; ind < result.carla_ids.size(); ind++){
                    if(result.carla_ids[ind] == obj2.carla_ids[i]){
                        found = true;
                        break;
                    }
                }

                if(!found){
                    result.carla_ids.push_back(obj2.carla_ids[i]);
                }
            }
        }

        int merged_object_class_count = obj1.object_class_count + obj2.object_class_count;
        result.object_type = obj1.object_type; // Both already have the same type so this logic holds
        // Prevents overflow issues
        if(merged_object_class_count > 100){
            result.object_class_count = 100;
        }

        if(merged_object_class_count < -100){
            result.object_class_count = -100;
        }

        fusion::msg::RemoteObjectState ros1 = obj1.states.back();
        fusion::msg::RemoteObjectState ros2 = obj2.states.back();
        fusion::msg::RemoteObjectState ros3;

        ros3.object_type = result.object_type;

        // Add ids
        if(USE_SIM_TIME){
            for(unsigned long int i = 0; i < result.carla_ids.size(); i++){
                ros3.carla_ids.push_back(result.carla_ids[i]);
            }
        }

        State s1(ros1.lat, ros1.lon, ros1.lat_vel, ros1.lon_vel);
        State s2(ros2.lat, ros2.lon, ros2.lat_vel, ros2.lon_vel);
        //log_state(ros1);
        //log_state(ros2);
        State s_res;

        Covariance cov1;
        Covariance cov2;

        Covariance res;

        cov_array_to_matrix(ros1.covariance, cov1);
        cov_array_to_matrix(ros2.covariance, cov2);
        // Merge the two covariance matrices together
        res = (cov1.inverse() + cov2.inverse()).inverse();

        // Merge the states together
        s_res = res * ((cov1.inverse() * s1) + (cov2.inverse() * s2));

        ros3.lat = s_res[0];
        ros3.lon = s_res[1];
        ros3.theta = (ros1.theta + ros2.theta)/2; //Handle Theta seperatly
        ros3.lat_vel = s_res[2];
        ros3.lon_vel = s_res[3];

        cov_matrix_to_array(ros3.covariance, res);

        ros3.object_id = object_count++;
        ros3.timestamp = ros2.timestamp;

        result.states.push_back(ros3);
        result.theta_valid = (obj1.theta_valid && obj2.theta_valid);
        
        result.delay = obj1.delay - (4 - obj2.delay);
        if (result.delay < 0)
        {
            result.delay = 0;
        }

        result.timestamp = ros3.timestamp;
        result.last_seen = ros3.timestamp;

        result.object_id = ros3.object_id;

        // Color it yellow so we can tell it apart from non-merged tracks
        result.r = 1.0;
        result.g = 1.0;
        result.b = 0.0;

        // Update the sensor track information
        for (long unsigned int i = 0; i < obj1.tracking_sensors.size(); i++)
        {
            result.tracking_sensors.push_back(obj1.tracking_sensors[i]);
            result.tracking_sensors_track_id.push_back(obj1.tracking_sensors_track_id[i]);
        }

        for (long unsigned int i = 0; i < obj2.tracking_sensors.size(); i++)
        {
            result.tracking_sensors.push_back(obj2.tracking_sensors[i]);
            result.tracking_sensors_track_id.push_back(obj2.tracking_sensors_track_id[i]);
        }

        // RCLCPP_INFO(this->get_logger(), "Merged State");
        //log_state(ros3);
        return result;
    }

    void visualize()
    {
        // Currently this is for ego frame of reference
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualize called, number of objects to visualize %ld", (*tracked_objects).size());

        // Add Ego
        visualization_msgs::msg::Marker ego;

        ego.header.frame_id = "/my_frame";
        ego.header.stamp = this->get_clock()->now();

        ego.ns = "EGO";
        ego.id = 0;

        ego.type = 1;
        ego.action = visualization_msgs::msg::Marker::ADD;

        ego.pose.position.x = 0.0;
        ego.pose.position.y = 0.0;
        ego.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0);
        ego.pose.orientation.x = q.getX();
        ego.pose.orientation.y = q.getY();
        ego.pose.orientation.z = q.getZ();
        ego.pose.orientation.w = q.getW();

        ego.scale.x = 2.0574;
        ego.scale.y = 5.38226;

        ego.scale.z = 1.0;

        ego.color.r = 1.0;
        ego.color.g = 1.0;
        ego.color.b = 1.0;
        ego.color.a = 1.0;

        visual_object_pub->publish(ego);

        auto it = objects->begin();
        while (it != objects->end())
        {

            fusion::msg::TrackedObject obj = it->second;

            if (obj.delay != 0)
            {
                it++;
                continue;
            }

            fusion::msg::RemoteObjectState state = obj.states.back();
            visualization_msgs::msg::Marker marker;

            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            marker.header.frame_id = "/my_frame";
            if (USE_SIM_TIME)
            {
                marker.header.stamp = last_time;
            }
            else
            {
                marker.header.stamp = this->get_clock()->now();
            }

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            marker.ns = "object_outline";
            marker.id = obj.object_id;

            // Set the marker action.  Options are ADD, DELETE, and new in rclcpp Indigo: 3 (DELETEALL)
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = -state.lat;
            marker.pose.position.y = state.lon;
            marker.pose.position.z = 0;
            if (state.theta_valid)
            {
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, state.theta);
                marker.pose.orientation.x = q.getX();
                marker.pose.orientation.y = q.getY();
                marker.pose.orientation.z = q.getZ();
                marker.pose.orientation.w = q.getW();
            }
            else
            {
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, 0);
                marker.pose.orientation.x = q.getX();
                marker.pose.orientation.y = q.getY();
                marker.pose.orientation.z = q.getZ();
                marker.pose.orientation.w = q.getW();
            }

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            if (it->second.object_type)
            {
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.scale.z = 1.0;
            }
            else
            {
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.scale.x = 2.0;
                marker.scale.y = 3.0;
                marker.scale.z = 1.0;
            }

            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = obj.r;
            marker.color.g = obj.g;
            marker.color.b = obj.b;
            marker.color.a = 0.5;

            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing an Object %d", obj.object_id);
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object %d Color: %f %f %f", obj.object_id, obj.r, obj.g, obj.b);
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Marker %d Color: %f %f %f", obj.object_id, marker.color.r, marker.color.g, marker.color.b);
            visual_object_pub->publish(marker);

            visualization_msgs::msg::Marker arrow2;

            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            arrow2.header.frame_id = "/my_frame";
            if (USE_SIM_TIME)
            {
                arrow2.header.stamp = last_time;
            }
            else
            {
                arrow2.header.stamp = this->get_clock()->now();
            }

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            arrow2.ns = "object_vel2";
            arrow2.id = obj.object_id;

            // Set the marker action.  Options are ADD, DELETE, and new in rclcpp Indigo: 3 (DELETEALL)
            arrow2.action = visualization_msgs::msg::Marker::ADD;

            arrow2.lifetime = rclcpp::Duration::from_seconds(0.5);
    
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            geometry_msgs::msg::Point p1;
            p1.x = -state.lat; //Accounts for the EGO vehicle having +X as left vs RVIZ where its right
            p1.y = state.lon;
            p1.z = 0.0;
            geometry_msgs::msg::Point p2;
            p2.x = -(state.lat + state.lat_vel); //Accounts for the EGO vehicle having +X as left vs RVIZ where its right
            p2.y = state.lon + state.lon_vel;
            p2.z = 0.0;
            arrow2.points.push_back(p1);
            arrow2.points.push_back(p2);

            arrow2.scale.x = 1;
            arrow2.scale.y = 1;
            arrow2.scale.z = 0;
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            arrow2.type = visualization_msgs::msg::Marker::ARROW;

            // Set the color -- be sure to set alpha to something non-zero!
            arrow2.color.r = 0.5;
            arrow2.color.g = 0.5;
            arrow2.color.b = 0.5;
            arrow2.color.a = 0.5;

            visual_object_pub->publish(arrow2);

            it++;
        }
    }

    void fusion_callback()
    {
        // Bring all tracks to current time
        // RCLCPP_INFO(this->get_logger(), "Fusion Called");
        unsigned long int num_tracks = track_buf->size();
        double cur_time;
        builtin_interfaces::msg::Time time_struct;
        if (USE_SIM_TIME)
        {
            time_struct = last_time;
        }
        else
        {
            time_struct = this->get_clock()->now();
        }
        cur_time = time_to_double(time_struct);
        // RCLCPP_INFO(this->get_logger(), "Fusion Called at %d.%09d or %lf", time_struct.sec, time_struct.nanosec, cur_time);
        // No new tracks
        if (num_tracks == 0)
        {
            // RCLCPP_INFO(this->get_logger(), "No Tracks");

            // Check for expiring Tracks
            std::vector<int> ids_to_erase;
            auto it = objects->begin();
            while (it != objects->end())
            {
                double delta_t2 = time_dif_double(time_struct, it->second.last_seen);
                if (delta_t2 > 0.5)
                {
                    ids_to_erase.push_back(it->second.object_id);
                }
                it++;
            }

            // Purge Expiring Tracks
            for (unsigned long int i = 0; i < ids_to_erase.size(); i++)
            {
                // RCLCPP_INFO(this->get_logger(), "Deleting Object %d", (*objects)[ids_to_erase[i]].object_id);
                objects->erase(ids_to_erase[i]);
            }
            ids_to_erase.clear();
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "Num Tracks %ld", num_tracks);
        // RCLCPP_INFO(this->get_logger(), "Updating Tracks to Current time stamp");

        // Update the tracks to the current time step using kalman filter predict
        for (long unsigned int i = 0; i < num_tracks; i++)
        {
            //log_track((*track_buf)[i]);
            double delta_t = cur_time - time_to_double((*track_buf)[i].timestamp);
            // RCLCPP_INFO(this->get_logger(), "Track Delta_t = %lf", delta_t);
            if (delta_t > 0)
            {
                Eigen::Matrix<double, 4, 4> Fx;
                Fx << 1, 0, delta_t, 0,
                    0, 1, 0, delta_t,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
               
                Covariance old_covariance;
                cov_array_to_matrix((*track_buf)[i].covariance, old_covariance);
                State s;
                s(0) = (*track_buf)[i].lat;
                s(1) =(*track_buf)[i].lon;
                s(2) = (*track_buf)[i].lat_vel;
                s(3) = (*track_buf)[i].lon_vel;

                kalman_pair p;
                p.covariance = old_covariance;
                p.state = s;
                kalman_pair res = state_kalman.predict(p, Fx, this);

                (*track_buf)[i].timestamp = time_struct;
                Covariance new_cov = res.covariance;
                cov_matrix_to_array((*track_buf)[i].covariance, new_cov);
                (*track_buf)[i].lat = res.state(0);
                (*track_buf)[i].lon = res.state(1);
                (*track_buf)[i].lat_vel = res.state(2);
                (*track_buf)[i].lon_vel = res.state(3);

                //log_track((*track_buf)[i]);
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Updating Objects");

        // Take each Object and update to current time stamp
        auto it = objects->begin();
        while (it != objects->end())
        {
            // RCLCPP_INFO(this->get_logger(), "t1 = %lf, t2 = %lf", time_to_double(time_struct), time_to_double(it->second.states.back().timestamp));
            // RCLCPP_INFO(this->get_logger(), "Delta_t = %lf", time_to_double(time_struct) - time_to_double(it->second.states.back().timestamp));
            it->second.states.push_back(kalman_predict(it->second.states.back(), time_struct));
            State s(it->second.states.back().lat, it->second.states.back().lon, it->second.states.back().lat_vel, it->second.states.back().lon_vel);
            bool containsNaN = false;
            for (int j = 0; j < 4; j++)
            {
                if (std::isnan(s[j]))
                {
                    containsNaN = true;
                    break;
                }
            }
            //log_state(it->second.states.back());
            if (containsNaN)
            {

                exit(1);
            }
            it++;
        }

        std::vector<fusion::msg::Track> to_associate;
        // RCLCPP_INFO(this->get_logger(), "Associating Tracks");

        // Utilize track_ids to associate tracks
        for (long unsigned int i = 0; i < track_buf->size(); i++)
        {
            bool found = false;
            fusion::msg::Track t = (*track_buf)[i];
            auto ptr = objects->begin();
            while (ptr != objects->end())
            {
                for (long unsigned int j = 0; j < ptr->second.tracking_sensors.size(); j++)
                {
                    if (ptr->second.tracking_sensors[j] == t.sensor_id && ptr->second.tracking_sensors_track_id[j] == t.track_id)
                    {
                        // RCLCPP_INFO(this->get_logger(), "Matched track to Object %d with state", ptr->second.object_id);
                        //log_state(ptr->second.states.back());
                        //log_track((*track_buf)[i]);

                        // Determine if the track still observes the object
                        double c = compute_cost(ptr->second.states.back(), t);
                        if (c > 1.25 * COST_THRESHOLD)
                        {
                            // RCLCPP_INFO(this->get_logger(), "Matched track %d.%d does not observe assigned obj %d", t.sensor_id, t.track_id, ptr->second.object_id);

                            // Un-Assign the track form the object
                            ptr->second.tracking_sensors.erase(ptr->second.tracking_sensors.begin() + j);
                            ptr->second.tracking_sensors_track_id.erase(ptr->second.tracking_sensors_track_id.begin() + j);
                            continue;
                        }
                        found = true;
                        //kalman_refine(ptr->second.states.back(), t);
                        ptr->second.tracks_to_refine.push_back(t);
                        if (t.theta_valid)
                        {
                            ptr->second.theta_valid = true;
                            ptr->second.states.back().theta_valid = true;
                        }
                        // RCLCPP_INFO(this->get_logger(), "Refined Object %d with state", ptr->second.object_id);
                        // log_state(ptr->second.states.back());
                        if (ptr->second.delay > 0)
                        {
                            ptr->second.delay--;
                        }
                        ptr->second.last_seen = time_struct;
                        if (ptr->second.last_seen != time_struct)
                        {
                            exit(0);
                        }
                        // RCLCPP_INFO(this->get_logger(), ",%d", t.object_type);
                        // Dynamically changes Object type based on observations
                        if(t.object_type){
                            ptr->second.object_class_count = 100;
                            // if(t.sensor_id == 2) {
                            //     ptr->second.object_class_count += 10*t.confidence;
                            //     ptr->second.object_class_count = 200;
                            // } else {
                            //     ptr->second.object_class_count += t.confidence;
                            // }
                        }else{
                            ptr->second.object_class_count = -100;
                            // if(t.sensor_id == 2) {
                            //     ptr->second.object_class_count -= 10*t.confidence;
                            //     ptr->second.object_class_count = -200;
                            // } else {
                            //     ptr->second.object_class_count -= t.confidence;
                            // } 

                        }

                        //  Prevents type flickering by requiring a large shift
                        if(ptr->second.object_class_count < -5){
                            ptr->second.object_type = false;
                        }

                        if(ptr->second.object_class_count > 5){
                            ptr->second.object_type = true;
                        }

                        // Prevents overflow issues
                        if(ptr->second.object_class_count > 100){
                            ptr->second.object_class_count = 100;
                        }

                        if(ptr->second.object_class_count < -100){
                            ptr->second.object_class_count = -100;
                        }

                        break;
                    }
                }
                if (found)
                {
                    break;
                }

                ptr++;
            }
            if (!found)
            {
                to_associate.push_back(t);
            }
        }

        long unsigned int to_associate_size = to_associate.size();

        // RCLCPP_INFO(this->get_logger(), "Assigning Unassociated Tracks");

        // Take the un-assigned tracks and assign them to an object
        for (long unsigned int i = 0; i < to_associate_size; i++)
        {
            int close_object = 0;
            double cost = COST_THRESHOLD + 50; // Just needs to be larger than threshold
            if (objects->size() > 0)
            {
                auto it = objects->begin();
                close_object = it->first;
                cost = compute_cost(it->second.states.back(), to_associate[i]);

                while (it != objects->end())
                {
                    double new_c = compute_cost(it->second.states.back(), to_associate[i]);
                    if (new_c < cost)
                    {
                        cost = new_c;
                        close_object = it->first;
                    }
                    it++;
                }
            }

            // Create a new object as it is not tracking an existing one
            if (cost > COST_THRESHOLD)
            {
                fusion::msg::Track t = to_associate[i];
                fusion::msg::TrackedObject obj;

                // Add carla id
                obj.carla_ids.push_back(t.carla_id);

                obj.object_id = object_count++;
                obj.delay = 4;
                obj.object_type = t.object_type;
                // Dynamically changes Object type based on observations
                if(to_associate[i].object_type){
                    obj.object_class_count += t.confidence;
                }else{
                    obj.object_class_count -= t.confidence;
                }

                fusion::msg::RemoteObjectState st;
                st.lat = t.lat;
                st.lon = t.lon;
                st.theta = t.theta;
                st.lat_vel = t.lat_vel;
                st.lon_vel = t.lon_vel;
                st.covariance = t.covariance;
                st.object_id = obj.object_id;
                st.timestamp = time_struct;
                st.object_type = obj.object_type;
                if(USE_SIM_TIME){
                    st.carla_ids.push_back(t.carla_id);
                }
                if (t.theta_valid)
                {
                    st.theta_valid = true;
                    obj.theta_valid = true;
                }
                obj.timestamp = time_struct;
                obj.last_seen = time_struct;
                obj.r = 1.0;
                obj.g = 1.0;
                obj.b = 1.0;
                obj.states.push_back(st);
                obj.tracking_sensors.push_back(t.sensor_id);
                obj.tracking_sensors_track_id.push_back(t.track_id);
                (*objects)[obj.object_id] = obj;
            }
            else
            {
                // Add to an existing object
                // RCLCPP_INFO(this->get_logger(), "Matched track to Object %d with state", close_object);
                //log_state((*objects)[close_object].states.back());
                //log_track(to_associate[i]);
                (*objects)[close_object].tracks_to_refine.push_back(to_associate[i]);
                // kalman_refine((*objects)[close_object].states.back(), to_associate[i]);
                // log_state((*objects)[close_object].states.back());
                if (to_associate[i].theta_valid)
                {
                    (*objects)[close_object].theta_valid = true;
                    (*objects)[close_object].states.back().theta_valid = true;
                }
                if ((*objects)[close_object].delay > 0)
                {
                    (*objects)[close_object].delay--;
                }

                // Dynamically changes Object type based on observations
                if(to_associate[i].object_type){
                    (*objects)[close_object].object_class_count += to_associate[i].confidence;
                }else{
                    (*objects)[close_object].object_class_count -= to_associate[i].confidence;
                }

                //  Prevents type flickering by requiring a large shift
                if((*objects)[close_object].object_class_count < -5){
                    (*objects)[close_object].object_type = false;
                }

                if((*objects)[close_object].object_class_count > 5){
                    (*objects)[close_object].object_type = true;
                }

                // Prevents overflow issues
                if((*objects)[close_object].object_class_count > 100){
                    (*objects)[close_object].object_class_count = 100;
                }

                if((*objects)[close_object].object_class_count < -100){
                    (*objects)[close_object].object_class_count = -100;
                }

                if(USE_SIM_TIME){                    
                    // Add calra_id
                    fusion::msg::Track t = to_associate[i];
                    bool found = false;
                    for(unsigned long int ind = 0; ind < (*objects)[close_object].carla_ids.size(); ind++){
                        if((*objects)[close_object].carla_ids[ind] == t.carla_id){
                            found = true;
                            break;
                        }
                    }
                    
                    if(!found){
                        (*objects)[close_object].carla_ids.push_back(t.carla_id);
                    }

                    found = false;
                    for(unsigned long int ind = 0; ind < (*objects)[close_object].states.back().carla_ids.size(); ind++){
                        if((*objects)[close_object].states.back().carla_ids[ind] == t.carla_id){
                            found = true;
                            break;
                        }
                    }
                    
                    if(!found){
                        (*objects)[close_object].states.back().carla_ids.push_back(t.carla_id);
                    }
                }
                (*objects)[close_object].last_seen = time_struct;
                (*objects)[close_object].tracking_sensors.push_back(to_associate[i].sensor_id);
                (*objects)[close_object].tracking_sensors_track_id.push_back(to_associate[i].track_id);
            }
        }

        // RCLCPP_INFO(this->get_logger(), "Refining Objects");
        auto refine = objects->begin();
        while (refine != objects->end())
        {
            if (refine->second.tracks_to_refine.size() > 0)
            {
                // RCLCPP_INFO(this->get_logger(), "Refining Object %d", refine->second.object_id);
                kalman_refine(refine->second.states.back(), refine->second.tracks_to_refine);

                refine->second.tracks_to_refine.clear();
            }
            refine++;
        }

        // Check for expiring objects and remove them
        std::vector<int> ids_to_erase;
        auto it2 = objects->begin();
        while (it2 != objects->end())
        {
            double delta_t2 = time_dif_double(time_struct, it2->second.last_seen);
            if (delta_t2 > 0.5)
            {
                ids_to_erase.push_back(it2->second.object_id);
            }
            it2++;
        }
        for (unsigned long int i = 0; i < ids_to_erase.size(); i++)
        {
            // RCLCPP_INFO(this->get_logger(), "Deleting Object %d", (*objects)[ids_to_erase[i]].object_id);
            objects->erase(ids_to_erase[i]);
        }
        ids_to_erase.clear();

        // If two objects come within a certain distance of each other we want to merge them

        std::vector<int> to_merge; // Stores the ids of objects to merge. Merge to_merge[i] with to_merge[i+1]
        auto first_obj = objects->begin();
        while (first_obj != objects->end())
        {
            auto second_obj = first_obj;
            second_obj++;
            while (second_obj != objects->end())
            {
                fusion::msg::RemoteObjectState s1 = first_obj->second.states.back();
                fusion::msg::RemoteObjectState s2 = second_obj->second.states.back();

                // Check to see if the two objects are similair enough
                if (abs(s1.lat - s2.lat) < MERGE_THRESHOLD && abs(s1.lon - s2.lon) < MERGE_THRESHOLD && first_obj->second.object_type == second_obj->second.object_type)
                {
                    bool share_sensor = false;
                    for (unsigned long int i = 0; i < first_obj->second.tracking_sensors.size(); i++)
                    {
                        for (unsigned long int j = 0; j < second_obj->second.tracking_sensors.size(); j++)
                        {
                            if (first_obj->second.tracking_sensors[i] == second_obj->second.tracking_sensors[j])
                            {
                                share_sensor = true;
                            }
                        }
                    }

                    if (!share_sensor)
                    {
                        // Check if valid thetas
                        // If valid thetas compare to make sure the objects are facing the same direction
                        bool valid_merge = true;
                        if (first_obj->second.theta_valid && second_obj->second.theta_valid)
                        {
                            if (abs(first_obj->second.states.back().theta - second_obj->second.states.back().theta) > M_PI / 4)
                            {
                                valid_merge = false;
                            }
                        }

                        // Add to merge list
                        if (valid_merge)
                        {
                            // RCLCPP_INFO(this->get_logger(), "Merging Obj: %d and Obj %d", first_obj->first, second_obj->first);
                            to_merge.push_back(first_obj->first);
                            to_merge.push_back(second_obj->first);
                        }
                    }
                }

                second_obj++;
            }
            first_obj++;
        }

        // Perform Merge operation
        for (unsigned long int i = 0; i < to_merge.size(); i += 2)
        {
            fusion::msg::TrackedObject res = merge_objects((*objects)[to_merge[i]], (*objects)[to_merge[i + 1]]);
            (*objects)[res.object_id] = res;
            objects->erase(to_merge[i]);
            objects->erase(to_merge[i + 1]);

            // Update the list with the new ids, in case it needs to be merged multiple times
            for (unsigned long int j = i + 2; j < to_merge.size(); j++)
            {
                if (to_merge[j] == to_merge[i] || to_merge[j] == to_merge[i + 1])
                {
                    to_merge[j] = res.object_id;
                }
            }
        }
        to_merge.clear();

        visualize();
        track_buf->clear();

        if(LOG_GT){
            //RCLCPP_INFO(this->get_logger(), "Time: %ld.%09d" ,time_struct.sec, time_struct.nanosec);
            //RCLCPP_INFO(this->get_logger(), "GroundTruth: Lat: %lf, Lon: %lf, Lat_Vel: %lf, Lon_Vel: %lf", GT_STATE.lat, GT_STATE.lon, GT_STATE.lat_vel, GT_STATE.lon_vel);
        }

        fusion::msg::TrackedObjectList m;
        auto to_pub = objects->begin();
        while (to_pub != objects->end())
        {
            fusion::msg::RemoteObjectState local_state = to_pub->second.states.back();
            fusion::msg::RemoteObjectState global_state;
            fusion::msg::RemoteObjectState sim_state = kalman_predict(SIM_EGO_STATE, local_state.timestamp);
            global_state.timestamp = local_state.timestamp;
            // There is a leading comma to help parse from the Prefix by roslog
            if(local_state.theta_valid){
                RCLCPP_INFO(this->get_logger(), ",%d.%09d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,True", time_struct.sec, time_struct.nanosec,local_state.object_id, local_state.object_type, local_state.lat, local_state.lon, local_state.theta, local_state.lat_vel, local_state.lon_vel, local_state.covariance[0], local_state.covariance[5], local_state.covariance[10], local_state.covariance[15], GT_STATE.lat, GT_STATE.lon, GT_STATE.lat_vel, GT_STATE.lon_vel, abs(local_state.lat - GT_STATE.lat), abs(local_state.lon - GT_STATE.lon), abs(local_state.lat_vel - GT_STATE.lat_vel), abs(local_state.lon_vel - GT_STATE.lon_vel), local_state.theta, abs(local_state.theta - GT_STATE.theta));
            }else{
                RCLCPP_INFO(this->get_logger(), ",%d.%09d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,N/A,N/A,False", time_struct.sec, time_struct.nanosec,local_state.object_id, local_state.object_type,local_state.lat, local_state.lon, local_state.theta, local_state.lat_vel, local_state.lon_vel, local_state.covariance[0], local_state.covariance[5], local_state.covariance[10], local_state.covariance[15], GT_STATE.lat, GT_STATE.lon, GT_STATE.lat_vel, GT_STATE.lon_vel, abs(local_state.lat - GT_STATE.lat), abs(local_state.lon - GT_STATE.lon), abs(local_state.lat_vel - GT_STATE.lat_vel), abs(local_state.lon_vel - GT_STATE.lon_vel));

            }
            double lat_offset_correction = local_state.lat * cos(-sim_state.theta) - local_state.lon * sin(-sim_state.theta);
            double lon_offset_correction = local_state.lat * sin(-sim_state.theta) + local_state.lon * cos(-sim_state.theta);
            global_state.lat = lat_offset_correction + sim_state.lat;
            global_state.lon = lon_offset_correction + sim_state.lon;
            global_state.theta = local_state.theta + sim_state.theta;
            double lat_vel_correction = local_state.lat_vel * cos(-sim_state.theta) - local_state.lon_vel * sin(-sim_state.theta);
            double lon_vel_correction = local_state.lat_vel * sin(-sim_state.theta) + local_state.lon_vel * cos(-sim_state.theta);
            global_state.lat_vel = lat_vel_correction + sim_state.lat_vel;
            global_state.lon_vel = lon_vel_correction + sim_state.lon_vel;
            global_state.theta_valid = local_state.theta_valid;
            global_state.object_id = local_state.object_id;
            global_state.covariance = local_state.covariance;
            global_state.carla_ids = local_state.carla_ids;
            global_state.object_type = to_pub->second.object_type;
            m.objs.push_back(local_state);
            to_pub++;
        }
        
        m.timestamp = time_struct;
        tracked_object_list_pub->publish(m);

        //RCLCPP_INFO(this->get_logger(), "------------------DONE---------------------");
    }

    
    
    fusion::msg::RemoteObjectState SIM_EGO_STATE;
    fusion::msg::RemoteObjectState GT_STATE;

    std::map<int, fusion::msg::TrackedObject> *objects;
    std::vector<fusion::msg::Track> *track_buf;
    rclcpp::Subscription<fusion::msg::Track>::SharedPtr track_subscription;
    rclcpp::TimerBase::SharedPtr fusion_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visual_object_pub;
    rclcpp::Publisher<fusion::msg::TrackedObjectList>::SharedPtr tracked_object_list_pub;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;
    rclcpp::Subscription<fusion::msg::RemoteObjectState>::SharedPtr sim_ego_state_sub;
    rclcpp::Subscription<fusion::msg::TrackedObjectList>::SharedPtr gt_state_sub;
    Eigen::MatrixXd Q_theta;

    KalmanFilter state_kalman;
    KalmanFilter theta_kalman;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Fusion>());
    rclcpp::shutdown();
}
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <map>
#include <chrono>

#include <boost/make_shared.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>

#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

using std_msgs::msg::String;
using json = nlohmann::json;

double wrapAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

struct ParsedObject
{
  std::string unique_id;
  std::string class_label;

  gtsam::Vector5 z;
  gtsam::Vector5 sigma;
};

struct ParsedObjFrame
{
  double stamp_sec;
  std::vector<ParsedObject> objects;
};





// ============================================================================
// Measurement factor
// State: [x, y, vx, vy, yaw]
// ============================================================================

class ObjectMeasurementFactor
  : public gtsam::NoiseModelFactor1<gtsam::Vector5>
{
public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Vector5>;

  ObjectMeasurementFactor(
    gtsam::Key key,
    const gtsam::Vector5 & z,
    const gtsam::SharedNoiseModel & noise)
  : Base(noise, key),
    z_(z)
  {
  }

  gtsam::Vector evaluateError(
    const gtsam::Vector5 & x,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    if (H) {
      *H = gtsam::Matrix::Identity(5, 5);
    }

    gtsam::Vector5 e = x - z_;
    e(4) = wrapAngle(x(4) - z_(4));
    return e;
  }

private:
  gtsam::Vector5 z_;
};

// ============================================================================
// Constant velocity motion factor
//
// x_j   = x_i + vx_i * dt
// y_j   = y_i + vy_i * dt
// vx_j  = vx_i
// vy_j  = vy_i
// yaw_j = yaw_i
// ============================================================================

class ConstantVelocityFactor
  : public gtsam::NoiseModelFactor2<gtsam::Vector5, gtsam::Vector5>
{
public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Vector5, gtsam::Vector5>;

  ConstantVelocityFactor(
    gtsam::Key key_i,
    gtsam::Key key_j,
    double dt,
    const gtsam::SharedNoiseModel & noise)
  : Base(noise, key_i, key_j),
    dt_(dt)
  {
  }

  gtsam::Vector evaluateError(
    const gtsam::Vector5 & xi,
    const gtsam::Vector5 & xj,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override
  {
    gtsam::Vector5 pred;
    pred << xi(0) + xi(2) * dt_,
            xi(1) + xi(3) * dt_,
            xi(2),
            xi(3),
            xi(4);

    gtsam::Vector5 e = xj - pred;
    e(4) = wrapAngle(xj(4) - xi(4));

    if (H1) {
      gtsam::Matrix A = gtsam::Matrix::Zero(5, 5);

      A(0, 0) = -1.0;
      A(0, 2) = -dt_;

      A(1, 1) = -1.0;
      A(1, 3) = -dt_;

      A(2, 2) = -1.0;
      A(3, 3) = -1.0;
      A(4, 4) = -1.0;

      *H1 = A;
    }

    if (H2) {
      *H2 = gtsam::Matrix::Identity(5, 5);
    }

    return e;
  }

private:
  double dt_;
};
// ============================================================================
// Per-object delayed-measurement-safe fixed-lag tracker
// ============================================================================

class ObjectTracker
{
public:
  ObjectTracker(
    const std::string & object_id,
    double fixed_lag_sec,
    double same_time_tolerance_sec,
    const gtsam::SharedNoiseModel & motion_noise)
  : object_id_(object_id),
    fixed_lag_sec_(fixed_lag_sec),
    same_time_tolerance_sec_(same_time_tolerance_sec),
    motion_noise_(motion_noise)
  {
  }

  bool addMeasurement(
    double measurement_stamp_sec,
    const gtsam::Vector5 & z,
    const gtsam::Vector5 & sigma,
    double receive_ros_time_sec,
    const std::string & class_label)
  {
    class_label_ = class_label;
    last_receive_ros_time_sec_ = receive_ros_time_sec;

    if (!has_any_measurement_) {
      max_measurement_stamp_sec_ = measurement_stamp_sec;
      has_any_measurement_ = true;
    } else {
      max_measurement_stamp_sec_ =
        std::max(max_measurement_stamp_sec_, measurement_stamp_sec);
    }

    const double window_start = max_measurement_stamp_sec_ - fixed_lag_sec_;

    if (measurement_stamp_sec < window_start) {
      // meas too old to be useful for the active fixed-lag window.
      return false;
    }

    const double slot_time = findOrCreateTimeSlot(measurement_stamp_sec);

    MeasurementRecord rec;
    rec.stamp_sec = measurement_stamp_sec;
    rec.z = z;
    rec.sigma = sigma;

    time_slots_[slot_time].measurements.push_back(rec);

    pruneOldSlots();
    return rebuildAndOptimize();
  }

  std::optional<gtsam::Vector5> predictTo(double output_time_sec) const
  {
    if (!has_estimate_) {
      return std::nullopt;
    }

    const double dt = output_time_sec - latest_estimate_stamp_sec_;

    if (dt < 0.0) {
      return latest_estimate_;
    }

    return predictState(latest_estimate_, dt);
  }

  double latestEstimateStampSec() const
  {
    return latest_estimate_stamp_sec_;
  }

  double lastReceiveRosTimeSec() const
  {
    return last_receive_ros_time_sec_;
  }

  bool hasEstimate() const
  {
    return has_estimate_;
  }

  std::string classLabel() const
  {
    return class_label_;
  }

  gtsam::Vector5 latestStddev() const
  {
    return latest_stddev_;
  }

private:
  struct MeasurementRecord
  {
    double stamp_sec;
    gtsam::Vector5 z;
    gtsam::Vector5 sigma;
  };

  struct TimeSlot
  {
    std::vector<MeasurementRecord> measurements;
  };

  double findOrCreateTimeSlot(double stamp_sec)
  {
    if (time_slots_.empty()) {
      time_slots_[stamp_sec] = TimeSlot{};
      return stamp_sec;
    }

    auto upper = time_slots_.lower_bound(stamp_sec);

    if (upper != time_slots_.end()) {
      if (std::abs(upper->first - stamp_sec) <= same_time_tolerance_sec_) {
        return upper->first;
      }
    }

    if (upper != time_slots_.begin()) {
      auto lower = std::prev(upper);
      if (std::abs(lower->first - stamp_sec) <= same_time_tolerance_sec_) {
        return lower->first;
      }
    }

    time_slots_[stamp_sec] = TimeSlot{};
    return stamp_sec;
  }

  void pruneOldSlots()
  {
    const double window_start = max_measurement_stamp_sec_ - fixed_lag_sec_;

    std::vector<double> times_to_remove;

    for (const auto & [slot_time, slot] : time_slots_) {
      if (slot_time < window_start) {
        times_to_remove.push_back(slot_time);
      }
    }

    for (const double t : times_to_remove) {
      time_slots_.erase(t);
    }
  }

  bool rebuildAndOptimize()
  {
    if (time_slots_.empty()) {
      has_estimate_ = false;
      return false;
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_values;

    std::map<double, gtsam::Key> time_to_key;

    size_t idx = 0;

    for (const auto & [slot_time, slot] : time_slots_) {
      const gtsam::Key key = gtsam::Symbol('x', idx++);
      time_to_key[slot_time] = key;

      const gtsam::Vector5 initial = initialGuessForSlot(slot_time, slot);
      initial_values.insert(key, initial);
    }

    // Add measurement factors.
    for (const auto & [slot_time, slot] : time_slots_) {
      const gtsam::Key key = time_to_key.at(slot_time);

      for (const auto & meas : slot.measurements) {
        const auto noise = gtsam::noiseModel::Diagonal::Sigmas(meas.sigma);

        graph.add(
          boost::make_shared<ObjectMeasurementFactor>(key, meas.z, noise));
      }
    }

    // Add motion factors between consecutive time slots.
    auto prev_it = time_to_key.begin();
    auto curr_it = prev_it;
    ++curr_it;

    for (; curr_it != time_to_key.end(); ++prev_it, ++curr_it) {
      const double t_prev = prev_it->first;
      const double t_curr = curr_it->first;
      const double dt = t_curr - t_prev;

      if (dt <= 0.0) {
        continue;
      }

      graph.add(
        boost::make_shared<ConstantVelocityFactor>(prev_it->second, curr_it->second, dt, motion_noise_));
    }

    try {
      gtsam::LevenbergMarquardtParams params;
      params.setMaxIterations(10);
      params.setRelativeErrorTol(1e-5);
      params.setAbsoluteErrorTol(1e-5);

      gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

      gtsam::Values result = optimizer.optimize();

      const auto latest_it = time_to_key.rbegin();
      const double latest_time = latest_it->first;
      const gtsam::Key latest_key = latest_it->second;

      if (!result.exists(latest_key)) {
        return false;
      }

      latest_estimate_ = result.at<gtsam::Vector5>(latest_key);
      try {
        gtsam::Marginals marginals(graph, result);
        gtsam::Matrix cov = marginals.marginalCovariance(latest_key);

        for (int i = 0; i < 5; ++i) {
            latest_stddev_(i) =
            std::sqrt(std::max(cov(i, i), 1e-8));
        }
      } catch (const std::exception &) {
        // keep prev/default stddev if marginal computation fails.
      }
      latest_estimate_(4) = wrapAngle(latest_estimate_(4));
      latest_estimate_stamp_sec_ = latest_time;
      has_estimate_ = true;

      // cache result for better future initial guesses.
      last_optimized_values_.clear();

      for (const auto & [t, key] : time_to_key) {
        if (result.exists(key)) {
          last_optimized_values_[t] = result.at<gtsam::Vector5>(key);
        }
      }

      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  gtsam::Vector5 initialGuessForSlot(double slot_time, const TimeSlot & slot) const
  {
    // if this slot was optimized before, reuse it.
    auto cached = last_optimized_values_.find(slot_time);

    if (cached != last_optimized_values_.end()) {
      return cached->second;
    }

    // else use average of measurements in the slot.
    if (!slot.measurements.empty()) {
      return averageMeasurements(slot.measurements);
    }

    gtsam::Vector5 zero;
    zero << 0.0, 0.0, 0.0, 0.0, 0.0;
    return zero;
  }

  gtsam::Vector5 averageMeasurements(const std::vector<MeasurementRecord> & measurements) const
  {
    gtsam::Vector5 avg;
    avg << 0.0, 0.0, 0.0, 0.0, 0.0;

    double sin_yaw = 0.0;
    double cos_yaw = 0.0;

    for (const auto & meas : measurements) {
      avg(0) += meas.z(0);
      avg(1) += meas.z(1);
      avg(2) += meas.z(2);
      avg(3) += meas.z(3);

      sin_yaw += std::sin(meas.z(4));
      cos_yaw += std::cos(meas.z(4));
    }

    const double n = static_cast<double>(measurements.size());

    avg(0) /= n;
    avg(1) /= n;
    avg(2) /= n;
    avg(3) /= n;
    avg(4) = std::atan2(sin_yaw / n, cos_yaw / n);

    return avg;
  }

  gtsam::Vector5 predictState(
    const gtsam::Vector5 & x,
    double dt) const
  {
    gtsam::Vector5 pred;
    pred << x(0) + x(2) * dt,
            x(1) + x(3) * dt,
            x(2),
            x(3),
            wrapAngle(x(4));

    return pred;
  }

private:
  std::string object_id_;

  double fixed_lag_sec_;
  double same_time_tolerance_sec_;
  gtsam::SharedNoiseModel motion_noise_;

  bool has_any_measurement_{false};
  bool has_estimate_{false};

  double max_measurement_stamp_sec_{0.0};
  double latest_estimate_stamp_sec_{0.0};
  double last_receive_ros_time_sec_{0.0};

  gtsam::Vector5 latest_estimate_;

  std::map<double, TimeSlot> time_slots_;
  std::map<double, gtsam::Vector5> last_optimized_values_;
  std::string class_label_{"unknown"};

  gtsam::Vector5 latest_stddev_ =
    (gtsam::Vector5() << 1.0, 1.0, 1.0, 1.0, 0.5).finished();
  };

// ============================================================================
// ROS2 fusion node
// ============================================================================

class ObjectFactorGraphFusionNode : public rclcpp::Node
{
public:
  ObjectFactorGraphFusionNode()
  : Node("object_factor_graph_fusion_node")
  {

    publish_rate_hz_ = 10.0;
    fixed_lag_sec_ = 2.0;
    same_time_tolerance_sec_ = 0.01;
    object_stale_timeout_sec_ = 2.0;
    max_extrapolation_sec_ = 0.35;

    heading_in_degrees_ =
      declare_parameter<bool>("heading_in_degrees", true);

    min_measurement_sigma_ =
      declare_parameter<double>("min_measurement_sigma", 0.001);

    // Motion model noise
    const double motion_xy_sigma = 0.8;
    const double motion_v_sigma = 1.5;
    const double motion_yaw_sigma = 0.3;

    motion_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector5() <<
        motion_xy_sigma,
        motion_xy_sigma,
        motion_v_sigma,
        motion_v_sigma,
        motion_yaw_sigma).finished());

    source1_sub_ = create_subscription<String>("matched_ego_actors", 100,
      std::bind(&ObjectFactorGraphFusionNode::source1Callback, this, std::placeholders::_1));

    source2_sub_ = create_subscription<String>("matched_infra_actors", 100,
      std::bind(&ObjectFactorGraphFusionNode::source2Callback, this, std::placeholders::_1));

    fused_pub_ = create_publisher<String>("fg_fused_topic", 100);

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_wall_timer(period_ns, std::bind(&ObjectFactorGraphFusionNode::publishTimerCallback, this));
    RCLCPP_INFO(get_logger(), "obj factor graph fusion node started. Rate %.2f Hz, lag %.2f s.", publish_rate_hz_, fixed_lag_sec_);
  }

private:
  ObjectTracker & getOrCreateTracker(const std::string & object_id)
  {
    auto it = trackers_.find(object_id);

    if (it == trackers_.end()) {
      auto tracker = std::make_unique<ObjectTracker>(
        object_id,
        fixed_lag_sec_,
        same_time_tolerance_sec_,
        motion_noise_);

      it = trackers_.emplace(object_id, std::move(tracker)).first;
    }

    return *(it->second);
  }

  ParsedObjFrame parseObjJsonFrame(const std::string & data, bool heading_in_degrees, double min_sigma)
  {
    const auto j = json::parse(data);
    RCLCPP_INFO(rclcpp::get_logger("parseObjJsonFrame"), "INPUT Parsing JSON frame with data: %s", data.c_str());

    ParsedObjFrame frame;
    const auto & ts = j.at("timestamp");
    if (ts.is_string()) {
      frame.stamp_sec = std::stod(ts.get<std::string>());
    } else {
      frame.stamp_sec = ts.get<double>();
    }

    const rclcpp::Time output_time = now();
    const double output_time_sec = output_time.seconds();

    json sensor_msg;
    sensor_msg["timestamp"] = std::to_string(output_time_sec);
    sensor_msg["visible_actors"] = json::array();

    for (const auto & obj : j.at("visible_actors")) {
      ParsedObject parsed;
      if (obj.at("uniqueId").is_string()) {
        parsed.unique_id = obj.at("uniqueId").get<std::string>();
      } else {
        parsed.unique_id = std::to_string(obj.at("uniqueId").get<int64_t>());
      }
      if (obj.at("class").is_string()) {
        parsed.class_label = obj.at("class").get<std::string>();
      } else {
        parsed.class_label = std::to_string(obj.at("class").get<int64_t>());
      }
      double yaw = obj.at("heading").at("theta").get<double>();
      double yaw_std = obj.at("heading").at("thetaStdDev").get<double>();

      if (heading_in_degrees) {
        yaw *= M_PI / 180.0;
        yaw_std *= M_PI / 180.0;
      }
      parsed.z << obj.at("pos").at("lat").get<double>(),
                  obj.at("pos").at("lon").get<double>(),
                  obj.at("vel").at("lat_vel").get<double>(),
                  obj.at("vel").at("lon_vel").get<double>(),
                  wrapAngle(yaw);
      parsed.sigma << std::max(obj.at("pos").at("posStdDev_lat").get<double>(), min_sigma),
                      std::max(obj.at("pos").at("posStdDev_lon").get<double>(), min_sigma),
                      std::max(obj.at("vel").at("velStdDev_lat").get<double>(), min_sigma),
                      std::max(obj.at("vel").at("velStdDev_lon").get<double>(), min_sigma),
                      std::max(yaw_std, min_sigma);

      frame.objects.push_back(parsed);

      json obj2;
      obj2["class"] = parsed.class_label;
      obj2["uniqueId"] = parsed.unique_id;

      obj2["estimated_position"] = {{"x", parsed.z(0)}, {"y", parsed.z(1)}
      // {"pos_x_stddev", tracker->latestStddev()(0)},
      // {"pos_y_stddev", tracker->latestStddev()(1)}
      };

      obj2["estimated_velocity"] = {{"v_x", parsed.z(2)}, {"v_y", parsed.z(3)}
      // {"vel_x_stddev", tracker->latestStddev()(2)},
      // {"vel_y_stddev", tracker->latestStddev()(3)}
      };

      obj2["estimated_heading"] = wrapAngle(yaw);

      // obj["headingStdDev"] = heading_in_degrees_ ?
      // tracker->latestStddev()(4) * 180.0 / M_PI :
      // tracker->latestStddev()(4);

      sensor_msg["visible_actors"].push_back(obj2);
    }

    String out;
    out.data = sensor_msg.dump();
    RCLCPP_INFO(get_logger(), "OUTPUT DATA: %s", out.data.c_str());
    fused_pub_->publish(out);
    
    return frame;
  }

  void source1Callback(const String::SharedPtr msg)
  {
    // One common measurement timestamp for all ego/source-1 objects.
    const double receive_ros_time_sec = now().seconds();
    ParsedObjFrame frame;
    try {
        frame = parseObjJsonFrame(msg->data, heading_in_degrees_, min_measurement_sigma_);
    } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to parse source1 JSON: %s", e.what());
        return;
    }

    for (const auto & obj : frame.objects) {
        auto & tracker = getOrCreateTracker(obj.unique_id);
        tracker.addMeasurement(frame.stamp_sec, obj.z, obj.sigma, receive_ros_time_sec, obj.class_label);
    }
    cleanupStaleObjects(receive_ros_time_sec);
  }

  void source2Callback(const String::SharedPtr msg)
  {
    const double receive_ros_time_sec = now().seconds();
    ParsedObjFrame frame;
    try {
        frame = parseObjJsonFrame(msg->data, heading_in_degrees_, min_measurement_sigma_);
    } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to parse source2 JSON: %s", e.what());
        return;
    }

    for (const auto & obj : frame.objects) {
        auto & tracker = getOrCreateTracker(obj.unique_id);

        tracker.addMeasurement(frame.stamp_sec, obj.z, obj.sigma, receive_ros_time_sec, obj.class_label);
    }

    cleanupStaleObjects(receive_ros_time_sec);
  }

  void publishTimerCallback()
  {
    const rclcpp::Time output_time = now();
    const double output_time_sec = output_time.seconds();

    cleanupStaleObjects(output_time_sec);

    json sensor_msg;
    sensor_msg["timestamp"] = std::to_string(output_time_sec);
    sensor_msg["visible_actors"] = json::array();

    for (const auto & [object_id, tracker] : trackers_) {
      RCLCPP_INFO(this->get_logger(), "obj_id=%s", object_id.c_str());
      if (!tracker->hasEstimate()) {
      continue;
      }

      const double extrapolation_time = output_time_sec - tracker->latestEstimateStampSec();
      RCLCPP_INFO(this->get_logger(), "extrap_time=%f, output_time==%f, tracker_sec=%f", extrapolation_time, output_time_sec, tracker->latestEstimateStampSec());

      if (extrapolation_time < 0.0 || extrapolation_time > max_extrapolation_sec_) {
        continue;
      }

      const auto state = tracker->predictTo(output_time_sec);

      if (!state.has_value()) {
        continue;
      }

      const gtsam::Vector5 x = state.value();

      json obj;
      obj["class"] = tracker->classLabel();
      obj["uniqueId"] = object_id;

      obj["estimated_position"] = {{"x", x(0)}, {"y", x(1)}
      // {"pos_x_stddev", tracker->latestStddev()(0)},
      // {"pos_y_stddev", tracker->latestStddev()(1)}
      };

      obj["estimated_velocity"] = {{"v_x", x(2)}, {"v_y", x(3)}
      // {"vel_x_stddev", tracker->latestStddev()(2)},
      // {"vel_y_stddev", tracker->latestStddev()(3)}
      };

      obj["estimated_heading"] = wrapAngle(x(4));

      // obj["headingStdDev"] = heading_in_degrees_ ?
      // tracker->latestStddev()(4) * 180.0 / M_PI :
      // tracker->latestStddev()(4);

      sensor_msg["visible_actors"].push_back(obj);
    }

    String out;
    out.data = sensor_msg.dump();
    //RCLCPP_INFO(get_logger(), "OUTPUT DATA: %s", out.data.c_str());
    //fused_pub_->publish(out);
  }

  void cleanupStaleObjects(double current_ros_time_sec)
  {
    std::vector<std::string> ids_to_delete;

    for (const auto & [object_id, tracker] : trackers_) {
      const double age =
        current_ros_time_sec - tracker->lastReceiveRosTimeSec();

      if (age > object_stale_timeout_sec_) {
        ids_to_delete.push_back(object_id);
      }
    }

    for (const std::string & id : ids_to_delete) {
      trackers_.erase(id);
    }
  }

  rclcpp::Publisher<String>::SharedPtr fused_pub_;
private:
  std::string latest_input_frame_id_;
  std::string sensing_mode {"sen_infra"};

  double publish_rate_hz_{10.0};
  double fixed_lag_sec_{2.0};
  double same_time_tolerance_sec_{0.01};
  double object_stale_timeout_sec_{4.0};
  double max_extrapolation_sec_{0.35};
  bool heading_in_degrees_{false};
  double min_measurement_sigma_{0.001};

  rclcpp::Subscription<String>::SharedPtr source1_sub_;
  rclcpp::Subscription<String>::SharedPtr source2_sub_;
  
  rclcpp::TimerBase::SharedPtr publish_timer_;

  gtsam::SharedNoiseModel motion_noise_;

  std::unordered_map<std::string, std::unique_ptr<ObjectTracker>> trackers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectFactorGraphFusionNode>());
  rclcpp::shutdown();
  return 0;
}
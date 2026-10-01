import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from fusion.msg import TrackedObjectList  # Import your custom message type
from fusion.msg import GPS
from fusion.msg import RemoteObjectState
from builtin_interfaces.msg import Time
import numpy as np
import math
from collections import deque
import time

class EgoMsgToJson(Node):
    def __init__(self):
        super().__init__('real_infra_source_from_gt')

        # Subscribe to the custom message topic
        self.remote_subscriber = self.create_subscription(GPS, 'GT', self.real_gt_remote_callback, 10)
        self.ego_subscriber = self.create_subscription(String, "ego_states", self.ego_callback, 10)
        self.inter_basic_sub = self.create_subscription(String, "closest_intersection_basic", self.inter_basic_callback, 10)

        # Publisher for JSON messages
        self.publisher = self.create_publisher(String, 'infra_local_measures', 10)
        self.publish_json_msg = None

        self.mode_arg = self.declare_parameter('mode_arg', 'sim').value
        self.prev_timestamp_trial = None
        self.ref_pos = None

        self.ego_theta = 0
        
        self.prev_theta = None
        self.prev_log_theta = None
        self.remote_theta_queue = deque()

        self.json_data = {
            "timestamp": None,
            "refPos": self.ref_pos,
            "sdsmData": []
        }

        self.std_dev_position_infra = 0.8
        self.std_dev_velocity_infra = 0.04
        self.std_dev_heading_infra = 0.08  # in radians

        self.last_received_time = time.time()
        self.timer = self.create_timer(3.0, self.check_timeout)

    def check_timeout(self):
        if time.time() - self.last_received_time > 3:
            self.prev_theta = None
            self.prev_log_theta = None
            self.remote_theta_queue = deque()

    def inter_basic_callback(self, msg):
        data = json.loads(msg.data)
        self.ref_pos = data['pos']

    def ego_callback(self, msg):
        if self.mode_arg == "sim" or self.ref_pos is None:
            return
        ego_states = json.loads(msg.data)
        self.json_data = {
            "timestamp": None,
            "refPos": self.ref_pos,
            "sdsmData": []
        }
        transform = ego_states['ego_vehicle']['pos']
        velocity = ego_states['ego_vehicle']['vel']
        heading = ego_states['ego_vehicle']['heading']['theta'] + 90

        # Compute offset relative to refPos
        offset_x = -transform['lon']- self.ref_pos["x"]
        offset_y = -transform['lat'] - self.ref_pos["y"]
        noisy_offset_x = self.add_gaussian_noise(offset_x, self.std_dev_position_infra)
        noisy_offset_y = self.add_gaussian_noise(offset_y, self.std_dev_position_infra)
        speed = np.linalg.norm([velocity['lon_vel'], velocity['lat_vel']])
        noisy_speed = np.abs(self.add_gaussian_noise(speed, self.std_dev_velocity_infra))
        noisy_heading = self.add_gaussian_noise(heading, self.std_dev_heading_infra)

        self.last_received_time = time.time()

        actor_data = {
            "class": "vehicle",
            "uniqueId": ego_states['ego_vehicle']['uniqueId']+50,
            "pos": {
                "x": offset_x,
                "y": offset_y
            },
            "posStdDev": self.std_dev_position_infra,
            "speed": speed,
            "speedStdDev": self.std_dev_velocity_infra,
            "heading": heading,
            "headingStdDev": self.std_dev_heading_infra,
            "halfDim": None
        }
        self.json_data["sdsmData"].append(actor_data)
        json_str = json.dumps(self.publish_json_msg)
        self.ego_theta = heading

        if not ego_states["is_visible_actors_present"]:
            json_msg_pub = String()
            json_msg_pub.data = json_str
            self.publisher.publish(json_msg_pub)
            self.publish_json_msg = None
            self.get_logger().info(f"Ego JSON: {msg.data}")

    def real_gt_remote_callback(self, msg):
        if self.mode_arg == "sim" or self.ref_pos is None:
            return
        timestamp = None

        # json_msg = {
        #     "timestamp": {"sec": msg.timestamp.sec, "nanosec": msg.timestamp.nanosec},
        #     "visible_actors": []
        # }

        # Convert each object state to JSON format
        for obj in msg.objs:
            ego_fusion_est = [float(obj.lat), float(obj.lon), float(obj.theta), float(obj.theta_cov), float(obj.lat_vel), float(obj.lon_vel), abs(float(obj.covariance[0])), abs(float(obj.covariance[5])), abs(float(obj.covariance[10])), abs(float(obj.covariance[15]))]
            if np.any(np.isnan(ego_fusion_est)):
                self.get_logger().info("********************received NaN**********************")
                return
            carla_id = -2
            if (len(obj.carla_ids)) > 0:
                carla_id = int(obj.carla_ids[-1])
            is_theta_valid = bool(obj.theta_valid)
            lat = -ego_fusion_est[0]
            lon = ego_fusion_est[1]
            theta = -ego_fusion_est[2]
            theta_cov = ego_fusion_est[3]
            lat_vel = -ego_fusion_est[4]
            lon_vel = ego_fusion_est[5]
            lat_cov = ego_fusion_est[6]
            lon_cov = ego_fusion_est[7]
            vel_lat_cov = ego_fusion_est[8]
            vel_lon_cov = ego_fusion_est[9]

            # Compute offset relative to refPos
            offset_x = -ego_fusion_est[1]- self.ref_pos["x"]
            offset_y = -ego_fusion_est[0] - self.ref_pos["y"]
            ego_theta = (self.ego_theta-90)*np.pi/180
            ref_pos_x = self.ref_pos["x"]
            self.get_logger().info(f"some parameters: {ref_pos_x}, {lat * np.cos(ego_theta)}, {lon * np.sin(ego_theta)}")
            offset_x = -self.ref_pos["x"] + lat * np.cos(ego_theta) - lon * np.sin(ego_theta)
            offset_y = -self.ref_pos["y"] + lat * np.sin(ego_theta) + lon * np.cos(ego_theta)
            noisy_offset_x = self.add_gaussian_noise(offset_x, self.std_dev_position_infra)
            noisy_offset_y = self.add_gaussian_noise(offset_y, self.std_dev_position_infra)
            speed = np.linalg.norm([lon_vel, lat_vel])
            noisy_speed = np.abs(self.add_gaussian_noise(speed, self.std_dev_velocity_infra))
            noisy_heading = self.add_gaussian_noise(theta, self.std_dev_heading_infra)

            self.get_logger().info(f"Remote GT wrt Ego: {lat}, {lon}")

            theta = theta*180/np.pi - self.ego_theta
            # if self.prev_theta is None:
            #     self.prev_theta = theta
            # elif abs(self.prev_theta-theta) > 100:
            #     # if abs(self.prev_log_theta-theta) < 10:
            #     #     self.prev_theta = theta
            #     # else:
            #     theta = None
            # else:
            #     self.prev_theta = theta
            # if theta is not None:
            #     self.prev_log_theta = theta
            #     theta = (theta + 180)%360 - 180
            #     self.remote_theta_queue.append(theta)
            #     if len(self.remote_theta_queue) > 3:
            #         self.remote_theta_queue.popleft()
            #     theta = sum(self.remote_theta_queue)/len(self.remote_theta_queue)
            # else:
            #     theta = self.prev_theta

            actor_data = {
                "class": "vehicle",
                "uniqueId": int(obj.object_id),
                "pos": {
                    "x": offset_x,
                    "y": offset_y
                },
                "posStdDev": self.std_dev_position_infra,
                "speed": speed,
                "speedStdDev": self.std_dev_velocity_infra,
                "heading": theta,
                "headingStdDev": self.std_dev_heading_infra*180/np.pi,
                "halfDim": None
            }
            timestamp = obj.timestamp.sec + obj.timestamp.nanosec/1e9
            self.prev_timestamp_trial = timestamp
            if obj.object_type:
                actor_data["class"] = "vru"
            self.json_data["sdsmData"].append(actor_data)
            
        if timestamp is not None:
            self.json_data["timestamp"] = timestamp
            json_str = json.dumps(self.json_data)
            json_msg_pub = String()
            json_msg_pub.data = json_str
            self.publisher.publish(json_msg_pub)
            self.publish_json_msg = None

            self.get_logger().info(f"Published JSON from GT-Infra: {json_str}")

    def add_gaussian_noise(self, value, std_dev):
        return value + np.random.normal(0, std_dev)

def main(args=None):
    rclpy.init(args=args)
    node = EgoMsgToJson()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

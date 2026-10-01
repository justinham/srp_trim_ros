import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from fusion.msg import TrackedObjectList  # Import your custom message type
from fusion.msg import RemoteObjectState
from fusion.msg import GPS
from builtin_interfaces.msg import Time
import numpy as np
import math
from collections import deque
import time

class GlobalGTPublisherNode(Node):
    def __init__(self):
        super().__init__('global_gt_publisher_node')

        # Subscribe to the custom message topic
        self.remote_subscriber = self.create_subscription(GPS, 'GT', self.real_gt_remote_callback, 10)
        self.ego_states_sub = self.create_subscription(String, 'ego_states', self.ego_callback, 10)
        self.gt_publisher = self.create_publisher(String, 'ground_truth_topic', 10)

        self.mode_arg = self.declare_parameter('mode_arg', 'sim').value

        self.gt_json_data = {
            "timestamp": None,
            "ego_vehicle": None,
            "remote_actors": None
        }

        self.timer = self.create_timer(0.1, self.publish_gt)

    def publish_gt(self):
        if self.gt_json_data["ego_vehicle"] is not None and (time.time() - self.gt_json_data["timestamp"] < 1):
            json_gt_msg = String()
            json_gt_msg.data = json.dumps(self.gt_json_data)
            self.gt_publisher.publish(json_gt_msg)
            self.get_logger().info(f"Published ground truth data: {json_gt_msg.data}")

    def ego_callback(self, msg):
        if self.mode_arg == "sim":
            return
        ego_states = json.loads(msg.data)
        ego_states = ego_states["ego_vehicle"]
        ego_states["uniqueId"] = str(ego_states["uniqueId"])+"_gt"
        self.gt_json_data["ego_vehicle"] = ego_states
        self.gt_json_data["timestamp"] = time.time()

    def real_gt_remote_callback(self, msg):
        if self.mode_arg == "sim":
            return
        
        if self.gt_json_data["ego_vehicle"] is None:
            return

        for obj in msg.objs:
            ego_fusion_est = [float(obj.lat), float(obj.lon), float(obj.theta), float(obj.theta_cov), float(obj.lat_vel), float(obj.lon_vel), abs(float(obj.covariance[0])), abs(float(obj.covariance[5])), abs(float(obj.covariance[10])), abs(float(obj.covariance[15]))]
            if np.any(np.isnan(ego_fusion_est)):
                self.get_logger().info("********************received NaN**********************")
                return
            lat = -ego_fusion_est[0]
            lon = ego_fusion_est[1]
            theta = -ego_fusion_est[2]
            lat_vel = -ego_fusion_est[4]
            lon_vel = ego_fusion_est[5]

            ego_global_pos = self.gt_json_data["ego_vehicle"]["pos"]
            ego_global_x, ego_global_y = ego_global_pos["lat"], ego_global_pos["lon"]
            ego_theta = self.gt_json_data["ego_vehicle"]["heading"]["theta"]

            ego_theta = (ego_theta)*np.pi/180
            # self.get_logger().info(f"some parameters: {lat * np.cos(ego_theta)}, {lon * np.sin(ego_theta)}")
            remote_global_x = ego_global_x + lat * np.cos(ego_theta) - lon * np.sin(ego_theta)
            remote_global_y = ego_global_y + lat * np.sin(ego_theta) + lon * np.cos(ego_theta)
            speed = np.linalg.norm([lon_vel, lat_vel])

            self.get_logger().info(f"Remote GT wrt Ego: {lat}, {lon}")

            theta = theta*180/np.pi + ego_theta*180/np.pi

            actor_data = {
                "uniqueId": str(obj.object_id)+"_gt",
                "carlaId": None,
                "class": "vehicle",
                "pos": {
                    "lat": remote_global_x,
                    "lon": remote_global_y,
                    "posStdDev_lat": "0.0",
                    "posStdDev_lon": "0.0"
                },
                "vel": {
                    "lat_vel": "0.0",
                    "lon_vel": "0.0",
                    "velStdDev_lat": "0.0",
                    "velStdDev_lon": "0.0"
                },
                "heading": {
                    "theta": theta,
                    "theta_valid": True,
                    "thetaStdDev": "0.0"
                },
                "halfDim": None
            }
            if obj.object_type:
                actor_data["class"] = "vru"
            self.gt_json_data["remote_actors"] = []
            self.gt_json_data["remote_actors"].append(actor_data)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalGTPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

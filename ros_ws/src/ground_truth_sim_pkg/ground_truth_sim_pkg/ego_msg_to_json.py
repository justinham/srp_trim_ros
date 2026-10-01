import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from fusion.msg import TrackedObjectList  # Import your custom message type
from fusion.msg import RemoteObjectState
from builtin_interfaces.msg import Time
import numpy as np
import math
from collections import defaultdict
import time

class EgoMsgToJson(Node):
    def __init__(self):
        super().__init__('ego_msg_to_json')

        # Subscribe to the custom message topic
        self.remote_subscriber = self.create_subscription(TrackedObjectList, 'tracked_object_list', self.fusion_callback, 10)

        # Publisher for JSON messages
        self.publisher = self.create_publisher(String, 'ego_local_measures', 10)

        self.mode_arg = self.declare_parameter('mode_arg', 'real').value
        self.prev_timestamp_trial = None
        self.theta_dict = defaultdict(list)

        self.prev_theta_buffer = {}
        self.obj_history_tracker = {}
        self.last_received_time = time.time()
        # self.timer = self.create_timer(1.0, self.check_timeout)

    # def check_timeout(self):
    #     if time.time() - self.last_received_time > 1:
    #         self.theta_dict = defaultdict(list)
    #         self.publish_json_msg["visible_actors"] = []

    def fusion_callback(self, msg):
        self.last_received_time = time.time()
        self.publish_json_msg = {
            "timestamp": msg.objs[0].timestamp.sec + msg.objs[0].timestamp.nanosec/1e9,
            "visible_actors": [],
        }
        for obj_id in self.obj_history_tracker:
            self.obj_history_tracker[obj_id] += 1
        for obj in msg.objs:
            ego_fusion_est = [float(obj.lat), float(obj.lon), float(obj.theta), float(obj.theta_cov), float(obj.lat_vel), float(obj.lon_vel), abs(float(obj.covariance[0])), abs(float(obj.covariance[5])), abs(float(obj.covariance[10])), abs(float(obj.covariance[15]))]
            if np.any(np.isnan(ego_fusion_est)):
                self.get_logger().info(f"NaN reported timestamp: {obj.timestamp}")
                self.get_logger().info("********************received NaN**********************")
                continue
            self.obj_history_tracker[str(obj.object_id)] = 0
            is_theta_valid = bool(obj.theta_valid)
            if self.mode_arg == 'real':
                lat = -ego_fusion_est[0]
                lon = ego_fusion_est[1]
                theta = ego_fusion_est[2]
                # theta = (theta+np.pi)%(2*np.pi)-np.pi
                theta_cov = ego_fusion_est[3]
                lat_vel = -ego_fusion_est[4]
                lon_vel = ego_fusion_est[5]
                if not is_theta_valid:
                    if abs(lat_vel) > 1.0 or abs(lon_vel) > 1.0:
                        theta = math.atan2(lon_vel, lat_vel) - np.pi/2
                        theta = (theta+np.pi)%(2*np.pi)-np.pi
                        self.prev_theta_buffer[str(obj.object_id)] = theta
                    else:
                        if str(obj.object_id) in self.prev_theta_buffer:
                            theta = self.prev_theta_buffer[str(obj.object_id)]
                        else:
                            theta = 0
                else:
                    theta = ego_fusion_est[2]
                    self.prev_theta_buffer[str(obj.object_id)] = theta
                # theta = math.atan2(lon_vel, lat_vel) - np.pi/2
                # theta = (theta+np.pi)%(2*np.pi)-np.pi
                if int(obj.object_id) in self.theta_dict:
                    if abs(sum(self.theta_dict[int(obj.object_id)])/len(self.theta_dict[int(obj.object_id)]) - theta) < 100:
                        self.theta_dict[int(obj.object_id)].append(theta)
                else:
                    self.theta_dict[int(obj.object_id)].append(theta)
                if len(self.theta_dict[int(obj.object_id)]) > 3:
                    del self.theta_dict[int(obj.object_id)][0]
                theta = sum(self.theta_dict[int(obj.object_id)])/len(self.theta_dict[int(obj.object_id)])
                lat_cov = ego_fusion_est[6]
                lon_cov = ego_fusion_est[7]
                vel_lat_cov = ego_fusion_est[8]
                vel_lon_cov = ego_fusion_est[9]
            else:
                lat = ego_fusion_est[1]
                lon = ego_fusion_est[0]
                theta = ego_fusion_est[2]
                theta_cov = ego_fusion_est[3]
                lat_vel = ego_fusion_est[5]
                lon_vel = ego_fusion_est[4]
                if not is_theta_valid:
                    if abs(lat_vel) > 1.0 or abs(lon_vel) > 1.0:
                        theta = math.atan2(lon_vel, lat_vel)
                        theta = (theta+np.pi)%(2*np.pi)-np.pi
                    else:
                        theta = 0
                lat_cov = ego_fusion_est[7]
                lon_cov = ego_fusion_est[6]
                vel_lat_cov = ego_fusion_est[9]
                vel_lon_cov = ego_fusion_est[8]
            obj_data = {
                "uniqueId": int(obj.object_id),
                "class": "vehicle",
                "pos": {
                    "lat": lat,
                    "lon": lon,
                    "posStdDev_lat": math.sqrt(lat_cov),
                    "posStdDev_lon": math.sqrt(lon_cov)
                },
                "vel": {
                    "lat_vel": lat_vel,
                    "lon_vel": lon_vel,
                    "velStdDev_lat": math.sqrt(vel_lat_cov),
                    "velStdDev_lon": math.sqrt(vel_lon_cov)
                },
                "heading": {
                    "theta": theta*180/np.pi,
                    "thetaStdDev": math.sqrt(theta_cov*180/np.pi),
                    "is_theta_valid": is_theta_valid
                }
            }
            self.get_logger().info(f"{obj.object_type}")
            if obj.object_type:
                obj_data["class"] = "vru"
            self.publish_json_msg["visible_actors"].append(obj_data)

        for obj_id in list(self.obj_history_tracker.keys()):
            if self.obj_history_tracker[obj_id] > 10:
                del self.obj_history_tracker[obj_id]
                self.prev_theta_buffer.pop(obj_id, None)
                self.theta_dict.pop(int(obj_id), None)

        json_str = json.dumps(self.publish_json_msg)
        json_msg_pub = String()
        json_msg_pub.data = json_str
        self.publisher.publish(json_msg_pub)

        self.get_logger().info(f"Published JSON: {json_str}")

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

import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import gtsam
from gtsam import noiseModel
import numpy as np
from collections import deque
import math

class EstimateEgoNode(Node):
    def __init__(self):
        super().__init__('estimate_ego_node')

        self.factor_graphs = {}

        self.fused_estimates = {
            'timestamp': 0,

            'ego_vehicle': [],

            'visible_actors': []
        }

        self.ego_sub = self.create_subscription(String, 'ego_global_measures', self.ego_callback, 10)


        self.ego_id = None

        self.pub = self.create_publisher(String, 'ego_actor_states', 10)

    def ego_callback(self, msg):

        data = json.loads(msg.data)

        ego_vehicle = data["ego_vehicle"]
        timestamp = data.get("timestamp", self.get_clock().now().nanoseconds/1e9)
        self.fused_estimates["timestamp"] = timestamp

        if ego_vehicle != None:
            visible_actors = data["visible_actors"]
            self.ego_id = ego_vehicle["uniqueId"]

            # self.get_logger().info(f"Received in estimate_infra {data}")

            self.fused_estimates["ego_vehicle"].append({
                "uniqueId": str(ego_vehicle["uniqueId"]),
                "class": ego_vehicle["class"],
                "estimated_position": {"x": ego_vehicle["pos"]["lat"], "y": ego_vehicle["pos"]["lon"]},
                "estimated_velocity": {"v_x": ego_vehicle["vel"]["lat_vel"], "v_y": ego_vehicle["vel"]["lon_vel"]},
                "estimated_heading": np.radians(ego_vehicle["heading"]["theta"])
            })

            for actor_global in visible_actors:
                self.fused_estimates["visible_actors"].append({
                    "uniqueId": str(actor_global["uniqueId"]),
                    "class": actor_global["class"],
                    "estimated_position": {"x": actor_global["pos"]["lat"], "y": actor_global["pos"]["lon"]},
                    "estimated_velocity": {"v_x": actor_global["vel"]["lat_vel"], "v_y": actor_global["vel"]["lon_vel"]},
                    "estimated_heading": np.radians(actor_global["heading"]["theta"])
                    # "estimated_heading": 0.0
                })

        self.publish_fused_estimates()

    def publish_fused_estimates(self):
        if not self.fused_estimates:
            return

        fused_msg = String()
        fused_msg.data = json.dumps(self.fused_estimates)

        self.pub.publish(fused_msg)

        # self.get_logger().info(f"Published1 ego state for all actors: {fused_msg.data}")


        self.fused_estimates = {
            'timestamp': 0,
            'refPos': None,

            'ego_vehicle': [],
            'visible_actors': []
        }

def main(args=None):
    rclpy.init(args=args)
    node = EstimateEgoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
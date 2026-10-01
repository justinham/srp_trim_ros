import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import gtsam
from gtsam import noiseModel
import numpy as np
from collections import deque
import math

class EstimateInfraNode(Node):
    def __init__(self):
        super().__init__('estimate_infra_node')

        self.factor_graphs = {}

        self.fused_estimates = {
            'timestamp': 0,
            'ego_vehicle': [],
            'visible_actors': []
        }

        self.infra_sub = self.create_subscription(String, 'infra_global_measures', self.infra_callback, 10)

        self.ego_id = None

        self.ego_data_received = False
        self.infra_data_received = False

        self.pub = self.create_publisher(String, 'infra_actor_states', 10)

    def infra_callback(self, msg):

        data = json.loads(msg.data)
        ego_vehicle = data["ego_vehicle"]
        timestamp = data["timestamp"]
        self.fused_estimates['timestamp'] = timestamp
        self.fused_estimates['refPos'] = data['refPos']


        visible_actors = data["visible_actors"]
        # if ego_vehicle is not None or len(ego_vehicle) != 0:
        #     self.fused_estimates["ego_vehicle"].append({
        #         "uniqueId": str(ego_vehicle["uniqueId"]),
        #         "class": ego_vehicle["class"],
        #         "estimated_position": {"x": ego_vehicle["pos"]["lat"], "y": ego_vehicle["pos"]["lon"]},
        #         "estimated_velocity": {"v_x": ego_vehicle["vel"]["lat_vel"], "v_y": ego_vehicle["vel"]["lon_vel"]},
        #         "estimated_heading": np.radians(ego_vehicle["heading"]["theta"])
        #     })
        
        for actor_global in visible_actors:
            self.fused_estimates["visible_actors"].append({
                "uniqueId": str(actor_global["uniqueId"]),
                "class": actor_global["class"],
                "estimated_position": {"x": actor_global["pos"]["lat"], "y": actor_global["pos"]["lon"]},
                "estimated_velocity": {"v_x": actor_global["vel"]["lat_vel"], "v_y": actor_global["vel"]["lon_vel"]},
                "estimated_heading": np.radians(actor_global["heading"]["theta"])
            })

        self.publish_fused_estimates()

    def publish_fused_estimates(self):
        if not self.fused_estimates:
            return

        fused_msg = String()
        fused_msg.data = json.dumps(self.fused_estimates)

        self.pub.publish(fused_msg)
        self.get_logger().info(f"Published ego state for all actors: {fused_msg.data}")

        self.fused_estimates = {
            'timestamp': 0,
            'refPos': None,
            'ego_vehicle': [],
            'visible_actors': []
        }

def main(args=None):
    rclpy.init(args=args)
    node = EstimateInfraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
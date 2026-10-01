import rclpy
from rclpy.node import Node
import json
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import distance
from std_msgs.msg import String
from collections import defaultdict, deque

class ObjectMatcherNode(Node):
    def __init__(self):
        super().__init__('object_matcher_node')

        self.ego_sub = self.create_subscription(String, 'ego_global_measures', self.ego_callback, 10)
        self.infra_sub = self.create_subscription(String, 'infra_global_measures', self.infra_callback, 10)
        self.ego_gps_subscriber = self.create_subscription(String, "ego_states", self.ego_gps_callback, 10)
        self.ego_vehicle = None

        self.ego_pub = self.create_publisher(String, 'matched_ego_actors', 10)
        self.infra_pub = self.create_publisher(String, 'matched_infra_actors', 10)

        self.ego_message = None
        self.infra_message = None

        self.matched_ids = {}  # (infra_id -> assigned_id)
        self.object_lifetime = defaultdict(int)  # track how long an object is seen

        self.max_lost_time = 1  # timesteps
        self.selected_topic = "fused_actor_states"

    def ego_gps_callback(self, msg):
        data = json.loads(msg.data)
        self.ego_vehicle = data["ego_vehicle"]

    def ego_callback(self, msg):
        self.ego_message = json.loads(msg.data)
        self.get_logger().info("calling from ego")
        self.process_matching()

    def infra_callback(self, msg):
        self.infra_message = json.loads(msg.data)
        self.get_logger().info("calling from infra")
        self.process_matching()

    def process_matching(self):
        ego_actors = self.ego_message["visible_actors"] if self.ego_message else []
        infra_actors = self.infra_message["visible_actors"] if self.infra_message else []

        matched_pairs = self.multi_feature_hungarian_matching(ego_actors, infra_actors)

        assigned_ids = {}
        for ego_id, infra_id in matched_pairs.items():
            assigned_ids[infra_id] = ego_id

        # handle unmatched infra objects
        for infra_obj in infra_actors:
            infra_id = infra_obj["uniqueId"]
            if infra_id not in assigned_ids:
                # keep previous assigned id if exists
                if infra_id in self.matched_ids:
                    assigned_ids[infra_id] = self.matched_ids[infra_id]
                else:
                    # assign a new unique id for this object
                    assigned_ids[infra_id] = f"{infra_id}"

        # update id tracking
        self.matched_ids = assigned_ids

        # prepare messages
        ego_output = self.format_output(self.ego_message, ego_actors, assigned_ids, "ego")
        infra_output = self.format_output(self.infra_message, infra_actors, assigned_ids, "infra")

        # publish updated messages
        if ego_output:
            self.get_logger().info(f"Ego match: {ego_output.data}")
            self.ego_pub.publish(ego_output)
        if infra_output:
            self.get_logger().info(f"Infra match: {infra_output.data}")
            self.infra_pub.publish(infra_output)

        # cleanup old objects
        self.cleanup_tracking()

    def multi_feature_hungarian_matching(self, ego_vehicles, infra_vehicles, position_weight=0.3, velocity_weight=2.0, threshold=5.0):
        matched_pairs = {}

        if not ego_vehicles or not infra_vehicles:
            return matched_pairs  # no matching possible

        ego_positions = np.array([[v["pos"]["lat"], v["pos"]["lon"]] for v in ego_vehicles])
        infra_positions = np.array([[v["pos"]["lat"], v["pos"]["lon"]] for v in infra_vehicles])

        ego_velocities = np.array([[abs(v["vel"]["lat_vel"]), abs(v["vel"]["lon_vel"])] for v in ego_vehicles])
        infra_velocities = np.array([[abs(v["vel"]["lat_vel"]), abs(v["vel"]["lon_vel"])] for v in infra_vehicles])

        # compute euclidean distance matrices for position and velocity
        pos_cost_matrix = distance.cdist(ego_positions, infra_positions, metric="euclidean")
        vel_cost_matrix = distance.cdist(ego_velocities, infra_velocities, metric="euclidean")

        # total cost matrix (weighted sum of position and velocity differences)
        cost_matrix = position_weight * pos_cost_matrix + velocity_weight * vel_cost_matrix

        # solve the assignment problem
        ego_indices, infra_indices = linear_sum_assignment(cost_matrix)

        for ego_idx, infra_idx in zip(ego_indices, infra_indices):
            if cost_matrix[ego_idx, infra_idx] < threshold:  # just accept valid matches
                matched_pairs[ego_vehicles[ego_idx]["uniqueId"]] = infra_vehicles[infra_idx]["uniqueId"]

        return matched_pairs

    def format_output(self, message, actors, assigned_ids, source):
        if not message:
            return None

        output = {
            "timestamp": message["timestamp"],
            "visible_actors": []
        }

        for actor in actors:
            actor_id = actor["uniqueId"]
            assigned_id = assigned_ids.get(actor_id, actor_id)

            new_actor = actor.copy()
            new_actor["uniqueId"] = assigned_id
            output["visible_actors"].append(new_actor)

        msg = String()
        msg.data = json.dumps(output)
        return msg

    def cleanup_tracking(self):
        """
        Remove objects that have not been seen for a certain number of timesteps.
        """
        for obj_id in list(self.matched_ids.keys()):
            self.object_lifetime[obj_id] += 1
            if self.object_lifetime[obj_id] > self.max_lost_time:
                del self.matched_ids[obj_id]
                del self.object_lifetime[obj_id]


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMatcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

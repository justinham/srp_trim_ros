import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import numpy as np
from typing import List, Tuple
import time

Point = Tuple[float, float]

vertices = [
    (-8.685, 8.435), (-10.352, 69.783), (3.972, 70.2), (6.08, 13.961), (11.54, 14.077),
    (11.761, 2.216), (43.12, 2.653), (42.52, -8.864), (12.054, -9.231), (12.055, -9.63), (12.21, -14.2), (6.367, -14.48),
    (7.795, -69.954), (-5.1, -69.628), (-7.317, -15.27), (-16.3, -15.161), (-17.457, -2.147), (-41.705, -2.701), (-42.142, 7.736)
]

closed_track_vertices = [
    (-28.9, 571.93), (-78.31, 569.44), (-77.86, 557.42), (-28, 560.04)
]

existing_obj_ids = set()

def point_in_cross(p: Point, eps: float = 1e-12) -> bool:
    #return True if point p is inside (or on the boundary of) the polygon defined by `vertices`
    x, y = p
    def on_segment(a: Point, b: Point, p: Point) -> bool:
        (x1, y1), (x2, y2) = a, b
        cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        if abs(cross) > eps:
            return False
        return (min(x1, x2) - eps <= x <= max(x1, x2) + eps and
                min(y1, y2) - eps <= y <= max(y1, y2) + eps)

    n = len(vertices)
    inside = False
    for i in range(n):
        a = vertices[i]
        b = vertices[(i + 1) % n]
        if on_segment(a, b, p):
            return True
        xi, yi = a
        xj, yj = b
        intersects = ( (yi > y) != (yj > y) )
        if intersects:
            x_at_y = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x_at_y >= x - eps:
                inside = not inside

    return inside

class InfraToGlobal(Node):
    def __init__(self):
        super().__init__('infra_to_global')

        # self.subscriber_id_pos = self.create_subscription(String, 'ego_id', self.ego_id_callback, 10)
        self.subscriber_ = self.create_subscription(String, 'infra_local_measures', self.infra_callback, 10)
        self.publisher_ = self.create_publisher(String, 'infra_global_measures', 10)
        self.ego_subscriber = self.create_subscription(String, "ego_states", self.ego_callback, 10)

        self.mode_arg = self.declare_parameter('mode_arg', 'sim').value
        self.infra_ref_pos = {"x": 0.0, "y": 0.0}
        self.ego_id = None
        self.ego_data = None

        self.objs_headings, self.track_curr_objs = {}, {}

    def ego_callback(self, msg):
        self.ego_data = json.loads(msg.data)['ego_vehicle']
        self.ego_data["heading"]["theta"] += 90
        # self.publish_json_msg['timestamp'] = time.time()

    def ego_id_callback(self, msg):
        data = json.loads(msg.data)
        self.ego_id = data["uniqueId"]
        
    def infra_callback(self, msg):

        data = json.loads(msg.data)
        visible_actors = data["sdsmData"]
        timestamp = float(data['timestamp'])/1000

        json_data = {
            "timestamp": timestamp,
            "visible_actors": []
        }

        curr_obj_ids = []

        for actor_id in self.track_curr_objs:
            self.track_curr_objs[actor_id] += 1

        if self.infra_ref_pos != None:
            new_visible_actors = []
            for actor in visible_actors:
                actor_id = actor['uniqueId']
                if actor_id in curr_obj_ids:
                    continue
                self.track_curr_objs[actor_id] = 0
                if actor['uniqueId'] not in existing_obj_ids and float(actor["speed"]) == 0.0:  # noisy data from sdsm
                    continue
                existing_obj_ids.add(actor_id)
                heading = float(actor["heading"])
                if heading in [0.0, 180.0] or float(actor["speed"]) == 0.0: # handling heading problem from sdsm
                    if actor_id in self.objs_headings:
                        heading = self.objs_headings[actor_id]
                else:
                    self.objs_headings[actor_id] = heading
                actor2 = {
                    "uniqueId": actor['uniqueId'],
                    "class": actor["class"],
                    "pos": {
                        "lon": float(actor["pos"]["offsetX"])/10,
                        "lat": float(actor["pos"]["offsetY"])/10,
                        "posStdDev_lat": float(actor["posStdDev"]),
                        "posStdDev_lon": float(actor["posStdDev"])
                    },
                    "vel": {
                        "lon_vel": float(actor["speed"])*np.cos(heading*np.pi/180),
                        "lat_vel": float(actor["speed"])*np.sin(heading*np.pi/180),
                        "velStdDev_lat": float(actor["speedStdDev"]),
                        "velStdDev_lon": float(actor["speedStdDev"])
                    },
                    "heading": {
                        # "theta": (heading + 180)%360 - 180,
                        "theta": -heading+90,
                        "theta_valid": True,
                        "thetaStdDev": float(actor["headingStdDev"])
                    },
                    "halfDim": None
                }
                # self.infra_to_global_transform(actor)
                # self.get_logger().info(f"uniqueId is {(actor['uniqueId'])} with id {type(actor['uniqueId'])}")
                # self.get_logger().info(f"self_ego_id is {(self.ego_id)} with id {type(self.ego_id)}")
                # if actor['uniqueId'] == self.ego_id:
                #     json_data['ego_vehicle'] = actor
                #     temp_ego_id = json_data['ego_vehicle']['uniqueId']
                # else:
                if self.ego_data is not None:
                    if np.linalg.norm(np.array([actor2['pos']['lat'], actor2['pos']['lon']]) - np.array([self.ego_data['pos']['lat'], self.ego_data['pos']['lon']])) > 3.5: #filtering ego from derq estimates
                        new_visible_actors.append(actor2)
                        curr_obj_ids.append(actor_id)
                else:
                    new_visible_actors.append(actor2)
                    curr_obj_ids.append(actor['uniqueId'])
                # if point_in_cross((actor["pos"]["lat"], actor["pos"]["lon"])):
                #     new_visible_actors.append(actor2)
                # new_visible_actors.append(actor2)
            json_data['visible_actors'] = new_visible_actors

        keys_to_delete = []
        for actor_id, count in self.track_curr_objs.items():
            if count > 5:
                keys_to_delete.append(actor_id)
        for actor_id in keys_to_delete:
            del self.track_curr_objs[actor_id]
            if actor_id in self.objs_headings:
                del self.objs_headings[actor_id]
            existing_obj_ids.discard(actor_id)
        
        updated_msg = String()
        updated_msg.data = json.dumps(json_data)
        # self.get_logger().info(f"Infrastructure l {data}")
        self.publisher_.publish(updated_msg)
        self.get_logger().info(f"Infrastructure l2g {json_data}")

    def infra_to_global_transform(self, actor):
        offset_x = actor["pos"]["lat"]
        offset_y = actor["pos"]["lon"]

        infra_ref_x = self.infra_ref_pos['x']
        infra_ref_y = self.infra_ref_pos['y']

        global_x = infra_ref_x + offset_x
        global_y = infra_ref_y + offset_y

        remote_global_heading = actor["heading"]["theta"]

        # if self.mode_arg == "real":
        #     remote_global_heading += 90

        # actor["heading"]["theta"] = (remote_global_heading + 180)%360 - 180

        actor["pos"]["lat"], actor["pos"]["lon"] = global_x, global_y

        return actor
    
def main(args=None):
    rclpy.init(args=args)
    node = InfraToGlobal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
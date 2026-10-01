import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String, Int32
import numpy as np
import math

from typing import List, Tuple

Point = Tuple[float, float]

vertices = [
    (-8.685, 8.435), (-10.352, 69.783), (3.972, 70.2), (6.08, 13.961), (11.54, 14.077),
    (11.761, 2.216), (43.12, 2.653), (42.52, -8.864), (12.054, -9.231), (12.055, -9.63), (12.21, -14.2), (6.367, -14.48),
    (7.795, -69.954), (-5.1, -69.628), (-7.317, -15.27), (-16.3, -15.161), (-17.457, -2.147), (-41.705, -2.701), (-42.142, 7.736)
]

closed_track_vertices = [
    (-28.9, 571.93), (-78.31, 569.44), (-77.86, 557.42), (-28, 560.04)
]

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

class EgoToGlobal(Node):
    def __init__(self):
        super().__init__('ego_to_global')

        self.subscriber_ = self.create_subscription(String, 'ego_local_measures', self.ego_sensing_callback, 10)
        self.publisher_ego_observation = self.create_publisher(String, 'ego_global_measures', 10)
        # self.publisher_ego_id = self.create_publisher(String, 'ego_id', 10)
        self.ego_gps_subscriber = self.create_subscription(String, "ego_states", self.ego_gps_callback, 10)
        self.ego_vehicle = None


        self.mode_arg = self.declare_parameter('mode_arg', 'real').value

        self.ref_lat, self.ref_lon = 42.5150354710, -83.0439986570
        self.ego_global_pos, self.ego_global_head = None, None

        self.crosswalk_coords = [(-14.08, 10.39), (9.18, 10.72), (10.08, -12.27), (-14, -12.94), (-14.08, 10.39)]
        self.slopes_clockwise = [
            (self.crosswalk_coords[1][1]-self.crosswalk_coords[0][1])/((self.crosswalk_coords[1][0]-self.crosswalk_coords[0][0])),
            (self.crosswalk_coords[2][1]-self.crosswalk_coords[1][1])/((self.crosswalk_coords[2][0]-self.crosswalk_coords[1][0])),
            (self.crosswalk_coords[3][1]-self.crosswalk_coords[2][1])/((self.crosswalk_coords[3][0]-self.crosswalk_coords[2][0])),
            (self.crosswalk_coords[4][1]-self.crosswalk_coords[3][1])/((self.crosswalk_coords[4][0]-self.crosswalk_coords[3][0]))
        ]
        self.objs_prev_pos = {}

    def ego_gps_callback(self, msg):
        data = json.loads(msg.data)
        self.ego_vehicle = data["ego_vehicle"]
    
    def ego_sensing_callback(self, msg):
        data = json.loads(msg.data)

        ego_vehicle = self.ego_vehicle
        if ego_vehicle is None:
            return
        visible_actors = data["visible_actors"]

        ego_global_position = ego_vehicle["pos"]
        ego_global_heading = ego_vehicle["heading"]["theta"]
        self.ego_global_pos, self.ego_global_head = ego_global_position, ego_global_heading

        new_visible_actors = []
        for actor in visible_actors:
            local_cov = [actor["pos"]["posStdDev_lat"], actor["pos"]["posStdDev_lon"], 
                         actor["vel"]["velStdDev_lat"], actor["vel"]["velStdDev_lon"], actor["heading"]["thetaStdDev"]]
            self.local_to_global_transform(actor, ego_global_position, ego_global_heading, local_cov)

            # Filter
            # if actor['class'] == 'vru':
            #     for i in range(4):
            #         x1, y1 = self.crosswalk_coords[i]
            #         x2, y2 = self.crosswalk_coords[i+1]
            #         x, y = actor['pos']['lat'], actor['pos']['lon']
            #         prep_dist = abs((y1-y2)*x + (x2-x1)*y + x1*y2 - x2*y1)/(np.sqrt((y1-y2)**2 + (x2-x1)**2))  
            #         if prep_dist < 6:
            #             if actor['uniqueId'] in self.objs_prev_pos:
            #                 line_vector = np.array([(x2-x1), (y2-y1)])
            #                 x1, y1 = self.objs_prev_pos[actor['uniqueId']]
            #                 obj_vector = np.array([x-x1, y-y1])
            #                 # obj_vector = np.array([1, np.radians(actor['heading']['theta'])])
            #                 # self.get_logger().info(f"here are {line_vector}, {obj_vector}")
            #                 if np.dot(line_vector, obj_vector) >= 0:
            #                     actor['heading']['theta'] = math.degrees(math.atan(self.slopes_clockwise[i]))
            #                 else:
            #                     actor['heading']['theta'] = math.degrees(math.atan(self.slopes_clockwise[i])+np.pi)
            #                 # actor['heading']['theta'] = -90.0

            #             new_visible_actors.append(actor)
            #             self.objs_prev_pos[actor['uniqueId']] = (x, y)
            #             break
            # else:
            #     if math.sqrt((ego_global_position["lat"]-actor["pos"]["lat"])**2 + (ego_global_position["lon"]-actor["pos"]["lon"])**2) < 20:
            #         if point_in_cross((actor["pos"]["lat"], actor["pos"]["lon"])):
            #             new_visible_actors.append(actor)
            new_visible_actors.append(actor)

        data["visible_actors"] = new_visible_actors

        ego_vehicle["heading"]["theta"] = ego_global_heading + 90
        updated_msg = String()
        updated_msg.data = json.dumps(data)
        self.get_logger().info("Ego l2g " + updated_msg.data)
        self.publisher_ego_observation.publish(updated_msg)
        # self.get_logger().info(updated_msg.data)
        # updated_msg = String()
        # self.get_logger().info(type(ego_vehicle["uniqueId"]))
        # updated_msg.data = json.dumps({"uniqueId": ego_vehicle["uniqueId"]})
        # self.publisher_ego_id.publish(updated_msg)
    
    def local_to_global_transform(self, actor, ego_global_position, ego_global_heading, local_cov):

        local_position = actor["pos"]
        local_vel = actor["vel"]
        # local_speed = actor["speed"]
        local_heading = actor["heading"]["theta"]

        ego_x, ego_y = ego_global_position["lat"], ego_global_position["lon"]

        local_x, local_y = local_position["lat"], local_position["lon"]
        local_vel_x, local_vel_y = local_vel["lat_vel"], local_vel["lon_vel"]

        global_x = ego_x + local_x * np.cos(ego_global_heading*np.pi/180) - local_y * np.sin(ego_global_heading*np.pi/180)
        global_y = ego_y + local_x * np.sin(ego_global_heading*np.pi/180) + local_y * np.cos(ego_global_heading*np.pi/180)

        global_vel_x = local_vel_x * np.cos(ego_global_heading*np.pi/180) - local_vel_y * np.sin(ego_global_heading*np.pi/180)
        global_vel_y = local_vel_x * np.sin(ego_global_heading*np.pi/180) + local_vel_y * np.cos(ego_global_heading*np.pi/180)

        remote_global_heading = (local_heading + ego_global_heading)

        actor["pos"]["lat"] = global_x
        actor["pos"]["lon"] = global_y
        actor["vel"]["lat_vel"] = global_vel_x
        actor["vel"]["lon_vel"] = global_vel_y
        if self.mode_arg == "real":
            remote_global_heading += 90
        actor["heading"]["theta"] = (remote_global_heading + 180)%360 - 180

        local_cov[0] = min(1, local_cov[0])
        local_cov[1] = min(1, local_cov[1])
        cov_local_pos_matrix = np.array([[local_cov[0], 0], [0, local_cov[1]]])
        cov_local_vel_matrix = np.array([[local_cov[2], 0], [0, local_cov[3]]])
        rotation_matrix = np.array([
            [np.cos(ego_global_heading), -np.sin(ego_global_heading)],
            [np.sin(ego_global_heading), np.cos(ego_global_heading)]
        ])
        cov_global_pos_matrix = rotation_matrix @ cov_local_pos_matrix @ rotation_matrix.T
        cov_global_vel_matrix = rotation_matrix @ cov_local_vel_matrix @ rotation_matrix.T
        actor["pos"]["posStdDev_lat"] = np.sqrt(cov_global_pos_matrix[0, 0])
        actor["pos"]["posStdDev_lon"] = np.sqrt(cov_global_pos_matrix[1, 1])
        actor["vel"]["velStdDev_lat"] = np.sqrt(cov_global_vel_matrix[0, 0])
        actor["vel"]["velStdDev_lon"] = np.sqrt(cov_global_vel_matrix[1, 1])

    def gt_remote_local_to_global_transform(self, actor, ego_global_position, ego_global_heading):

        local_position = actor["pos"]
        # local_speed = actor["speed"]
        local_heading = actor["heading"]

        ego_x, ego_y = ego_global_position["lat"], ego_global_position["lon"]

        local_x, local_y = local_position["lat"], local_position["lon"]

        global_x = ego_x + local_x * np.cos(ego_global_heading*np.pi/180) - local_y * np.sin(ego_global_heading*np.pi/180)
        global_y = ego_y + local_x * np.sin(ego_global_heading*np.pi/180) + local_y * np.cos(ego_global_heading*np.pi/180)

        remote_global_heading = (local_heading + ego_global_heading)

        actor["pos"]["lat"] = global_x
        actor["pos"]["lon"] = global_y
        if self.mode_arg == "real":
            remote_global_heading += 90
        actor["heading"] = (remote_global_heading + 180)%360 - 180
    
def main(args=None):
    rclpy.init(args=args)
    node = EgoToGlobal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
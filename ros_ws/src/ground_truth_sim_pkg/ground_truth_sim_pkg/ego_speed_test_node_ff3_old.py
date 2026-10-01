import rclpy
from rclpy.node import Node
import numpy as np
import json
from std_msgs.msg import String
import csv
import numpy as np
from fusion.msg import GPS
import time
from std_msgs.msg import Float64

from typing import List, Tuple, Dict, Optional
import time
import math
from fusion.msg import TripleVectorWps

Point = Tuple[float, float]

rab_inter_plus_lanes_vertices = [
    (-8.685, 8.435), (-10.352, 69.783), (3.972, 70.2), (6.08, 13.961), (11.54, 14.077),
    (11.761, 2.216), (43.12, 2.653), (42.52, -8.864), (12.054, -9.231), (12.055, -9.63), (12.21, -14.2), (6.367, -14.48),
    (7.795, -69.954), (-5.1, -69.628), (-7.317, -15.27), (-16.3, -15.161), (-17.457, -2.147), (-41.705, -2.701), (-42.142, 7.736)
]

rab_inter_vertices = [
    (-16.05, 11.32), (10.72, 12.07), (12.19, -16.41), (-16.53, -17.72)
]

existing_obj_ids = []

def angle_diff(a, b):
    d = (a - b + np.pi) % (2*np.pi) - np.pi
    return d

def angles_within_range(angle_list, query_angle, deg=30):
    threshold = np.deg2rad(deg)

    # compute wrap-safe differences
    diffs = np.array([angle_diff(a, query_angle) for a in angle_list])

    # pick the ones within ±threshold
    mask = np.abs(diffs) <= threshold
    return np.array(angle_list)[mask]

def point_in_polygon(obj_xy, eps: float = 1e-12, code: str = 'lanes') -> bool:
    
    #return True if point p is inside (or on the boundary of) the polygon defined by `vertices`
    x, y = obj_xy[0], obj_xy[1]
    def on_segment(a: Point, b: Point) -> bool:
        (x1, y1), (x2, y2) = a, b
        cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        if abs(cross) > eps:
            return False
        return (min(x1, x2) - eps <= x <= max(x1, x2) + eps and
                min(y1, y2) - eps <= y <= max(y1, y2) + eps)
    if code == "lanes":
        vertices = rab_inter_plus_lanes_vertices
    else:
        vertices = rab_inter_vertices
    n = len(vertices)
    inside = False
    for i in range(n):
        a = vertices[i]
        b = vertices[(i + 1) % n]
        if on_segment(a, b):
            return True
        xi, yi = a
        xj, yj = b
        intersects = ( (yi > y) != (yj > y) )
        if intersects:
            x_at_y = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x_at_y >= x - eps:
                inside = not inside

    return inside

GLOBAL_LANE_SEGMENTS: Dict[str, Dict[str, Tuple[float, float]]] = {}

def _point_to_segment_distance(px: float, py: float,
                               x0: float, y0: float,
                               x1: float, y1: float) -> Tuple[float, float]:
    """
    Returns (distance_m, t) where t in [0,1] is the clamped projection param
    of P onto the segment P0->P1 (t=0 at start, t=1 at end).
    """
    vx, vy = x1 - x0, y1 - y0
    wx, wy = px - x0, py - y0
    seg_len2 = vx*vx + vy*vy
    if seg_len2 == 0.0:
        # Degenerate segment: distance to the single point
        dx, dy = px - x0, py - y0
        return math.hypot(dx, dy), 0.0
    t = (wx*vx + wy*vy) / seg_len2
    t = max(0.0, min(1.0, t))
    projx, projy = x0 + t*vx, y0 + t*vy
    return math.hypot(px - projx, py - projy), t

def load_lane_boundaries(json_path: str) -> Dict[int, dict]:
    global GLOBAL_LANE_SEGMENTS
    GLOBAL_LANE_SEGMENTS.clear()

    with open(json_path, "r") as f:
        data = json.load(f)

    # The file structure has a top-level list with one intersection object.
    inter = data[0]
    default_lane_width_cm = inter.get("laneWidth", 400)  # fallback if missing

    for lane in inter["laneSet"]:
        lid = int(lane["laneID"])
        nodes = lane["nodeList"]["nodes"]
        # Use first and last nodes as a segment representation
        x0, y0 = nodes[0]["xy"]
        x1, y1 = nodes[-1]["xy"]

        lane_width_cm = lane.get("laneWidth", default_lane_width_cm)
        lane_width_m = lane_width_cm / 100.0

        manv_options = {}
        for manv_option in lane['connectsTo']:
            manv_options[manv_option['connectingLane']['maneuver']] = f"l{manv_option['connectingLane']['lane']}"

        GLOBAL_LANE_SEGMENTS[f'l{lid}'] = {
            "p0": (float(x0), float(y0)),
            "p1": (float(x1), float(y1)),
            "half_width": lane_width_m * 0.5,
            "manv": manv_options
        }

    return GLOBAL_LANE_SEGMENTS

def lane_of_point(obj_xy,
                  margin_m: float = 0.25,
                  require_inside_width: bool = True) -> Optional[int]:
    if not GLOBAL_LANE_SEGMENTS:
        raise RuntimeError("GLOBAL_LANE_SEGMENTS is empty. Call load_lane_boundaries(..) first")

    best_lane = None
    best_dist = float("inf")

    x, y = obj_xy[0], obj_xy[1]

    for lid, seg in GLOBAL_LANE_SEGMENTS.items():
        (x0, y0), (x1, y1) = seg["p0"], seg["p1"]
        d, _ = _point_to_segment_distance(x, y, x0, y0, x1, y1)
        threshold = seg["half_width"] + margin_m

        if require_inside_width:
            if d <= threshold and d < best_dist:
                best_dist = d
                best_lane = lid
        else:
            if d < best_dist:
                best_dist = d
                best_lane = lid

    return best_lane

def convert_lat_lon_to_xy(ref_lat_rad, ref_lon_rad, ref_heading_rad, lat_rad,lon_rad):
    """
    Computes xy distances between two sets of lat/lon
    """
    f = 0.003353
    a = 6378137
    f1 = pow((f*(2-f)),0.5)
    f2 = a*(1-pow(f1,2))/pow((1-pow(f1,2)*pow(np.sin(ref_lat_rad),2)),3.0/2.0)
    f3 = a/pow((1-pow(f1,2)*pow((np.sin(ref_lat_rad)),2)),(1.0/2.0))
    E = f3 * np.cos(ref_lat_rad) * (lon_rad - ref_lon_rad)
    N = f2 * (lat_rad - ref_lat_rad)
    x_m = N*np.cos(ref_heading_rad) + E*np.sin(ref_heading_rad)
    y_m = - N*np.sin(ref_heading_rad)+ E*np.cos(ref_heading_rad)
    d2d_m = pow((pow(x_m,2)+pow(y_m,2)),0.5)
    return [x_m, y_m,d2d_m]

class EgoSpeedTestNode(Node):
    def __init__(self):
        super().__init__('ego_speed_test_node_ff3')

        self.gps_subscriber = self.create_subscription(GPS, 'gps', self.gps_callback, 10)
        self.speed_publisher = self.create_publisher(Float64, 'ego_speed_cmd', 10)
        self.obstacles_id_publisher = self.create_publisher(String, 'obstacles_id_data', 10)
        self.prev_heading = {}

        self.ego_lane = 'l3'
        self.turn_signal_subscriber = self.create_subscription(String, 'ego_turn_signal', self.turn_signal_listener, 10)
        self.ego_turn_signal = 'L'

        self.subscription = None
        self.selected_topic = 'infra_actor_states'
        self.source = 'infra'
        self.topic_selection_sub = self.create_subscription(String, '/selected_topic', self.topic_selection_callback, 10)
        self.path_wps_pub = self.create_subscription(String, '/selected_direction', self.dir_selection_callback, 10)
        self.ing_lane_selection_sub = self.create_subscription(String, '/selected_ingress_lane', self.ing_lane_selection_callback, 10)
        self.publisher_ = self.create_publisher(TripleVectorWps, 'interpolated_wps', 10)
        self.subscribe_to_topic(self.selected_topic)
        self.ego_wps_publisher = self.create_publisher(String, 'ego_waypoints', 10)
        self.rv_wps_publisher = self.create_publisher(String, 'actors_waypoints', 10)
        self.last_updated_wps_time = time.time()

        self.origin_lat, self.origin_lon = 42.520156105740654, -83.04382593113446 # closed track origin
        # self.origin_lat, self.origin_lon = 42.51635737397318, -83.0434636223155 # garage
        self.origin_lat, self.origin_lon = 42.5150354710, -83.0439986570 # RAB
        self.wps_w_speed = [[-74.19,-3.58], [-153.31,-7.7], [-187.22,-35.1], [-159.34,-59.76], [-102.64, -33.25]] # closed track
        self.wps_w_speed = [[-3.32, 10.28], [12.38, -5.78], [42.56, -5.05]] # RAB (l1-l4) (just initializing variable)
        self.wps_des_speeds = [0, 2, 0]
        self.curr_speed_wp_tracking_id = 0
        self.prev_speed_cmd = 0.0
        self.est_msg = None
        self.is_wp_decel_unlatched = False

        self.ego_xy = None
        self.ego_speed = 0
        self.ego_heading = 0
        self.ego_v_max, self.ego_a_max = 2, 1
        self.ego_a_min = -3
        self.prev_timestamp = 0
        self.vru_proj_future_t = 8
        self.vru_proj_past_t = 1
        self.proj_wp_sampling_time = 0.5
        self.vrus_wps = {}

        self.lanes_connects = []
        self.lanes_wps = []
        self.ego_init_lane, self.ego_manv = 5, 'R'
        self.ingress, self.egress = "l3", "l8"
        self.selected_dir = 'S'
        self.curr_ego_index = 0
        self.prev_ego_index = 0
        self.lookahead_time = 8 # sec

        load_lane_boundaries("/home/yz4d3h/ConnAu/Data/Intersections/intersection_database_gmaps.json")

        self.is_obstacle_on_way = 0
        self.is_ego_stopped_for_obstacle = 0
        self.obs_start_time = None
        self.is_rv_on_way = False

        file_path = "log"
        self.gps_file = open(file_path + "-speedu.csv", 'w+')
        self.gps_file.write("timestamp,ego_speed,ego_speed_cmd,ego_x,ego_y,ego_acc_unf,ego_acc,stopping_dist,speed_wp_track_id,ego_track_id_in_traj,is_obs_detected\n")

        self.offline_pts = []
        self.offline_pts_xy_only = []
        self.load_offline_pts()
        self.offline_pts_rv = []
        self.offline_pts_xy_only_rv = []
        # self.load_offline_pts_rv()

        self.start_time = time.time()
        self.prev_ts = time.monotonic()

        self.timer = self.create_timer(0.1, self.listener_callback)
        self.last_est_time = time.monotonic()

    def turn_signal_listener(self, msg):
        turn_signal = msg.data
        if turn_signal == 'S' and self.ego_lane != self.ingress:
            return
        self.ego_turn_signal = turn_signal
        self.egress = GLOBAL_LANE_SEGMENTS[self.ingress]['manv'][turn_signal]
        self.load_offline_pts()
        self.curr_speed_wp_tracking_id = 0
        self.curr_ego_index = 0

    def dir_selection_callback(self, msg):
        new_dir = msg.data[0]
        self.get_logger().info(f"new direction: {new_dir}")
        if self.selected_dir != new_dir:
            self.selected_dir = new_dir
            self.egress = GLOBAL_LANE_SEGMENTS[self.ingress]['manv'][new_dir]
            self.load_offline_pts()
            self.curr_speed_wp_tracking_id = 0
            self.curr_ego_index = 0

    def ing_lane_selection_callback(self, msg):
        new_lane_id = msg.data[-3:-1]
        self.get_logger().info(f"new ingress: {new_lane_id}")
        if new_lane_id != self.ingress:
            self.ingress = new_lane_id
            self.egress = GLOBAL_LANE_SEGMENTS[self.ingress]['manv']['S']
            self.load_offline_pts()
            self.curr_speed_wp_tracking_id = 0
            self.curr_ego_index = 0

    def topic_selection_callback(self, msg):
        new_topic = msg.data
        if new_topic != self.selected_topic:
            self.get_logger().info(f"Switching to topic: {new_topic}")
            self.selected_topic = new_topic
            self.source = new_topic.split('_')[0]
            self.subscribe_to_topic(self.selected_topic)

    def subscribe_to_topic(self, topic_name):
        if self.subscription:
            self.destroy_subscription(self.subscription)

        self.subscription = self.create_subscription(
            String, topic_name, self.estimates_listener_callback, 10
        )
        self.get_logger().info(f"Subscribed to {topic_name}")

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        heading = msg.heading
        self.ego_speed = msg.speed

        local_x, local_y, _ = convert_lat_lon_to_xy(self.origin_lat*np.pi/180, self.origin_lon*np.pi/180, 0, latitude*np.pi/180, longitude*np.pi/180)
        self.ego_xy = np.array([local_y, local_x])
        self.ego_heading = -heading + np.pi/2
        # self.get_logger().info(f"ego states: {local_x}, {local_y}")
        self.ego_lane = lane_of_point(self.ego_xy)
        if self.ego_lane != self.ingress:
            if self.ego_lane in ['l1', 'l3', 'l5', 'l7']:
                self.ingress = self.ego_lane
                self.egress = GLOBAL_LANE_SEGMENTS[self.ingress]['manv']['S']
                self.load_offline_pts()
                self.curr_speed_wp_tracking_id = 0
                self.curr_ego_index = 0

        # self.load_intersection('/connau/ConnAu/Data/Intersections/intersection_database_gmaps.json')

    def load_offline_pts(self):
        csv_name = f'RAB_{self.ingress}_to_{self.egress}_xy.csv'
        try:
            with open(f'/home/yz4d3h/ConnAu/Data/offline_path_files/RAB_paths/{csv_name}', 'r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                self.offline_pts = []
                self.offline_pts_xy_only_rv = []
                msg = TripleVectorWps()
                msg.plan_x, msg.plan_y, msg.plan_h = [], [], []
                for row in reader:
                    self.offline_pts.append([float(row['x']), float(row['y']), float(row['head_rad'])])
                    self.offline_pts_xy_only.append([float(row['x']), float(row['y'])])
                    msg.plan_x.append(float(row['x']))
                    msg.plan_y.append(float(row['y']))
                    msg.plan_h.append(float(row['head_rad']))
                # self.path_wps_pub.publish(msg)
                self.get_logger().info('Publishing triple vectors to steering control node')
                self.wps_w_speed = []
                for i in range(100, len(self.offline_pts_xy_only), 100):
                    self.wps_w_speed.append([self.offline_pts_xy_only[i][0], self.offline_pts_xy_only[i][1]])
                if (len(self.offline_pts_xy_only))%100 == 0:
                    self.wps_w_speed.append([self.offline_pts_xy_only[-1][0], self.offline_pts_xy_only[-1][1]])
            print(f"Log data successfully read from {csv_name}")
        except IOError as e:
            print(f"Error reading from CSV file: {e}")
    
    # def load_offline_pts_rv(self):
    #     csv_name = f'RAB_{self.rv_ingress}_to_{self.rv_egress}_xy.csv'
    #     try:
    #         with open(f'/home/yz4d3h/ConnAu/Data/offline_path_files/RAB_paths/{csv_name}', 'r', newline='') as csvfile:
    #             reader = csv.DictReader(csvfile)
    #             for row in reader:
    #                 self.offline_pts_rv.append([float(row['x']), float(row['y']), float(row['head_rad'])])
    #                 self.offline_pts_xy_only_rv.append([float(row['x']), float(row['y'])])
    #         print(f"Log data successfully read from {csv_name}")
    #     except IOError as e:
    #         print(f"Error reading from CSV file: {e}")

    def estimates_listener_callback(self, msg):
        self.est_msg = msg
        self.last_est_time = time.monotonic()

    def listener_callback(self):
        if time.monotonic() - self.last_est_time > 1:
            self.est_msg = None
        msg = self.est_msg
        if self.ego_xy is None:
            return
        ego_accs_list = [self.ego_a_max]
        if self.is_ego_stopped_for_obstacle == 1:
            if time.monotonic() - self.obs_start_time > 3:
                self.is_ego_stopped_for_obstacle, self.is_obstacle_on_way = 0, 0
                self.get_logger().info("here4")
            # else:
            #     msg = Float64()
            #     msg.data = 0.0
            #     self.speed_publisher.publish(msg)
            #     now = time.monotonic()
            #     self.gps_file.write(f"{now - self.start_time}, {self.ego_speed}, {0.0}, {self.ego_xy[0]}, {self.ego_xy[1]}, {0.0}, {0.0}, {-1.0}, {self.curr_speed_wp_tracking_id}, {self.curr_ego_index}, {1}, {-1}\n")
            #     self.get_logger().info("**************************Obstacle on the way*****************************")
            #     return
        
        if msg is not None:
            data = json.loads(msg.data)
        
        # ego_vehicle = data['ego_vehicle'][0]
        # ego_xy = [ego_vehicle['estimated_position']['x'], ego_vehicle['estimated_position']['y']]
        # self.ego_heading = ego_vehicle['estimated_heading']
        stopping_dist = 0
        now = time.monotonic()
        dt  = max(0.05, min(0.25, now - self.prev_ts))  # clamp dt to keep controller sane
        self.prev_ts = now
        if self.curr_speed_wp_tracking_id == len(self.wps_des_speeds):
            msg = Float64()
            msg.data = 0.0
            self.speed_publisher.publish(msg)
            return

        self.get_logger().info(f"{self.curr_speed_wp_tracking_id}, {self.wps_des_speeds[self.curr_speed_wp_tracking_id]}, {self.wps_w_speed[self.curr_speed_wp_tracking_id]}")  

        ego_acc_unf = self.ego_a_max
        self.vrus_wps = {}
        self.ego_wps = []
        ego_xy = self.ego_xy
        if ego_xy is None:
            self.get_logger().info("Not connected to ego gps")
            return
        min_ego_to_wp_dist = float('inf')
        for i, offline_wp in enumerate(self.offline_pts):
            ego_to_wp_dist = np.linalg.norm(ego_xy - np.array([offline_wp[0], offline_wp[1]]))
            if ego_to_wp_dist < min_ego_to_wp_dist:
                min_ego_to_wp_dist = ego_to_wp_dist
            else:
                self.curr_ego_index = max(i - 1, 0)
                break
        lookahead_dist = max(2.0, self.ego_speed)*self.lookahead_time
        track_dist_along_path = 0
        prev_offline_wp = np.array([self.offline_pts[self.curr_ego_index][0], self.offline_pts[self.curr_ego_index][1]])
        ego_lookahead_wps, ego_lookahead_wps_t = [], []
        ego_output_wps = {"waypoints": []}
        for i, offline_wp in enumerate(self.offline_pts[self.curr_ego_index:]):
            offline_wp = np.array([offline_wp[0], offline_wp[1]])
            if i == 0:
                ego_lookahead_wps.append(offline_wp)
                ego_lookahead_wps_t.append(0)
                continue
            temp_dist = np.linalg.norm(prev_offline_wp - offline_wp)
            ego_lookahead_wps_t.append(ego_lookahead_wps_t[-1]+temp_dist/max(1.0, self.ego_speed))
            track_dist_along_path += temp_dist
            ego_lookahead_wps.append(offline_wp)
            ego_output_wps['waypoints'].append({"x": offline_wp[0], "y": offline_wp[1]})
            prev_offline_wp = offline_wp
            # self.get_logger().info(f"{track_dist_along_path}, {lookahead_dist}")
            if track_dist_along_path > lookahead_dist:
                break
            if (self.curr_ego_index + i) == ((self.curr_speed_wp_tracking_id+1)*100-1) or self.curr_ego_index > ((self.curr_speed_wp_tracking_id+1)*100-1):
            # if [offline_wp[0], offline_wp[1]] == self.wps_w_speed[self.curr_speed_wp_tracking_id]:
                ego_to_swp_dist = track_dist_along_path
                if ego_to_swp_dist < 3:
                    if self.wps_des_speeds[self.curr_speed_wp_tracking_id] == 0:
                        if self.ego_speed < 0.1:
                            self.curr_speed_wp_tracking_id += 1
                            self.get_logger().info("stopped for stop sign")
                            self.is_wp_decel_unlatched = False
                            # self.is_ego_stopped_for_obstacle = 1
                            # self.obs_start_time = time.monotonic()
                            break
                    # elif ego_to_swp_dist < 1.0:
                    elif self.curr_ego_index > ((self.curr_speed_wp_tracking_id+1)*100-1):
                        self.curr_speed_wp_tracking_id += 1
                        self.is_wp_decel_unlatched = False
                        # break

                stopping_dist = ego_to_swp_dist
                stopping_dist2 = max(0.1, ego_to_swp_dist-1.5)
                # self.get_logger().info(f"speed_wp_id: {self.curr_speed_wp_tracking_id}")
                ego_acc_unf = (self.wps_des_speeds[self.curr_speed_wp_tracking_id]**2 - self.ego_speed**2) / (2 * stopping_dist2)
                if ego_acc_unf <= -0.5 or self.is_wp_decel_unlatched:
                    ego_accs_list.append(ego_acc_unf)
                    self.is_wp_decel_unlatched = True
                # break
        
        ego_lane_id = lane_of_point(ego_xy)
        if ego_lane_id is not None:
            ego_to_inter_dist = np.linalg.norm(ego_xy - np.array(GLOBAL_LANE_SEGMENTS[ego_lane_id]["p1"]))
            is_ego_ingress = int(ego_lane_id[1]) in list(range(1,8,2))
        else:
            is_ego_ingress = False
        if msg is not None:
            # self.get_logger().info(f"ObejctData: {data}")
            # self.get_logger().info(f"here2: {len(data['visible_actors'])}")
            # self.get_logger().info(f"here are ego limit wps: {ego_lookahead_wps[0][0]}, {ego_lookahead_wps[0][1]}, {ego_lookahead_wps[-1][0]}, {ego_lookahead_wps[-1][1]}")
            obstacles_id_list = {"ids": []}
            remote_objects_wps_dict = {"paths": []}
            objects = data['visible_actors']
            for object in objects:
                obj_id = object['uniqueId']
                remote_obj_wps = {
                    "id": obj_id,
                    "source": self.source,
                    "waypoints": []
                }
                if object['class'] == 'vru':
                    # self.get_logger().info(f"obj_id is {obj_id}")
                    vru_x, vru_y = object['estimated_position']['x'], object['estimated_position']['y']
                    vru_speed = 1.5
                    vru_heading = object['estimated_heading']
                    vru_heads_approx_list = [0.04, -1.53, -3.1, 1.61]
                    approx_angle = angles_within_range(vru_heads_approx_list, vru_heading)
                    
                    # if len(approx_angle) != 0 and object['speed'] > 0.5:
                    #     vru_heading = approx_angle[0]
                    # else:
                    #     pass
                    if len(approx_angle) != 0:
                        vru_heading = approx_angle[0]
                    # vru_heading = -1.53
                    # vru_heading = -3.1
                    # vru_heading = 1.61
                    # vru_heading = 0.04
                    curr_vru_wp_list = []
                    t_range = np.arange(-self.vru_proj_past_t, self.vru_proj_future_t, self.proj_wp_sampling_time)
                    for t in t_range:
                        temp_vru_x = vru_x + vru_speed * t * np.cos(vru_heading)
                        temp_vru_y = vru_y + vru_speed * t * np.sin(vru_heading)
                        curr_vru_wp_list.append([temp_vru_x, temp_vru_y])
                        remote_obj_wps['waypoints'].append({"x": temp_vru_x, "y": temp_vru_y})
                    min_temp_dist = float('inf')
                    remote_objects_wps_dict['paths'].append(remote_obj_wps)
                    for ego_wp in ego_lookahead_wps:
                        for vru_wp in curr_vru_wp_list:
                            temp_vru_x = vru_wp[0]
                            temp_vru_y = vru_wp[1]
                            # temp_dist = np.linalg.norm([ego_wp[0], ego_wp[1]] - curr_vru_wp_list[-1]) * \
                            #     np.cos(self.ego_heading - np.arctan((curr_vru_wp_list[-1][1] - ego_wp[1])/(curr_vru_wp_list[-1][0] - ego_wp[0])))
                            temp_dist = np.linalg.norm(np.array([ego_wp[0], ego_wp[1]]) - np.array(vru_wp))
                            min_temp_dist = min(temp_dist, min_temp_dist)
                            # self.get_logger().info(f"min temp dist: {min_temp_dist}")
                        if min_temp_dist < 1:
                            obstacles_id_list['ids'].append(obj_id)
                            # self.get_logger().info(f"obstacle: {obj_id}")
                            stopping_dist2 = max(0.1, np.linalg.norm([ego_wp[0], ego_wp[1]] - ego_xy)-5)
                            self.get_logger().info(f"stopping dist: {stopping_dist2}")
                            acc = -self.ego_speed**2 / (2 * stopping_dist2)
                            ego_accs_list.append(max(self.ego_a_min, acc))
                            # remote_objects_wps_dict['paths'].append(remote_obj_wps)
                            self.is_obstacle_on_way = 1
                            if self.ego_speed < 0.1:
                                self.is_ego_stopped_for_obstacle = 1
                                self.obs_start_time = time.monotonic()
                            break
                    self.get_logger().info(f"min_temp_dist = {min_temp_dist}")
                    self.vrus_wps[obj_id] = curr_vru_wp_list

                self.is_rv_on_way = False
                if object['class'] == 'vehicle'and is_ego_ingress:
                    rv_pos = object['estimated_position']
                    rv_xy = np.array([rv_pos['x'], rv_pos['y']])
                    rv_lane_id = lane_of_point([rv_pos['x'], rv_pos['y']])
                    self.get_logger().info(f"rv ingress lane: {rv_lane_id}")
                    if point_in_polygon(rv_xy, code='inner_inter'):
                        self.is_rv_on_way = True
                        obstacles_id_list['ids'].append(obj_id)
                    elif rv_lane_id is not None:
                        if int(rv_lane_id[1]) in list(range(1,8,2)): #check if rv in ingress lane
                            # if not (self.ego_turn_signal == 'R' and (int(self.egress[1]) - int(rv_lane_id[1]) == 1)):
                            rv_to_inter_dist = np.linalg.norm(rv_xy - np.array(GLOBAL_LANE_SEGMENTS[rv_lane_id]["p1"]))
                            if rv_to_inter_dist < (ego_to_inter_dist+3):
                                self.is_rv_on_way = True
                                obstacles_id_list['ids'].append(obj_id)
            if self.is_rv_on_way:
                ego_inter_acc = -self.ego_speed**2/(2*ego_to_inter_dist)
                ego_accs_list.append(ego_inter_acc)

            msg = String()
            msg.data = json.dumps(obstacles_id_list)
            self.obstacles_id_publisher.publish(msg)
            self.get_logger().info(f"ids are {obstacles_id_list}")

            rv_wps_msg = String()
            rv_wps_msg.data = json.dumps(remote_objects_wps_dict)
            # self.get_logger().info(f"Sending wps {ego_wps_msg.data}")
            # self.rv_wps_publisher.publish(rv_wps_msg)

        ego_acc = max(self.ego_a_min, min(ego_accs_list))
        ego_speed_cmd = float(max(0, min(self.prev_speed_cmd + ego_acc * dt, self.ego_v_max)))
        if self.is_ego_stopped_for_obstacle:
            ego_speed_cmd = 0.0
        else:
            self.last_updated_wps_time = time.time()
        self.prev_speed_cmd = ego_speed_cmd

        self.get_logger().info(f"{ego_accs_list}")

        # if time.time() - self.last_updated_wps_time > 0.1:
        ego_wps_msg = String()
        ego_wps_msg.data = json.dumps(ego_output_wps)
        # self.get_logger().info(f"Sending wps {ego_wps_msg.data}")
        self.ego_wps_publisher.publish(ego_wps_msg)

        msg = Float64()
        msg.data = ego_speed_cmd
        self.speed_publisher.publish(msg)
        self.gps_file.write(f"{now - self.start_time}, {self.ego_speed}, {ego_speed_cmd}, {ego_xy[0]}, {ego_xy[1]}, {ego_acc_unf}, {ego_acc}, {stopping_dist}, {self.curr_speed_wp_tracking_id}, {self.curr_ego_index}, {self.is_obstacle_on_way}\n")
        self.prev_ego_index = self.curr_ego_index

        # self.get_logger().info(f'Publishing: "{msg.data}"')
        # self.get_logger().info(
        #     f"u={self.ego_speed:.3f}, v_cmd={ego_speed_cmd:.3f}, swp_d={stopping_dist:.1f}, v_swp={self.wps_des_speeds[self.curr_speed_wp_tracking_id]:.2f}, swp_id={self.curr_speed_wp_tracking_id}, w_id={self.curr_ego_index}"
        # )

def main(args=None):
    rclpy.init(args=args)
    node = EgoSpeedTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
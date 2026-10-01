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
        super().__init__('ego_speed_test_node_ff')

        self.gps_subscriber = self.create_subscription(GPS, 'gps', self.gps_callback, 10)
        self.publisher_ = self.create_publisher(Float64, 'ego_speed_cmd', 10)
        self.prev_heading = {}

        self.origin_lat, self.origin_lon = 42.520156105740654, -83.04382593113446 # closed track origin
        # self.origin_lat, self.origin_lon = 42.51635737397318, -83.0434636223155 # garage
        self.wps_w_speed = [[-74.19,-3.58], [-153.31,-7.7], [-187.22,-35.1], [-159.34,-59.76], [-102.64, -33.25]]
        self.wps_des_speeds = [1, 0, 0.5, 1, 0]
        self.curr_speed_wp_tracking_id = 0
        self.prev_speed_cmd = 0.0

        self.ego_xy = np.array([-1, -1])
        self.ego_speed = 0
        self.ego_heading = 0
        self.ego_v_max, self.ego_a_max = 3, 1
        self.ego_a_min = -3
        self.prev_timestamp = 0

        self.lanes_connects = []
        self.lanes_wps = []
        self.ego_init_lane, self.ego_manv = 5, 'L'
        self.curr_ego_index = 0
        self.prev_ego_index = 0
        self.lookahead_time = 8 # sec

        self.is_obstacle_on_way = 0
        self.is_ego_stopped_for_obstacle = 0
        self.obs_start_time = None

        file_path = "log"
        self.gps_file = open(file_path + "-speedu.csv", 'w+')
        self.gps_file.write("timestamp,ego_speed,ego_speed_cmd,ego_x,ego_y,ego_acc_unf,ego_acc,heading,speed_wp_track_id,ego_track_id_in_traj\n")
        self.offline_pts = []
        self.offline_pts_xy_only = []
        self.load_offline_pts()

        self.start_time = time.time()
        self.prev_ts = time.monotonic()

        self.timer = self.create_timer(0.1, self.listener_callback)

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        heading = msg.heading_f
        self.ego_speed = msg.speed_f

        local_x, local_y, _ = convert_lat_lon_to_xy(self.origin_lat*np.pi/180, self.origin_lon*np.pi/180, 0, latitude*np.pi/180, longitude*np.pi/180)
        self.ego_xy = np.array([local_y, local_x])
        self.ego_heading = -heading + np.pi/2

        # self.load_intersection('/connau/ConnAu/Data/Intersections/intersection_database_gmaps.json')

    def load_offline_pts(self):
        csv_name = 'closed_track_speed_test_traj.csv'
        try:
            with open(f'/home/yz4d3h/ConnAu/Data/offline_path_files/{csv_name}', 'r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    self.offline_pts.append([float(row['x']), float(row['y']), float(row['head_rad'])])
                    self.offline_pts_xy_only.append([float(row['x']), float(row['y'])])
            print(f"Log data successfully read from {csv_name}")
        except IOError as e:
            print(f"Error reading from CSV file: {e}")

    def listener_callback(self):
        if self.is_ego_stopped_for_obstacle == 1:
            if time.monotonic() - self.obs_start_time > 3:
                self.is_ego_stopped_for_obstacle, self.is_obstacle_on_way = 0, 0
            else:
                msg = Float64()
                msg.data = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info("**************************Obstacle on the way*****************************")
                return
                
        stopping_dist = 0
        now = time.monotonic()
        dt  = max(0.05, min(0.25, now - self.prev_ts))  # clamp dt to keep controller sane
        self.prev_ts = now
        if self.curr_speed_wp_tracking_id == len(self.wps_des_speeds):
            msg = Float64()
            msg.data = 0.0
            self.publisher_.publish(msg)
            return

        ego_acc_unf = self.ego_a_max
        self.vrus_wps = {}
        self.ego_wps = []
        ego_xy = self.ego_xy
        if ego_xy is None:
            self.get_logger().info("Not connected to ego gps")
            return
        min_ego_to_wp_dist = float('inf')
        for i, offline_wp in enumerate(self.offline_pts[self.curr_ego_index:]):
            ego_to_wp_dist = np.linalg.norm(ego_xy - np.array([offline_wp[0], offline_wp[1]]))
            if ego_to_wp_dist < min_ego_to_wp_dist:
                min_ego_to_wp_dist = ego_to_wp_dist
            else:
                self.curr_ego_index = max(i + self.curr_ego_index - 1, 0)
                break
        lookahead_dist = max(2, self.ego_speed)*self.lookahead_time
        track_dist_along_path = 0
        prev_offline_wp = ego_xy
        ego_lookahead_wps = []
        for i, offline_wp in enumerate(self.offline_pts[self.curr_ego_index:]):
            offline_wp = np.array([offline_wp[0], offline_wp[1]])
            temp_dist = np.linalg.norm(prev_offline_wp - offline_wp)
            track_dist_along_path += temp_dist
            ego_lookahead_wps.append(offline_wp)
            prev_offline_wp = offline_wp
            # self.get_logger().info(f"{track_dist_along_path}, {lookahead_dist}")
            if track_dist_along_path > lookahead_dist:
                break
            if [offline_wp[0], offline_wp[1]] == self.wps_w_speed[self.curr_speed_wp_tracking_id]:
                ego_to_swp_dist = track_dist_along_path
                prev_ego_to_swp_dist = np.linalg.norm(np.array([self.offline_pts[self.prev_ego_index][0],
                                                                self.offline_pts[self.prev_ego_index][1]]) - np.array([offline_wp[0], offline_wp[1]]))
                curr_ego_to_swp_dist = np.linalg.norm(np.array([self.offline_pts[self.curr_ego_index][0],
                                                                self.offline_pts[self.curr_ego_index][1]]) - np.array([offline_wp[0], offline_wp[1]]))
                if ego_to_swp_dist < 3:
                    if self.wps_des_speeds[self.curr_speed_wp_tracking_id] == 0:
                        if self.ego_speed < 0.1:
                            self.curr_speed_wp_tracking_id += 1
                            self.is_ego_stopped_for_obstacle = 1
                            self.obs_start_time = time.monotonic()
                            break
                    elif ego_to_swp_dist < 1.5:
                        self.curr_speed_wp_tracking_id += 1
                        break

                stopping_dist = ego_to_swp_dist
                stopping_dist2 = max(0.1, ego_to_swp_dist-1.5)
                self.get_logger().info(f"speed_wp_id: {self.curr_speed_wp_tracking_id}")
                ego_acc_unf = (self.wps_des_speeds[self.curr_speed_wp_tracking_id]**2 - self.ego_speed**2) / (2 * stopping_dist2)
                break

        ego_acc = max(self.ego_a_min, min(self.ego_a_max, ego_acc_unf))
        ego_speed_cmd = float(max(0, min(self.prev_speed_cmd + ego_acc * dt, self.ego_v_max)))
        self.prev_speed_cmd = ego_speed_cmd

        msg = Float64()
        msg.data = ego_speed_cmd
        self.publisher_.publish(msg)
        self.gps_file.write(f"{now - self.start_time}, {self.ego_speed}, {ego_speed_cmd}, {ego_xy[0]}, {ego_xy[1]}, {ego_acc_unf}, {ego_acc}, {stopping_dist}, {self.curr_speed_wp_tracking_id}, {self.curr_ego_index}\n")
        self.prev_ego_index = self.curr_ego_index
        self.get_logger().info(
            f"u={self.ego_speed:.3f}, v_cmd={ego_speed_cmd:.3f}, swp_d={stopping_dist:.1f}, v_swp={self.wps_des_speeds[self.curr_speed_wp_tracking_id]:.2f}, swp_id={self.curr_speed_wp_tracking_id}, w_id={self.curr_ego_index}"
        )

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
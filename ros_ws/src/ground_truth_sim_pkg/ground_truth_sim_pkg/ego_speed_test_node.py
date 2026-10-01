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
        super().__init__('ego_speed_test_node')

        self.gps_subscriber = self.create_subscription(GPS, 'gps', self.gps_callback, 10)
        self.publisher_ = self.create_publisher(Float64, 'ego_speed_cmd', 10)
        self.prev_heading = {}

        self.origin_lat, self.origin_lon = 42.51635737397318, -83.0434636223155
        self.wps_w_speed = [[-74.19,-3.58], [-153.31,-7.7], [-187.22,-35.1], [-159.34,-59.76], [-102.64, -33.25]]
        self.wps_des_speeds = [1, 3, 0.5, 1, 0]
        self.curr_speed_wp_tracking_id = 0
        self.prev_speed_cmd = 0.0

        self.ego_xy = np.array([0, 0])
        self.ego_speed = 0
        self.ego_heading = 0
        self.ego_v_max, self.ego_a_max = 3, 1
        self.ego_a_min = -2
        self.prev_timestamp = 0

        self.lanes_connects = []
        self.lanes_wps = []
        self.ego_init_lane, self.ego_manv = 5, 'L'
        self.curr_ego_index = 0
        self.prev_ego_index = None
        self.lookahead_time = 8 # sec

        self.offline_pts = []
        self.offline_pts_xy_only = []
        self.load_offline_pts()

        time.sleep(2.0)

        self.timer = self.create_timer(0.1, self.listener_callback)

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        heading = msg.heading
        self.ego_speed = msg.speed

        local_x, local_y, _ = convert_lat_lon_to_xy(self.origin_lat*np.pi/180, self.origin_lon*np.pi/180, 0, latitude*np.pi/180, longitude*np.pi/180)
        self.ego_xy = np.array([local_x, local_y])
        self.ego_heading = -heading + np.pi/2

        # self.load_intersection('/connau/ConnAu/Data/Intersections/intersection_database_gmaps.json')

    def load_offline_pts(self):
        csv_name = 'closed_loop_speed_test_traj.csv'
        try:
            with open(f'/home/connau/ConnAu/Data/offline_path_files/{csv_name}', 'r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    self.offline_pts.append([float(row['x']), float(row['y']), float(row['head_rad'])])
                    self.offline_pts_xy_only.append([float(row['x']), float(row['y'])])
            print(f"Log data successfully read from {csv_name}")
        except IOError as e:
            print(f"Error reading from CSV file: {e}")

    def listener_callback(self):
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
                self.curr_ego_index = min(i + self.curr_ego_index - 1, 0)
                break
        lookahead_dist = max(2, self.ego_speed)*self.lookahead_time
        track_dist_along_path = 0
        prev_offline_wp = ego_xy
        ego_lookahead_wps = []
        for i, offline_wp in enumerate(self.offline_pts[self.curr_ego_index:]):
            offline_wp = np.array([offline_wp[0], offline_wp[1]])
            if i == 0:
                if (offline_wp == ego_xy).all():
                    # temp_dist = np.linalg.norm(prev_offline_wp - [offline_wp[0], offline_wp[1]]) * \
                    #     np.cos(self.ego_heading - np.arctan((offline_wp[1] - prev_offline_wp[1])/(offline_wp[0] - prev_offline_wp[0])))
                    temp_dist = 0
                else:
                    temp_dist = np.linalg.norm(prev_offline_wp - offline_wp) * \
                        np.cos(self.ego_heading - np.arctan((offline_wp[1] - prev_offline_wp[1])/(offline_wp[0] - prev_offline_wp[0])))
            else:
                temp_dist = np.linalg.norm(prev_offline_wp - offline_wp)
            track_dist_along_path += temp_dist
            ego_lookahead_wps.append(offline_wp)
            prev_offline_wp = offline_wp
            # self.get_logger().info(f"{track_dist_along_path}, {lookahead_dist}")
            if track_dist_along_path > lookahead_dist:
                break

        for ego_wp in ego_lookahead_wps:
            if [ego_wp[0], ego_wp[1]] == self.wps_w_speed[self.curr_speed_wp_tracking_id]:
                ego_to_swp_dist = np.linalg.norm(np.array([ego_wp[0], ego_wp[1]]) - ego_xy)
                prev_ego_to_swp_dist = np.linalg.norm([self.offline_pts[self.prev_ego_index][0], 
                                                                self.offline_pts[self.prev_ego_index][1]] - [ego_wp[0], ego_wp[1]])
                curr_ego_to_swp_dist = np.linalg.norm([self.offline_pts[self.curr_ego_index][0], 
                                                                self.offline_pts[self.curr_ego_index][1]] - [ego_wp[0], ego_wp[1]])
                if curr_ego_to_swp_dist - prev_ego_to_swp_dist > 0:
                    self.curr_speed_wp_tracking_id += 1
                elif self.wps_des_speeds[self.curr_speed_wp_tracking_id] == 0 and self.ego_speed < 0.1 and ego_to_swp_dist < 3:
                    self.curr_speed_wp_tracking_id += 1
                self.get_logger().info(f"{ego_to_swp_dist}")
                stopping_dist = ego_to_swp_dist
                ego_acc_unf = self.wps_des_speeds[self.curr_speed_wp_tracking_id]**2 - self.ego_speed**2 / (2 * stopping_dist)
                break
        ego_acc = max(self.ego_a_min, min(self.ego_a_max, ego_acc_unf))
        ego_speed_cmd = float(max(0, min(self.prev_speed_cmd + ego_acc * 0.1, self.ego_v_max)))
        self.prev_speed_cmd = ego_speed_cmd

        msg = Float64()
        msg.data = ego_speed_cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f", {self.ego_speed}, {ego_speed_cmd}, {ego_xy[0]}, {ego_xy[1]}, {ego_acc_unf}, {ego_acc}")
        self.prev_ego_index = self.curr_ego_index

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
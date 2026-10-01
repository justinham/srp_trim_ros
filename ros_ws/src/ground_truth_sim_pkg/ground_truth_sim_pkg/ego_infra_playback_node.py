#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
import json
import time
import math
import csv
from fusion.msg import GPS
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from fusion.msg import Track

def parse_ts_ms_generic(val: str) -> int:
    """
    Robustly parse epoch timestamps provided as:
      - seconds (float or int)    -> convert to ms
      - milliseconds (int)        -> keep as ms
      - microseconds (int)        -> convert to ms
    Returns integer milliseconds.
    """
    s = str(val).strip()
    x = float(s)  # handles ints & floats
    if x >= 1e14:      # microseconds since epoch
        return int(round(x / 1e3))
    elif x >= 1e12:    # milliseconds since epoch
        return int(round(x))
    else:              # seconds (float/int) since epoch
        return int(round(x * 1e3))

def parse_ts_ms_seconds(sec_val):
    """CSV2 timestamps are in seconds (float). Return integer ms."""
    return int(round(float(sec_val) * 1000.0))

def wrap_to_pi(a):
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def circ_exp_smooth(prev_angle, new_angle, alpha):
    """
    Exponential smoothing for angles (radians), wrap-safe.
    alpha in (0,1]: higher = less smoothing (more responsive).
    """
    if prev_angle is None:
        return wrap_to_pi(new_angle)

    # Smooth on unit circle
    px, py = math.cos(prev_angle), math.sin(prev_angle)
    nx, ny = math.cos(new_angle), math.sin(new_angle)

    sx = (1.0 - alpha) * px + alpha * nx
    sy = (1.0 - alpha) * py + alpha * ny

    return math.atan2(sy, sx)

def ms_to_time_msg(ts_ms):
    sec = int(ts_ms // 1000)
    nanosec = int((ts_ms % 1000) * 1_000_000)
    t = Time()
    t.sec = sec
    t.nanosec = nanosec
    return t

class InfraPlaybackNode(Node):
    def __init__(self):
        super().__init__('ego_infra_playback_node')

        # Small startup delay (kept from your original)
        time.sleep(3)

        # ---------------- Loading CSVs ----------------
        self.gps_csv_path = self.declare_parameter('gps_csv_path', '/home/yz4d3h/Documents/infra_process/10_08/VRU/1051_to_1054/sc3/ego_gps_latched.csv').get_parameter_value().string_value
        self.ego_track_csv_path = self.declare_parameter('ego_track_csv_path', '/home/yz4d3h/Documents/vehicle_process/PCAN_logs_10_08_25/VRU_scenarios/sc3/ComparisonFiltered.csv').get_parameter_value().string_value
        self.sdsm_csv_path = self.declare_parameter('sdsm_csv_path', '/home/yz4d3h/Documents/infra_process/10_08/VRU/1051_to_1054/sc3/filtered_derq_gt.csv').get_parameter_value().string_value

        # ---------------- Publishers ----------------
        self.sdsm_pub = self.create_publisher(String, 'infra_local_measures', 10)
        self.gt_pub     = self.create_publisher(String, 'ground_truth_topic', 10)
        self.ego_track_pub   = self.create_publisher(Track, 'track', 10)
        self.gps_pub    = self.create_publisher(GPS, 'gps', 10)
        self.declare_parameter('source', 'fused') 
        self.source = self.get_parameter('source').value

        df = pd.read_csv(self.sdsm_csv_path)
        # Your timestamps are ms (int) in the infra file
        df['timestamp'] = df['timestamp'].astype(int)
        df = df.sort_values('timestamp')
        self.infra_timestamps = sorted(df['timestamp'].unique())
        self.ego_track_csv = self._load_csv(self.ego_track_csv_path)
        # Group sensor entries by timestamp (as before)
        self.grouped_entries = {ts: df[df['timestamp'] == ts].copy() for ts in self.infra_timestamps}
        self.get_logger().info(f"Loaded grouped data for {len(self.infra_timestamps)} infra timestamps.")

        # ---------------- Infra heading filter (MA)
        self.heading_alpha = self.declare_parameter('heading_alpha', 0.15).get_parameter_value().double_value
        self.min_speed_for_heading = self.declare_parameter('min_speed_for_heading', 1.0).get_parameter_value().double_value  # m/s

        self.filtered_heading = None
        self.have_filtered_heading = False
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.is_clock_published = False

        # ---------------- Load GPS CSV (seconds) ----------------
        self.gps_rows = []
        if self.gps_csv_path:
            try:
                gdf = pd.read_csv(self.gps_csv_path)
                # Expecting a header like:
                # timestamp (s) {PC Timestamp},lat (deg),lon (deg),heading (rad),calculated speed (km/h),
                # year - gps,day of year - gps,time of day in ms - gps UTC, gps valid, heading valid
                # Normalize column names for robust access
                gdf.columns = [c.strip() for c in gdf.columns]
                # Best-effort name for timestamp column (first col)
                ts_col = gdf.columns[0]
                gdf = gdf.sort_values(ts_col)

                for _, r in gdf.iterrows():
                    try:
                        ts_ms = parse_ts_ms_seconds(r[ts_col])
                        lat = float(r[gdf.columns[1]])
                        lon = float(r[gdf.columns[2]])
                        heading_rad = float(r[gdf.columns[3]])
                        speed_kmh = float(r[gdf.columns[4]])
                        gps_valid = str(r[gdf.columns[8]]).strip().lower() == 'true'
                        heading_valid = str(r[gdf.columns[9]]).strip().lower() == 'true'
                        self.gps_rows.append({
                            'ts_ms': ts_ms,
                            'lat': lat,
                            'lon': lon,
                            'heading_rad': heading_rad,
                            'speed_kmh': speed_kmh,
                            'gps_valid': gps_valid,
                            'heading_valid': heading_valid
                        })
                    except Exception as e:
                        self.get_logger().warn(f"GPS row parse error, skipping: {e}")
                self.get_logger().info(f"Loaded GPS rows: {len(self.gps_rows)}")
            except Exception as e:
                self.get_logger().error(f"Failed to load gps_csv_path='{self.gps_csv_path}': {e}")
                self.gps_rows = []
        else:
            self.get_logger().warn("No 'gps_csv_path' provided; GPS synchronization disabled.")

        # ---------------- Global clock setup ----------------
        # Earliest timestamp across both streams (ms)
        infra_start_ms = self.infra_timestamps[0]
        gps_start_ms = self.gps_rows[0]['ts_ms'] if self.gps_rows else float('inf')
        ego_track_start_ms = parse_ts_ms_generic(self.ego_track_csv[0][0])
        self.global_start_ms = min(infra_start_ms, gps_start_ms, ego_track_start_ms)
        # self.global_start_ms = infra_start_ms

        self.system_start_time_ms = int(time.time() * 1000)

        # Playback indices
        self.infra_idx = 0                  # index into self.infra_timestamps
        self.gps_idx = 0                    # index into self.gps_rows
        self.ego_track_idx = 0

        # Reusable GT message envelope (kept from original)
        self.gt_json_data = {
            "timestamp": None,
            "ego_vehicle": None,
            "remote_actors": None
        }

        # High-rate timer; publishes any due items each tick
        self.timer = self.create_timer(0.01, self._tick)

    def _publish_clock(self, ts_ms):
        clk = Clock()
        clk.clock = ms_to_time_msg(ts_ms)
        self.clock_pub.publish(clk)

    # -------------- Timer tick: publish due entries --------------
    def _tick(self):
        now_ms = int(time.time() * 1000)
        actual_elapsed = now_ms - self.system_start_time_ms
        replay_now_ms = self.global_start_ms + actual_elapsed
        self._publish_clock(replay_now_ms)
        # self.get_logger().info(f"actual_elapsed: {replay_now_ms}")

        made_progress = False

        # Publish all infra groups whose (ts - global_start) <= actual_elapsed
        if self.source == 'ego':
            pass
        else:
            while self.infra_idx < len(self.infra_timestamps):
                ts = self.infra_timestamps[self.infra_idx]
                due_elapsed = ts - self.global_start_ms
                if actual_elapsed >= due_elapsed:
                    self._publish_infra_at(ts)
                    self.infra_idx += 1
                    made_progress = True
                else:
                    break

        # Publish GPS rows that are due
        while self.gps_idx < len(self.gps_rows):
            ts_ms = self.gps_rows[self.gps_idx]['ts_ms']
            due_elapsed = ts_ms - self.global_start_ms
            if actual_elapsed >= due_elapsed:
                self._publish_gps_row(self.gps_rows[self.gps_idx])
                self.gps_idx += 1
                made_progress = True
            else:
                break
        
        if self.source == 'infra':
            pass
        else:
            while self.ego_track_idx < len(self.ego_track_csv):
                ts_ms = int(self.ego_track_csv[self.ego_track_idx][0])
                due_elapsed = ts_ms - self.global_start_ms
                if actual_elapsed >= due_elapsed:
                    self.to_fusion(self.ego_track_csv[self.ego_track_idx])
                    self.ego_track_idx += 1
                    made_progress = True
                else:
                    break

        # Stop when both streams are finished
        if self.infra_idx >= len(self.infra_timestamps) and self.gps_idx >= len(self.gps_rows):
            self.get_logger().info("Finished playback (infra + gps).")
            self.timer.cancel()
            return

        # (Optional) prevent busy-loop if nothing was ready
        if not made_progress:
            pass  # timer will trigger again in 10ms

    # -------------- Publish helpers --------------
    def _publish_infra_at(self, ts):
        detections = self.grouped_entries[ts]
        refLat = detections.iloc[0]['refLat']
        refLon = detections.iloc[0]['refLon']

        # Sensor (SDSM-like) message
        sdsmData = []
        for _, row in detections.iterrows():
            sdsmData.append({
                "class": row['class'],
                "uniqueId": row['uniqueId'],
                "pos": {
                    "offsetX": row['offsetX'] * 10,   # kept from your original
                    "offsetY": row['offsetY'] * 10,
                },
                "posStdDev": row['posStdDev'],
                "speed": row['speed'],
                "speedStdDev": row['speedStdDev'],
                "heading": row['heading'],
                "headingStdDev": row['headingStdDev']
            })

        sensor_msg = {
            "timestamp": str(ts),
            "refPos": {"lat": str(refLat), "long": str(refLon)},
            "sdsmData": sdsmData
        }
        msg1 = String()
        msg1.data = json.dumps(sensor_msg)
        self.sdsm_pub.publish(msg1)

        # Ground truth (first row’s GT fields, same as your original)
        row = detections.iloc[0]
        gt_msg = {
            "uniqueId": "rv_gt",
            "carlaId": None,
            "class": "vehicle",
            "pos": {
                "lat": row['gt_offsetY'],
                "lon": row['gt_offsetX'],
                "posStdDev_lat": "0.0",
                "posStdDev_lon": "0.0"
            },
            "vel": {
                "lat_vel": row['gt_SpeedMps'],
                "lon_vel": "0.0",
                "velStdDev_lat": "0.0",
                "velStdDev_lon": "0.0"
            },
            "heading": {
                "theta": row['gt_HeadingDeg'],
                "theta_valid": True,
                "thetaStdDev": "0.0"
            },
            "halfDim": None
        }
        self.gt_json_data["timestamp"] = str(ts/1000)
        self.gt_json_data["remote_actors"] = [gt_msg]

        msg2 = String()
        msg2.data = json.dumps(self.gt_json_data)
        self.gt_pub.publish(msg2)

        self.get_logger().info(f"Published INFRA at ts={ts}")

    def _publish_gps_row(self, r):
        lat = float(r['lat'])
        lon = float(r['lon'])
        heading_rad = float(r['heading_rad'])
        speed = float(r['speed_kmh']) * 0.514  # (your existing conversion)
        gps_valid = bool(r['gps_valid'])
        heading_valid = bool(r['heading_valid'])

        # Only update heading filter when heading is valid and speed is meaningful
        if heading_valid and gps_valid and speed >= self.min_speed_for_heading:
            self.filtered_heading = circ_exp_smooth(self.filtered_heading, heading_rad, self.heading_alpha)
            self.have_filtered_heading = True

        # If we never got a good heading yet, fall back to raw heading (or 0.0)
        heading_to_pub = self.filtered_heading if self.have_filtered_heading else heading_rad

        g = GPS()
        g.timestamp = ms_to_time_msg(float(r['ts_ms']))
        g.latitude = lat
        g.longitude = lon
        g.heading = heading_to_pub
        g.speed = speed
        self.gps_pub.publish(g)

    def to_fusion(self, line):
        (timestamp, sensor_id, track_id, track_status, confidence, object_type,
         obs_x_off, obs_y_off, obs_v_x, obs_v_y, obs_theta, obs_theta_valid,
         obs_theta_cov, obs_x_cov, obs_y_cov, obs_v_x_cov, obs_v_y_cov,
         int_x_off, int_y_off, int_v_x, int_v_y, lat_err, lon_err, v_lat_err,
         v_lon_err, int_heading, heading_err, ego_lat, ego_lon, ego_heading,
         remote_lat, remote_lon, remote_heading) = line

        if int(confidence) < 2:
            return

        tmsg = Track()
        tmsg.timestamp = ms_to_time_msg(int(timestamp))
        tmsg.sensor_id = int(sensor_id)
        tmsg.track_id = int(track_id)
        tmsg.track_status = int(track_status)
        tmsg.confidence = int(confidence)
        tmsg.object_type = (object_type == "True")
        tmsg.lat = float(obs_x_off)
        tmsg.lon = float(obs_y_off)
        tmsg.lat_vel = float(obs_v_x)
        tmsg.lon_vel = float(obs_v_y)

        if obs_theta_valid == "True":
            tmsg.theta = float(obs_theta)
            tmsg.theta_valid = True
            tmsg.theta_cov = float(obs_theta_cov)
        else:
            tmsg.theta = 0.0
            tmsg.theta_valid = False
            tmsg.theta_cov = 100.0

        tmsg.covariance[0]  = float(obs_x_cov)
        tmsg.covariance[5]  = float(obs_y_cov)
        tmsg.covariance[10] = float(obs_v_x_cov)
        tmsg.covariance[15] = float(obs_v_y_cov)

        # Ego GPS from CSV1 (if present)
        # g = GPS()
        # g.timestamp = tmsg.timestamp
        # g.latitude = float(ego_lat)
        # g.longitude = float(ego_lon)
        # g.heading = float(ego_heading)

        # Remote/GT GPS from CSV1
        gt_gps = GPS()
        gt_gps.timestamp = tmsg.timestamp
        gt_gps.latitude = float(remote_lat)
        gt_gps.longitude = float(remote_lon)
        gt_gps.heading = float(remote_heading)

        self.ego_track_pub.publish(tmsg)
        self.get_logger().info(f"Published EGO at ts={timestamp}")

    def _load_csv(self, path):
        rows = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            _ = next(reader, None)  # header
            for l in reader:
                if l:
                    rows.append([x.strip() for x in l])
        return rows
    
def main(args=None):
    rclpy.init(args=args)
    node = InfraPlaybackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
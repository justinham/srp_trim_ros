#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
import json
import time
import math
from fusion.msg import GPS

def parse_ts_ms_seconds(sec_val):
    """CSV2 timestamps are in seconds (float). Return integer ms."""
    return int(round(float(sec_val) * 1000.0))


class InfraPlaybackNode(Node):
    def __init__(self):
        super().__init__('infra_playback_node2')

        # Small startup delay (kept from your original)
        time.sleep(3)

        # ---------------- Params ----------------
        self.sc_num = self.declare_parameter('sc_num', 0).get_parameter_value().integer_value
        self.gps_csv_path = self.declare_parameter('gps_csv_path', '/home/connau/Documents/infra_process/10_08/VRU/1051_to_1054/ego_gps_latched.csv').get_parameter_value().string_value

        # ---------------- Publishers ----------------
        self.sensor_pub = self.create_publisher(String, 'infra_local_measures', 10)
        self.gt_pub     = self.create_publisher(String, 'ground_truth_topic', 10)
        self.publisher_gps    = self.create_publisher(GPS, 'gps', 10)

        # ---------------- Load primary (infra) CSV ----------------
        if self.sc_num == 0:
            csv_file = '/home/connau/Documents/infra_process/10_08/VRU/1051_to_1054/derq_gt.csv'
            self.get_logger().info("going w pedestrian")
        else:
            csv_file = f'/home/connau/Documents/infra_process/08_12/hev_stationary/sc{self.sc_num}/filtered_derq_gt_all.csv'

        df = pd.read_csv(csv_file)
        # Your timestamps are ms (int) in the infra file
        df['timestamp'] = df['timestamp'].astype(int)
        df = df.sort_values('timestamp')
        self.infra_timestamps = sorted(df['timestamp'].unique())

        # Group sensor entries by timestamp (as before)
        self.grouped_entries = {ts: df[df['timestamp'] == ts].copy() for ts in self.infra_timestamps}
        self.get_logger().info(f"Loaded grouped data for {len(self.infra_timestamps)} infra timestamps.")


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
        self.global_start_ms = min(infra_start_ms, gps_start_ms)

        self.system_start_time_ms = int(time.time() * 1000)

        # Playback indices
        self.infra_idx = 0                  # index into self.infra_timestamps
        self.gps_idx = 0                    # index into self.gps_rows

        # Reusable GT message envelope (kept from original)
        self.gt_json_data = {
            "timestamp": None,
            "ego_vehicle": None,
            "remote_actors": None
        }

        # High-rate timer; publishes any due items each tick
        self.timer = self.create_timer(0.01, self._tick)

    # -------------- Timer tick: publish due entries --------------
    def _tick(self):
        now_ms = int(time.time() * 1000)
        actual_elapsed = now_ms - self.system_start_time_ms


        made_progress = False

        # Publish all infra groups whose (ts - global_start) <= actual_elapsed
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
        self.sensor_pub.publish(msg1)

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

        self.gt_json_data["timestamp"] = str(ts)
        self.gt_json_data["remote_actors"] = [gt_msg]

        msg2 = String()
        msg2.data = json.dumps(self.gt_json_data)
        self.gt_pub.publish(msg2)

        self.get_logger().info(f"Published infra at ts={ts}")

    def _publish_gps_row(self, r):
        lat = float(r['lat'])
        lon = float(r['lon'])
        heading_rad = float(r['heading_rad'])

        speed = float(r['speed_kmh'])*0.514
        gps_valid_str = r['gps_valid']
        heading_valid_str = r['heading_valid']

        g = GPS()
        g.timestamp = self.get_clock().now().to_msg()
        g.latitude = lat
        g.longitude = lon

        # If your GPS msg expects degrees, convert; otherwise keep radians.
        # Here we assume 'heading' is in degrees in your GPS type; convert:
        g.heading = heading_rad
        g.speed = speed
        self.publisher_gps.publish(g)
def main(args=None):
    rclpy.init(args=args)
    node = InfraPlaybackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

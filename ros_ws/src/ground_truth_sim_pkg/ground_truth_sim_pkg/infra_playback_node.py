import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
import json
import time

class InfraPlaybackNode(Node):
    def __init__(self):
        super().__init__('infra_playback_node')

        time.sleep(3)

        self.sc_num = self.declare_parameter('sc_num', 0).get_parameter_value().integer_value

        self.sensor_pub = self.create_publisher(String, 'infra_local_measures', 10)
        self.gt_pub = self.create_publisher(String, 'ground_truth_topic', 10)

        # Load CSV and group by timestamp
        if self.sc_num == 0:
            csv_file = '/home/connau/Documents/infra_process/10_08/VRU/1005_to_1009/filtered_derq_gt3.csv'
            self.get_logger().info("going w pedestrian")
        else:
            csv_file = '/home/connau/Documents/infra_process/08_12/hev_stationary/sc' + str(self.sc_num) + '/filtered_derq_gt_all.csv'

        df = pd.read_csv(csv_file)
        df['timestamp'] = df['timestamp'].astype(int)
        df = df.sort_values('timestamp')
        self.timestamps = sorted(df['timestamp'].unique())

        # Group sensor entries by timestamp
        self.grouped_entries = {
            ts: df[df['timestamp'] == ts].copy() for ts in self.timestamps
        }

        self.get_logger().info(f"Loaded grouped data for {len(self.timestamps)} timestamps.")

        self.playback_start_time = self.timestamps[0]
        self.system_start_time = int(time.time() * 1000)
        self.index = 0
        # 1759932722862 1759932740376 1759932824758 1759932837568
        self.gt_json_data = {
            "timestamp": None,

            "ego_vehicle": None,
            "remote_actors": None
        }

        self.timer = self.create_timer(0.01, self.publish_next_entry)

    def publish_next_entry(self):
        if self.index >= len(self.timestamps):
            self.get_logger().info("Finished playback.")
            self.timer.cancel()
            return

        ts = self.timestamps[self.index]
        now = int(time.time() * 1000)
        expected_elapsed = ts - self.playback_start_time
        actual_elapsed = now - self.system_start_time

        if actual_elapsed >= expected_elapsed:
            detections = self.grouped_entries[ts]
            refLat = detections.iloc[0]['refLat']
            refLon = detections.iloc[0]['refLon']

            # Create sensor message
            sdsmData = []
            for _, row in detections.iterrows():
                sdsmData.append({
                    "class": row['class'],
                    "uniqueId": row['uniqueId'],
                    "pos": {
                        "offsetX": row['offsetX']*10,
                        "offsetY": row['offsetY']*10,
                    },
                    "posStdDev": row['posStdDev'],
                    "speed": row['speed'],
                    "speedStdDev": row['speedStdDev'],
                    "heading": row['heading'],
                    "headingStdDev": row['headingStdDev']
                })

            sensor_msg = {
                "timestamp": str(ts),
                "refPos": {
                    "lat": str(refLat),
                    "long": str(refLon)
                },
                "sdsmData": sdsmData
            }

            msg1 = String()
            msg1.data = json.dumps(sensor_msg)
            self.sensor_pub.publish(msg1)

            # Create ground truth message (use first row's GT fields)
            row = detections.iloc[0]
            gt_msg = {
                "uniqueId": "rv_gt",
                "carlaId": None,
                "class": "rv",
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
            # if self.sc_num == 0:
            #     gt_msg['class'] = "vru"

            self.gt_json_data["remote_actors"] = []
            self.gt_json_data["remote_actors"].append(gt_msg)

            msg2 = String()
            msg2.data = json.dumps(self.gt_json_data)
            self.gt_pub.publish(msg2)

            self.get_logger().info(f"Published all data for timestamp: {ts}")
            self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = InfraPlaybackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
import json
import time  

class InfraPlaybackNode(Node):
    def __init__(self):
        super().__init__('infra_test_sim_node')

        time.sleep(3)

        self.sc_num = self.declare_parameter('sc_num', 0).get_parameter_value().integer_value

        self.sensor_pub = self.create_publisher(String, 'infra_local_measures', 10)

        self.timer = self.create_timer(0.1, self.publish_next_entry)

        self.vru_start_loc = [9.37, 9.64]
        self.vru_heading = 180
        self.vru_speed = 1.0

        self.start_time, self.end_time = 0, 60
        self.start_timestamp = time.monotonic()

    def publish_next_entry(self):
        if self.start_time > time.monotonic()-self.start_timestamp:
            self.get_logger().info("Yet to start playback")
            return
        if self.end_time < time.monotonic()-self.start_timestamp:
            self.get_logger().info("Finished playback.")
            self.timer.cancel()
            return

        refLat, refLon = "42.5150354710", "-83.0439986570"

        self.vru_start_loc[0] += -self.vru_speed*0.1

        # Create sensor message with one vru
        sdsmData = []
        sdsmData.append({
            "class": "vru",
            "uniqueId": "1",
            "pos": {
                "offsetX": self.vru_start_loc[0]*10,
                "offsetY": self.vru_start_loc[1]*10,
            },
            "posStdDev": "0.1",
            "speed": str(self.vru_speed),
            "speedStdDev": "0.1",
            "heading": str(self.vru_heading),
            "headingStdDev": "1"
            })

        sensor_msg = {
            "timestamp": str(int(time.monotonic()*1000)),
            "refPos": {
                "lat": str(refLat),
                "long": str(refLon)
            },
            "sdsmData": sdsmData
        }

        msg1 = String()
        msg1.data = json.dumps(sensor_msg)
        self.sensor_pub.publish(msg1)

        self.get_logger().info(f"{time.monotonic()}")

def main(args=None):
    rclpy.init(args=args)
    node = InfraPlaybackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

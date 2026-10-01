import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class RawInfraPlaybackNode(Node):
    def __init__(self):
        super().__init__('raw_infra_playback_node')
        self.publisher_ = self.create_publisher(String, 'infra_local_measures', 10)
        self.inter_basic_sub = self.create_subscription(String, "closest_intersection_basic", self.inter_basic_callback, 10)

        # Load JSONL file

        jsonl_file = '/home/connau/Documents/infra_process/09_03/Pedestrian/pedestrian.jsonl'

        with open(jsonl_file, 'r') as f:
            self.entries = [json.loads(line) for line in f]

        # self.entries.sort(key=lambda x: int(x['timestamp']))

        self.get_logger().info(f"Loaded {len(self.entries)} SDSM entries.")

        self.playback_start_time = int(self.entries[0]['timestamp'])
        self.system_start_time = int(time.time() * 1000)  # ms since epoch

        # Start playback loop
        self.timer = self.create_timer(0.01, self.publish_next_entry)
        self.ref_pos = {"x": 0.0, "y": 0.0}

        self.index = 0

    def inter_basic_callback(self, msg):
        data = json.loads(msg.data)
        self.ref_pos = data['pos']

    def publish_next_entry(self):
        if self.index >= len(self.entries):
            self.get_logger().info("Finished playback.")
            self.timer.cancel()
            return

        entry = self.entries[self.index]
        entry_time = int(entry['timestamp'])
        if self.ref_pos is None:
            self.get_logger().info("still None")
            return
        entry['refPos'] = self.ref_pos

        now = int(time.time() * 1000)
        expected_elapsed = entry_time - self.playback_start_time
        actual_elapsed = now - self.system_start_time

        if actual_elapsed >= expected_elapsed:
            msg = String()
            msg.data = json.dumps(entry)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published entry at {entry_time}")
            self.index += 1  # move to next

def main(args=None):
    rclpy.init(args=args)
    node = RawInfraPlaybackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
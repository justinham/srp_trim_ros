#!/usr/bin/env python3
import json
import signal
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AppResponderNode(Node):

    def __init__(self):
        super().__init__("app_responder_node")
        self.proc = None
        self.sub = self.create_subscription(String, "/app_cmd", self.on_toggle, 10)

    def start_autonomy(self):
        if self.proc is not None:
            self.get_logger().info("Autonomy already running")
            return
        self.get_logger().info("Starting autonomy node...")
        self.proc = subprocess.Popen(["ros2", "run", "fusion_py", "VehicleCommander"])

    def stop_autonomy(self):
        if self.proc is None:
            self.get_logger().info("Autonomy already stopped")
            return
        self.get_logger().info("Stopping autonomy node...")
        self.proc.send_signal(signal.SIGINT)
        self.proc.wait()
        self.proc = None

    def on_toggle(self, msg: String):
        try:
            d = json.loads(msg.data)
            if d.get("type") != "toggle":
                return
            if d.get("name") != "enable_autonomy":
                return
            enabled = bool(d.get("value"))
            if enabled:
                self.start_autonomy()
            else:
                self.stop_autonomy()
        except Exception as e:
            self.get_logger().error(str(e))

def main():
    rclpy.init()
    node = AppResponderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_autonomy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
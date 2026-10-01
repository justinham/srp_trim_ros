import os, socket, time, selectors
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pycrate_sdsm.SDSMDecoder2 import sdsm_decoder
import json

def extract_from_0029(b: bytes) -> str | None:
    hx = b.hex()
    i = hx.find("0029")
    return hx[i:] if i != -1 else None

def extract_from_0029_nd_psid(pkt: bytes) -> str | None:
    hx = pkt.hex()
    i = hx.find("0029") 
    j = hx.find("008010")
    if i != -1 and j != -1:
        return hx[i:] if i > j else None
    else:
        return None

class OBUInterfaceNode(Node):
    def __init__(self):
        super().__init__('obu_interface_node2')
        bind_ip  = os.getenv('SDSM_BIND_IP', '192.168.31.10')
        bind_port= int(os.getenv('SDSM_PORT', '9010'))
        self.pub = self.create_publisher(String, '/infra_local_measures', 10)

        # setting up udp socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
        self.sock.bind((bind_ip, bind_port))
        self.sock.setblocking(False)

        file_path = "log_sdsm"
        self.gps_file = open(file_path + "_ublox_gps.csv", 'w+')
        self.gps_file.write("timestamp (s), sdsm_hex\n")

        self.sel = selectors.DefaultSelector()
        self.sel.register(self.sock, selectors.EVENT_READ)
        self.timer = self.create_timer(0.0, self._poll)  # fires whenever executor can

        self.get_logger().info(f"Listening on UDP {bind_ip}:{bind_port}")

    def _poll(self):
        for _key, _mask in self.sel.select(timeout=0):
            try:
                data, addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                continue
            sdsm_hex = extract_from_0029(data)
            if sdsm_hex:
                ts = time.time()
                self.gps_file.write(f"{ts},{sdsm_hex}\n")
                self.get_logger().info(f"{ts:.6f} {sdsm_hex}")
                decoded = sdsm_decoder(sdsm_hex)
                msg = String()
                msg.data = json.dumps(decoded)
                self.pub.publish(msg)
                self.get_logger().info(f"heres decoded: {msg.data}")

def main():
    rclpy.init()
    node = OBUInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

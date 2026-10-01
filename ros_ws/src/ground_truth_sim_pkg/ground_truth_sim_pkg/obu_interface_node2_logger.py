import os, socket, time, selectors
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from pycrate_sdsm.SDSMDecoder2 import sdsm_decoder
import json
from fusion.msg import GPS

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
        super().__init__('obu_interface_node2_logger')
        bind_ip  = os.getenv('SDSM_BIND_IP', '192.168.31.10')
        bind_port= int(os.getenv('SDSM_PORT', '9010'))
        self.gps_subscriber = self.create_subscription(GPS, 'gps', self.gps_callback, 10)
        self.pub = self.create_publisher(String, '/infra_local_measures', 10)

        # setting up udp socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
        self.sock.bind((bind_ip, bind_port))
        self.sock.setblocking(False)

        self.obu_listener_log_file = open("log-obu-listener.csv", 'w+')
        self.obu_listener_log_file.write("sys_timestamp,infra_timestamp,ego_speed,ego_northing_pos,ego_easting_pos,num_objects\n")
        self.ego_xy = np.array([0,0])
        self.ego_speed = 0
        self.ego_heading = 0
        self.origin_lat, self.origin_lon = 42.5150354710, -83.0439986570 # RAB

        self.sel = selectors.DefaultSelector()
        self.sel.register(self.sock, selectors.EVENT_READ)
        self.timer = self.create_timer(0.0, self._poll)  # fires whenever executor can

        self.get_logger().info(f"Listening on UDP {bind_ip}:{bind_port}")

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        heading = msg.heading
        self.ego_speed = msg.speed

        local_x, local_y, _ = convert_lat_lon_to_xy(self.origin_lat*np.pi/180, self.origin_lon*np.pi/180, 0, latitude*np.pi/180, longitude*np.pi/180)
        self.ego_xy = np.array([local_y, local_x])
        self.ego_heading = -heading + np.pi/2

    def _poll(self):
        for _key, _mask in self.sel.select(timeout=0):
            try:
                data, addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                continue
            sdsm_hex = extract_from_0029_nd_psid(data)
            if sdsm_hex:
                ts = time.time()
                self.get_logger().info(f"{ts:.6f} {sdsm_hex}")
                decoded = sdsm_decoder(sdsm_hex)
                self.obu_listener_log_file.write(f"{ts},{decoded['timestamp']},{self.ego_speed},{self.ego_xy[0]},{self.ego_xy[1]},{len(decoded['sdsmData'])}\n")
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
#!/usr/bin/env python3
import math, time, csv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from fusion.msg import GPS  # adjust if your msg name/namespace differs

# ---------- same geo helper you use ----------
def convert_lat_lon_to_xy(ref_lat_rad, ref_lon_rad, ref_heading_rad, lat_rad, lon_rad):
    f = 0.003353
    a = 6378137.0
    f1 = math.sqrt(f*(2-f))
    f2 = a*(1.0 - f1**2) / ((1.0 - (f1**2)*(math.sin(ref_lat_rad)**2))**(3.0/2.0))
    f3 = a / math.sqrt(1.0 - (f1**2)*(math.sin(ref_lat_rad)**2))
    E = f3*math.cos(ref_lat_rad) * (lon_rad - ref_lon_rad)
    N = f2 * (lat_rad - ref_lat_rad)
    x_m = N*math.cos(ref_heading_rad) + E*math.sin(ref_heading_rad)
    y_m = -N*math.sin(ref_heading_rad) + E*math.cos(ref_heading_rad)
    d2d_m = math.hypot(x_m, y_m)
    return [x_m, y_m, d2d_m]

def xy_to_lat_lon(ref_lat_deg, ref_lon_deg, x_east_m, y_north_m):
    """
    Inverse of the above when ref_heading_rad == 0 (your case).
    Local frame uses x=East, y=North.

    """
    ref_lat = math.radians(ref_lat_deg)
    ref_lon = math.radians(ref_lon_deg)
    f = 0.003353
    a = 6378137.0
    f1 = math.sqrt(f*(2-f))
    f2 = a*(1.0 - f1**2) / ((1.0 - (f1**2)*(math.sin(ref_lat)**2))**(3.0/2.0))
    f3 = a / math.sqrt(1.0 - (f1**2)*(math.sin(ref_lat)**2))
    # E = x_east, N = y_north
    lat = ref_lat + (y_north_m / f2)
    lon = ref_lon + (x_east_m / (f3*math.cos(ref_lat)))
    return math.degrees(lat), math.degrees(lon)

class GPSPathSimNode(Node):
    def __init__(self):
        super().__init__('gps_path_sim')


        # --------- config (match your controller) ---------
        self.origin_lat = 42.520156105740654
        self.origin_lon = -83.04382593113446
        self.path_csv   = '/home/connau/ConnAu/Data/offline_path_files/closed_track_speed_test_traj.csv'

        # dynamics: first-order speed response to ego_speed_cmd
        self.tau_speed = 0.6       # s, inner loop time constant
        self.a_sat     = 3.0       # m/s^2, accel clamp (sim realism)
        self.v_sat     = 6.0       # m/s, absolute max simulated speed


        # lateral “not perfect tracking”
        self.lateral_amp = 0.5     # m, wander amplitude
        self.lateral_wavelen = 60  # m, spatial wavelength for wander

        # measurement noise (applied in meters then converted to lat/lon)
        self.pos_noise_std = 0.15  # m (0 => off)
        self.speed_noise_std = 0.05 # m/s (0 => off)

        # timing
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.step)

        # I/O
        self.pub_gps = self.create_publisher(GPS, 'gps', 20)
        self.pub_gps_can = self.create_publisher(GPS, 'gps_can', 20)

        self.sub_cmd = self.create_subscription(Float64, 'ego_speed_cmd', self.cmd_cb, 20)

        # load path
        self.path_xy = self.load_path(self.path_csv)  # array Nx2 [x_east, y_north]
        self.precompute_geom()

        # sim state
        self.s = 0.0                # along-path distance (m)
        self.v = 0.0                # m/s
        self.cmd_v = 0.0            # m/s (latest from your node)
        self.seg_idx = 0            # current segment for fast lookup
        self.rng = np.random.default_rng(7)

        self.get_logger().info(f"Loaded path with {len(self.path_xy)} points; total length ~{self.cum_s[-1]:.1f} m")

    # -------------- path utilities --------------
    def load_path(self, csv_path):
        pts = []
        with open(csv_path, 'r', newline='') as f:
            r = csv.DictReader(f)
            for row in r:
                pts.append([float(row['x']), float(row['y'])])
        if len(pts) < 2:
            raise RuntimeError("Path CSV must have at least 2 points (x,y).")
        return np.array(pts, dtype=float)

    def precompute_geom(self):
        P = self.path_xy
        V = P[1:] - P[:-1]
        L = np.linalg.norm(V, axis=1)
        L[L < 1e-9] = 1e-9
        self.seg_dir = V / L[:, None]
        self.seg_len = L
        self.cum_s = np.zeros(len(P))
        self.cum_s[1:] = np.cumsum(L)

    def sample_at_s(self, s):
        """Return (x, y, t_hat, seg_idx) for along-path distance s (clamped at end)."""
        s = max(0.0, min(s, self.cum_s[-1]))
        # keeping search local
        i = self.seg_idx
        if s < self.cum_s[i] or s > self.cum_s[i+1]:
            # binary search for segment
            i = np.searchsorted(self.cum_s, s, side='right') - 1
            i = int(np.clip(i, 0, len(self.seg_len)-1))
        self.seg_idx = i
        t = (s - self.cum_s[i]) / self.seg_len[i]
        p = self.path_xy[i] + t*(self.path_xy[i+1] - self.path_xy[i])
        t_hat = self.seg_dir[i]
        return p[0], p[1], t_hat, i

    # -------------- ROS callbacks --------------
    def cmd_cb(self, msg: Float64):
        self.cmd_v = float(msg.data)

    # -------------- main sim step --------------
    def step(self):

        # 1) speed inner-loop: first-order, accel-limited
        e = self.cmd_v - self.v
        dv = (e / max(1e-3, self.tau_speed)) * self.dt
        dv = float(np.clip(dv, -self.a_sat*self.dt, self.a_sat*self.dt))
        self.v = float(np.clip(self.v + dv, 0.0, self.v_sat))

        # 2) advance along path
        self.s = min(self.s + self.v*self.dt, self.cum_s[-1])
        x, y, t_hat, _ = self.sample_at_s(self.s)

        # 3) add lateral wander (almost on the path)
        if self.lateral_amp > 0.0:
            n_hat = np.array([-t_hat[1], t_hat[0]])
            phase = 2.0*math.pi*self.s/max(1.0, self.lateral_wavelen)
            x += n_hat[0]*self.lateral_amp*math.sin(phase)
            y += n_hat[1]*self.lateral_amp*math.sin(phase)

        # 4) add measurement noise in meters
        if self.pos_noise_std > 0.0:
            x += float(self.rng.normal(0.0, self.pos_noise_std))
            y += float(self.rng.normal(0.0, self.pos_noise_std))
        v_meas = self.v + (float(self.rng.normal(0.0, self.speed_noise_std)) if self.speed_noise_std > 0 else 0.0)
        v_meas = max(0.0, v_meas)

        # 5) convert to geodetic and publish GPS
        lat, lon = xy_to_lat_lon(self.origin_lat, self.origin_lon, x_east_m=x, y_north_m=y)
        # heading_f: true course rad (0 = North, +CW) -> atan2(East, North)
        heading_true_rad = math.atan2(t_hat[0], t_hat[1])
        self.publish_gps(lat, lon, heading_true_rad, v_meas)

        # loop the path once we reach the end (optional)
        if self.s >= self.cum_s[-1] - 1e-3:
            self.s = 0.0
            self.v = 0.0
            self.seg_idx = 0

    def publish_gps(self, lat_deg, lon_deg, heading_true_rad, speed_ms):
        msg = GPS()
        # If your message uses different fields, adjust here:
        msg.latitude  = float(lat_deg)
        msg.longitude = float(lon_deg)
        msg.heading_f = float(heading_true_rad)  # rad, 0=N, +CW (RMC-style)

        msg.speed   = float(speed_ms)          # m/s

        msg.speed_f   = float(speed_ms)          # m/s
        # optional flags if your msg has them:
        if hasattr(msg, "valid"): msg.valid = True
        if hasattr(msg, "heading_valid"): msg.heading_valid = True
        self.pub_gps.publish(msg)

        msg2 = GPS()
        msg2.latitude  = float(lat_deg)
        msg2.longitude = float(lon_deg)
        msg2.heading_f = float(heading_true_rad)  # rad, 0=N, +CW (RMC-style)
        msg2.speed   = float(speed_ms)          # m/s
        msg2.speed_f   = float(speed_ms)          # m/s
        if hasattr(msg2, "valid"): msg2.valid = True
        if hasattr(msg2, "heading_valid"): msg2.heading_valid = True
        self.pub_gps_can.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = GPSPathSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

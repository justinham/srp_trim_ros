#!/usr/bin/env python3

import csv
import json
from dataclasses import dataclass
from typing import List, Dict, Any
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pycrate_sdsm.SDSMDecoder2 import sdsm_decoder
import numpy as np
import math
from fusion.msg import GPS

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

@dataclass
class SdsmRecord:
    timestamp: float
    sdsm_hex: str


@dataclass
class PoseRecord:
    timestamp: float
    lat_ms_arc: float
    lon_ms_arc: float
    heading_rad: float
    speed_kmh: float
    year: int
    day_of_year: int
    time_of_day_ms: int
    gps_valid: bool
    heading_valid: bool


class InfraPlaybackNode(Node):
    def __init__(self):
        super().__init__('infra_playback_node3')

        self.sdsm_csv_path = '/home/yz4d3h/Documents/infra_process/coninfra_log/032026/log_sdsm_hex.csv'
        self.pose_csv_path = '/home/yz4d3h/Documents/infra_process/coninfra_log/032026/log_ublox_gps.csv'

        # Playback time window in UNIX seconds
        self.playback_start_ts = 1774031110.0
        self.playback_end_ts = 1774031124.0

        # Timer periods (seconds)
        self.sdsm_timer_period = 0.001
        self.pose_timer_period = 0.001

        self.sdsm_pub = self.create_publisher(String, 'infra_local_measures', 10)
        self.publisher_gps = self.create_publisher(GPS, 'gps', 10)

        self.sdsm_records: List[SdsmRecord] = self._load_sdsm_csv(
            self.sdsm_csv_path,
            self.playback_start_ts,
            self.playback_end_ts
        )

        self.pose_records: List[PoseRecord] = self._load_pose_csv(
            self.pose_csv_path,
            self.playback_start_ts,
            self.playback_end_ts
        )

        if not self.sdsm_records and not self.pose_records:
            self.get_logger().error('No records found in either CSV within the requested time window.')
            raise RuntimeError('No playback data available.')

        # Common playback reference so both streams stay synchronized
        first_timestamps = []
        if self.sdsm_records:
            first_timestamps.append(self.sdsm_records[0].timestamp)
        if self.pose_records:
            first_timestamps.append(self.pose_records[0].timestamp)

        self.common_log_start_ts = min(first_timestamps)

        # Runtime state
        self.sdsm_index = 0
        self.pose_index = 0
        self.playback_started = False
        self.playback_start_wall_time = None

        # ---------------- Infra heading filter (MA)
        self.heading_alpha = self.declare_parameter('heading_alpha', 0.15).get_parameter_value().double_value
        self.min_speed_for_heading = self.declare_parameter('min_speed_for_heading', 1.0).get_parameter_value().double_value  # m/s

        self.filtered_heading = None
        self.have_filtered_heading = False

        self.get_logger().info(
            f'SDSM records loaded: {len(self.sdsm_records)}, '
            f'Pose records loaded: {len(self.pose_records)}'
        )
        self.get_logger().info(
            f'Playback window: [{self.playback_start_ts}, {self.playback_end_ts}]'
        )
        self.get_logger().info(
            f'Common playback reference timestamp: {self.common_log_start_ts}'
        )

        # Start playback
        self.playback_start_wall_time = self.get_clock().now()
        self.playback_started = True

        # Independent timers / loops
        self.sdsm_timer = self.create_timer(self.sdsm_timer_period, self._sdsm_loop)
        self.pose_timer = self.create_timer(self.pose_timer_period, self._pose_loop)

    def _load_sdsm_csv(self, path: str, start_ts: float, end_ts: float) -> List[SdsmRecord]:
        records = []
        with open(path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ts = float(row['timestamp (s)'].strip())
                    if start_ts <= ts <= end_ts:
                        records.append(
                            SdsmRecord(
                                timestamp=ts,
                                sdsm_hex=row[' sdsm_hex'].strip()
                            )
                        )
                except Exception as e:
                    self.get_logger().warn(f'Skipping bad SDSM row: {row}, error: {e}')

        records.sort(key=lambda x: x.timestamp)
        return records

    def _load_pose_csv(self, path: str, start_ts: float, end_ts: float) -> List[PoseRecord]:
        records = []
        with open(path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ts = float(row['timestamp (s)'].strip())
                    if start_ts <= ts <= end_ts:
                        records.append(
                            PoseRecord(
                                timestamp=ts,
                                lat_ms_arc=float(row['lat (deg)'].strip()),
                                lon_ms_arc=float(row['lon (deg)'].strip()),
                                heading_rad=float(row['heading (rad)'].strip()),
                                speed_kmh=float(row['calculated speed (km/h)'].strip()),
                                year=int(float(row['year - gps'].strip())),
                                day_of_year=int(float(row['day of year - gps'].strip())),
                                time_of_day_ms=int(float(row['time of day in ms - gps UTC'].strip())),
                                gps_valid = bool(row[' gps valid']),
                                heading_valid = bool(row[' heading valid'])
                            )
                        )
                except Exception as e:
                    self.get_logger().warn(f'Skipping bad pose row: {row}, error: {e}')

        records.sort(key=lambda x: x.timestamp)
        return records

    def _elapsed_playback_time(self) -> float:
        now = self.get_clock().now()
        return (now - self.playback_start_wall_time).nanoseconds * 1e-9

    def _sdsm_loop(self):
        if not self.playback_started or self.sdsm_index >= len(self.sdsm_records):
            self._check_finish()
            return

        elapsed = self._elapsed_playback_time()

        while self.sdsm_index < len(self.sdsm_records):
            rec = self.sdsm_records[self.sdsm_index]
            scheduled_offset = rec.timestamp - self.common_log_start_ts

            if elapsed + 1e-9 < scheduled_offset:
                break

            decoded = sdsm_decoder(rec.sdsm_hex)
            msg = String()
            msg.data = json.dumps(decoded)
            self.sdsm_pub.publish(msg)

            self.get_logger().info(
                f'Published SDSM[{self.sdsm_index}] ts={rec.timestamp:.6f}, hex={rec.sdsm_hex}'
            )
            self.sdsm_index += 1

        self._check_finish()

    def _pose_loop(self):
        if not self.playback_started or self.pose_index >= len(self.pose_records):
            self._check_finish()
            return

        elapsed = self._elapsed_playback_time()

        while self.pose_index < len(self.pose_records):
            rec = self.pose_records[self.pose_index]
            r = rec
            scheduled_offset = rec.timestamp - self.common_log_start_ts

            if elapsed + 1e-9 < scheduled_offset:
                break

            lat = rec.lat_ms_arc
            lon = rec.lon_ms_arc
            heading_rad = rec.heading_rad
            speed = rec.speed_kmh * 0.514 #(conversion)
            gps_valid = rec.gps_valid
            heading_valid = rec.heading_valid
            # Only update heading filter when heading is valid and speed is meaningful
            if heading_valid and gps_valid and speed >= self.min_speed_for_heading:
                self.filtered_heading = circ_exp_smooth(self.filtered_heading, heading_rad, self.heading_alpha)
                self.have_filtered_heading = True

            # If we never got a good heading yet, fall back to raw heading (or 0.0)
            heading_to_pub = self.filtered_heading if self.have_filtered_heading else heading_rad

            g = GPS()
            # g.timestamp = rec.timestamp
            g.timestamp = self.get_clock().now().to_msg()
            g.latitude = lat
            g.longitude = lon
            g.heading = heading_to_pub
            g.speed = speed
            self.publisher_gps.publish(g)

            self.get_logger().info(
                f'Published POSE[{self.pose_index}] ts={rec.timestamp:.6f}, '
                f'heading={rec.heading_rad:.3f}, speed={rec.speed_kmh:.3f}'
            )
            self.pose_index += 1

        self._check_finish()

    def _check_finish(self):
        sdsm_done = self.sdsm_index >= len(self.sdsm_records)
        pose_done = self.pose_index >= len(self.pose_records)

        if sdsm_done and pose_done:
            self.get_logger().info('Playback finished for both CSV streams.')
            self.sdsm_timer.cancel()
            self.pose_timer.cancel()
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InfraPlaybackNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
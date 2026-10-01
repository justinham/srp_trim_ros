#!/usr/bin/env python3
import math
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from fusion.msg import GPS, TripleVectorWps, VehicleCommand
import numpy as np
import re

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(value, hi))

def convert_xy_to_lat_lon(ref_lat_rad, ref_lon_rad, ref_heading_rad, x_m, y_m):
    f = 0.003353
    a = 6378137
    f1 = np.sqrt(f * (2 - f))
    sin_lat = np.sin(ref_lat_rad)
    denom = np.sqrt(1 - (f1 ** 2) * (sin_lat ** 2))
    f2 = a * (1 - f1 ** 2) / (denom ** 3)
    f3 = a / denom
    N = x_m * np.cos(ref_heading_rad) - y_m * np.sin(ref_heading_rad)
    E = x_m * np.sin(ref_heading_rad) + y_m * np.cos(ref_heading_rad)
    lat_rad = ref_lat_rad + N / f2
    lon_rad = ref_lon_rad + E / (f3 * np.cos(ref_lat_rad))
    return lat_rad*180/np.pi, lon_rad*180/np.pi

def nav_heading_to_math_yaw(h: float) -> float:
    return wrap_to_pi(math.pi / 2.0 - h)

class BicycleVehicleSimulator(Node):
    """Simple front-steered kinematic bicycle simulator for the MPC controller.

    Subscribes:
      - interpolated_wps (fusion/msg/TripleVectorWps)
      - vehicle_command (fusion/msg/VehicleCommand)

    Publishes:
      - gps (fusion/msg/GPS)
      - gps_can (fusion/msg/GPS)

    At the end of total_sim_steps, it saves a Cartesian X-Y plot containing:
      - reference path
      - traced path by the simulated bicycle model
    """

    def __init__(self) -> None:
        super().__init__('bicycle_vehicle_simulator_node')

        # Simulation parameters
        self.sim_period_s = float(self.declare_parameter('sim_period_s', 0.02).value)
        self.total_sim_steps = int(self.declare_parameter('total_sim_steps', 2500).value)
        self.wheelbase_m = float(self.declare_parameter('wheelbase_m', 3.18).value)
        self.steer_ratio = float(self.declare_parameter('steer_ratio', 18.33).value)
        self.command_steer_sign = float(self.declare_parameter('command_steer_sign', 1.0).value)
        self.rwa_max_deg = float(self.declare_parameter('rwa_max_deg', 45.0).value)
        self.steer_rate_limit_deg_s = float(self.declare_parameter('steer_rate_limit_deg_s', 180.0).value)
        self.accel_limit_mps2 = float(self.declare_parameter('accel_limit_mps2', 2.0).value)
        self.decel_limit_mps2 = float(self.declare_parameter('decel_limit_mps2', 3.0).value)
        self.brake_decel_mps2 = float(self.declare_parameter('brake_decel_mps2', 5.0).value)
        self.ref_lat_deg = float(self.declare_parameter('ref_lat_deg', 42.5150354710).value)
        self.ref_lon_deg = float(self.declare_parameter('ref_lon_deg', -83.0439986570).value)
        self.reset_on_new_path = bool(self.declare_parameter('reset_on_new_path', True).value)
        self.start_index = int(self.declare_parameter('start_index', 0).value)
        self.publish_gps_can = bool(self.declare_parameter('publish_gps_can', True).value)
        self.debug_log = bool(self.declare_parameter('debug_log', True).value)
        self.long_dir_forward_value = int(self.declare_parameter('long_dir_forward_value', 1).value)
        self.long_dir_reverse_value = int(self.declare_parameter('long_dir_reverse_value', 2).value)
        self.plot_output_path = str(self.declare_parameter('plot_output_path', '/home/yz4d3h/Documents/bicycle_sim_path.png').value)
        self.plot_output_path_steer = str(self.declare_parameter('plot_output_path_steer', '/home/yz4d3h/Documents/bicycle_sim_steer.png').value)
        self.shutdown_on_finish = bool(self.declare_parameter('shutdown_on_finish', True).value)

        self.rwa_max_rad = deg2rad(self.rwa_max_deg)
        self.steer_rate_limit_rad_s = deg2rad(self.steer_rate_limit_deg_s)

        # State
        self.path_valid = False
        self.have_state = False
        self.have_cmd = False
        self.finished = False
        self.brake_hold = False
        self.brake_rq = 0.0
        self.gear_sign = 1.0
        self.step_count = 0

        self.x_m = 2.685
        self.y_m = -0.685
        self.yaw_rad = 0*np.pi/180
        self.speed_mps = 0.0
        self.target_speed_mps = 2.0
        self.delta_rad = 0.0
        self.target_delta_rad = 0.0

        self.last_cmd: Optional[VehicleCommand] = None
        self.plan_x: List[float] = []
        self.plan_y: List[float] = []
        self.plan_h: List[float] = []
        self.trace_x: List[float] = []
        self.trace_y: List[float] = []
        self.steer_cmds: List[float] = []

        self.path_sub = self.create_subscription(
            TripleVectorWps, 'interpolated_wps', self.path_callback, 100)
        self.cmd_sub = self.create_subscription(
            VehicleCommand, 'vehicle_command', self.command_callback, 100)

        self.gps_pub = self.create_publisher(GPS, 'gps', 100)
        self.gps_can_pub = self.create_publisher(GPS, 'gps_can', 100)

        self.timer = self.create_timer(self.sim_period_s, self.tick)

        self.get_logger().info(
            f'BicycleVehicleSimulator started. dt={self.sim_period_s:.3f}s, '
            f'L={self.wheelbase_m:.3f}m, total_sim_steps={self.total_sim_steps}')

    def path_callback(self, msg: TripleVectorWps) -> None:
        if len(msg.plan_x) == 0 or len(msg.plan_x) != len(msg.plan_y) or len(msg.plan_x) != len(msg.plan_h):
            self.get_logger().error('Received invalid trajectory on interpolated_wps')
            self.path_valid = False
            return

        self.plan_x = list(msg.plan_x)
        self.plan_y = list(msg.plan_y)
        self.plan_h = [wrap_to_pi(float(h)) for h in msg.plan_h]
        self.path_valid = True

        idx = clamp(float(self.start_index), 0.0, float(len(self.plan_x) - 1))
        idx_int = int(idx)

        if (not self.have_state) or self.reset_on_new_path:
            self.x_m = float(self.plan_x[idx_int])
            self.y_m = float(self.plan_y[idx_int])
            self.yaw_rad = wrap_to_pi(float(self.plan_h[idx_int]))
            self.delta_rad = 0.0
            self.target_delta_rad = 0.0
            self.speed_mps = 0.0
            self.target_speed_mps = 0.0
            self.gear_sign = 1.0
            self.brake_hold = False
            self.brake_rq = 0.0
            self.have_state = True
            self.step_count = 0
            self.finished = False
            self.trace_x = [self.x_m]
            self.trace_y = [self.y_m]
            self.publish_state()

        self.get_logger().info(
            f'Received trajectory with {len(self.plan_x)} points. '
            f'reset={self.reset_on_new_path}, start_idx={idx_int}, '
            f'x={self.x_m:.3f}, y={self.y_m:.3f}, yaw={rad2deg(self.yaw_rad):.3f} deg')

    def command_callback(self, msg: VehicleCommand) -> None:
        self.last_cmd = msg
        self.have_cmd = True
        self.steer_cmds.append(msg.strgwhlang_rq)

        # Convert SWA command to road wheel angle command.
        rwa_deg = float(msg.strgwhlang_rq) / (self.command_steer_sign * self.steer_ratio)
        self.target_delta_rad = clamp(deg2rad(rwa_deg), -self.rwa_max_rad, self.rwa_max_rad)

        self.brake_hold = bool(msg.brake_hold_rq)
        self.brake_rq = max(0.0, float(msg.brake_rq))

        if int(msg.long_dir_rq) == self.long_dir_reverse_value:
            self.gear_sign = -1.0
        else:
            self.gear_sign = 1.0

        self.target_speed_mps = max(0.0, float(msg.velocity_rq))
        if self.brake_hold or self.brake_rq > 1e-6:
            self.target_speed_mps = 0.0

    def tick(self) -> None:
        if self.finished:
            return

        if (not self.have_state) or (not self.path_valid):
            self.publish_invalid_state()
            return

        self.step_steering()
        self.step_speed()
        self.integrate_bicycle_model()
        self.publish_state()

        self.trace_x.append(self.x_m)
        self.trace_y.append(self.y_m)
        self.step_count += 1

        if self.debug_log and (self.step_count % max(1, int(round(0.5 / self.sim_period_s))) == 0):
            self.get_logger().info(
                f'step={self.step_count}/{self.total_sim_steps} '
                f'x={self.x_m:.3f} y={self.y_m:.3f} yaw={rad2deg(self.yaw_rad):.3f}deg '
                f'v={self.speed_mps:.3f}mps delta={rad2deg(self.delta_rad):.3f}deg '
                f'target_delta={rad2deg(self.target_delta_rad):.3f}deg '
                f'gear={"R" if self.gear_sign < 0.0 else "F"}')

        if self.step_count >= self.total_sim_steps:
            self.finished = True
            self.save_path_plot()
            self.save_steer_plot()
            # plt.plot(self.steer_cmds)
            # plt.show()
            self.get_logger().info(
                f'Simulation finished after {self.total_sim_steps} steps. '
                f'Plot saved to {self.plot_output_path}')
            if self.shutdown_on_finish:
                self.get_logger().info('Shutting down simulator node.')
                self.destroy_timer(self.timer)
                rclpy.shutdown()

    def step_steering(self) -> None:
        max_step = self.steer_rate_limit_rad_s * self.sim_period_s
        err = self.target_delta_rad - self.delta_rad
        self.delta_rad += clamp(err, -max_step, max_step)
        self.delta_rad = clamp(self.delta_rad, -self.rwa_max_rad, self.rwa_max_rad)

    def step_speed(self) -> None:
        target_signed_speed = self.gear_sign * self.target_speed_mps
        max_delta_v = self.accel_limit_mps2 * self.sim_period_s
        if abs(target_signed_speed) < abs(self.speed_mps):
            max_delta_v = self.decel_limit_mps2 * self.sim_period_s
        if self.brake_hold or self.brake_rq > 1e-6:
            max_delta_v = self.brake_decel_mps2 * self.sim_period_s

        dv = target_signed_speed - self.speed_mps
        self.speed_mps += clamp(dv, -max_delta_v, max_delta_v)

        if abs(target_signed_speed) < 1e-6 and abs(self.speed_mps) < 1e-4:
            self.speed_mps = 0.0

    def integrate_bicycle_model(self) -> None:
        x_dot = self.speed_mps * math.cos(self.yaw_rad)
        y_dot = self.speed_mps * math.sin(self.yaw_rad)
        yaw_dot = (self.speed_mps / self.wheelbase_m) * math.tan(self.delta_rad)

        self.x_m += self.sim_period_s * x_dot
        self.y_m += self.sim_period_s * y_dot
        self.yaw_rad = wrap_to_pi(self.yaw_rad + self.sim_period_s * yaw_dot)

    def publish_invalid_state(self) -> None:
        gps_msg = GPS()
        gps_msg.valid = False
        gps_msg.heading_valid = False
        self.gps_pub.publish(gps_msg)

        if self.publish_gps_can:
            gps_can_msg = GPS()
            gps_can_msg.valid = False
            gps_can_msg.heading_valid = False
            self.gps_can_pub.publish(gps_can_msg)

    def publish_state(self) -> None:
        lat_deg, lon_deg = convert_xy_to_lat_lon(self.ref_lat_deg*np.pi/180, self.ref_lon_deg*np.pi/180, 0, self.y_m, self.x_m)

        gps_msg = GPS()
        gps_msg.valid = True
        gps_msg.heading_valid = True
        gps_msg.latitude = float(lat_deg)
        gps_msg.longitude = float(lon_deg)
        gps_msg.heading = float(self.yaw_rad)
        self.gps_pub.publish(gps_msg)

        if self.publish_gps_can:
            gps_can_msg = GPS()
            gps_can_msg.valid = True
            gps_can_msg.heading_valid = True
            gps_can_msg.latitude = float(lat_deg)
            gps_can_msg.longitude = float(lon_deg)
            gps_can_msg.heading = float(self.yaw_rad)
            self.gps_can_pub.publish(gps_can_msg)

    def save_path_plot(self) -> None:
        output_path = Path(self.plot_output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        fig, ax = plt.subplots(figsize=(10, 8))
        if self.plan_x and self.plan_y:
            ax.plot(self.plan_x, self.plan_y, linestyle='--', linewidth=2.0, label='Reference path')
            ax.scatter(self.plan_x[0], self.plan_y[0], marker='o', s=60, label='Reference start')
            ax.scatter(self.plan_x[-1], self.plan_y[-1], marker='x', s=80, label='Reference end')

        if self.trace_x and self.trace_y:
            ax.plot(self.trace_x, self.trace_y, linewidth=2.0, label='Traced sim path')
            ax.scatter(self.trace_x[0], self.trace_y[0], marker='o', s=60, label='Trace start')
            ax.scatter(self.trace_x[-1], self.trace_y[-1], marker='x', s=80, label='Trace end')

        file_path = "/home/yz4d3h/Downloads/mpc_log_h30.csv"   # change this
        x_vals = []
        y_vals = []
        pattern = re.compile(r"xy=\(\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*\)")
        with open(file_path, "r") as f:
            for line in f:
                match = pattern.search(line)
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    x_vals.append(x)
                    y_vals.append(y)
        ax.plot(y_vals[:120], x_vals[:120], marker="o", label="Traced real vehicle path")

        ax.set_title('Reference Path vs Simulated Bicycle Trace')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output_path, dpi=150)
        plt.close(fig)

    def save_steer_plot(self) -> None:
        output_path = Path(self.plot_output_path_steer)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        fig, ax = plt.subplots(figsize=(10, 8))
        if self.plan_x and self.plan_y:
            ax.plot(self.steer_cmds)

        ax.set_title('Steer cmds')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        # ax.axis('equal')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output_path, dpi=150)
        plt.close(fig)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BicycleVehicleSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.finished and node.trace_x and node.trace_y:
            node.save_path_plot()
            node.get_logger().info(f'Interrupted. Plot saved to {node.plot_output_path}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

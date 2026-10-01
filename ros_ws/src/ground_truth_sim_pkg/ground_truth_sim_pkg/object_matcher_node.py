#!/usr/bin/env python3

import copy
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.optimize import linear_sum_assignment
from scipy.spatial import distance
from std_msgs.msg import String


class ObjectMatcherNode(Node):
    def __init__(self):
        super().__init__('object_matcher_node')

        # ---------------- Params ----------------
        self.max_match_time_diff = 0.2
        self.id_memory_timeout = 1.0
        self.max_arrival_wait_sec = 0.3
        self.position_weight = 0.3
        self.velocity_weight = 2.0
        self.match_cost_threshold = 5.0

        # ---------------- Subscribers ----------------
        self.ego_sub = self.create_subscription(String,'ego_global_measures',self.ego_callback,10)
        self.infra_sub = self.create_subscription(String,'infra_global_measures',self.infra_callback,10)
        self.app_cmd_sub = self.create_subscription(String,'app_cmd',self.app_cmd_callback,10)

        # ---------------- Publishers ----------------
        self.ego_pub = self.create_publisher(String,'matched_ego_actors',10)
        self.infra_pub = self.create_publisher(String,'matched_infra_actors',10)

        # ---------------- State ----------------
        self.ego_buffer = []
        self.infra_buffer = []
        self.app_sensing_cmd = 'infra_sen'

        # infra raw ID -> {"canonical_id": ego_id, "last_seen_time": t}
        self.infra_to_canonical = {}
        self.latest_observed_ts = None

        # timer helps flush unmatched frames even if only one stream is active.
        self.flush_timer = self.create_timer(0.05, self.try_process_buffers)

        self.get_logger().info(f"Object matcher started. max_match_time_diff={self.max_match_time_diff:.3f}s")

    def ego_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.push_frame(self.ego_buffer, data, source="ego")
            self.try_process_buffers()
        except Exception as e:
            self.get_logger().warn(f"failed in ego_callback: {e}")

    def infra_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.push_frame(self.infra_buffer, data, source="infra")
            self.try_process_buffers()
        except Exception as e:
            self.get_logger().warn(f"failed in infra_callback: {e}")

    def app_cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['name'] == 'sen_select':
                self.app_sensing_cmd = data['value']
        except Exception as e:
            self.get_logger().warn(f"failed in app_cmd_callback: {e}")
            
    # =========================================================================
    # Buffer management
    # =========================================================================

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def get_timestamp(self, message):
        return float(message["timestamp"])

    def push_frame(self, buffer, message, source):
        t = self.get_timestamp(message)

        frame = {
            "timestamp": t,
            "message": message,
            "source": source,
            "arrival_ros_time": self.now_sec()
        }

        buffer.append(frame)
        buffer.sort(key=lambda f: f["timestamp"])

        if self.latest_observed_ts is None:
            self.latest_observed_ts = t
        else:
            self.latest_observed_ts = max(self.latest_observed_ts, t)

    def find_closest_frame_idx(self, buffer, target_t):
        if not buffer:
            return None, None

        best_idx = None
        best_dt = float("inf")

        for i, frame in enumerate(buffer):
            dt = abs(frame["timestamp"] - target_t)
            if dt < best_dt:
                best_dt = dt
                best_idx = i

        return best_idx, best_dt

    def should_flush_unmatched(self, frame):
        t = frame["timestamp"]

        # Measurement-time based flushing.
        if self.latest_observed_ts is not None:
            if t < self.latest_observed_ts - self.max_match_time_diff:
                return True

        # Arrival-time based flushing. Useful when one stream stops.
        age = self.now_sec() - frame["arrival_ros_time"]
        if age > self.max_arrival_wait_sec:
            return True

        return False

    def try_process_buffers(self):
        made_progress = True

        while made_progress:
            made_progress = False

            # Case A: both buffers have frames.
            if self.ego_buffer and self.infra_buffer:
                ego_t = self.ego_buffer[0]["timestamp"]
                infra_t = self.infra_buffer[0]["timestamp"]

                # Process whichever frame is earlier.
                if ego_t <= infra_t:
                    ego_frame = self.ego_buffer[0]
                    best_idx, best_dt = self.find_closest_frame_idx(self.infra_buffer, ego_frame["timestamp"])

                    if best_idx is not None and best_dt <= self.max_match_time_diff:
                        ego_frame = self.ego_buffer.pop(0)
                        infra_frame = self.infra_buffer.pop(best_idx)

                        self.process_matching_pair(ego_frame, infra_frame)
                        made_progress = True

                    elif self.should_flush_unmatched(ego_frame):
                        ego_frame = self.ego_buffer.pop(0)
                        self.publish_ego_only(ego_frame)
                        made_progress = True

                else:
                    infra_frame = self.infra_buffer[0]
                    best_idx, best_dt = self.find_closest_frame_idx(self.ego_buffer, infra_frame["timestamp"])

                    if best_idx is not None and best_dt <= self.max_match_time_diff:
                        infra_frame = self.infra_buffer.pop(0)
                        ego_frame = self.ego_buffer.pop(best_idx)

                        self.process_matching_pair(ego_frame, infra_frame)
                        made_progress = True

                    elif self.should_flush_unmatched(infra_frame):
                        infra_frame = self.infra_buffer.pop(0)
                        self.publish_infra_only(infra_frame)
                        made_progress = True

            # Case B: only ego buffer has frames.
            elif self.ego_buffer:
                ego_frame = self.ego_buffer[0]

                if self.should_flush_unmatched(ego_frame):
                    ego_frame = self.ego_buffer.pop(0)
                    self.publish_ego_only(ego_frame)
                    made_progress = True

            # Case C: only infra buffer has frames.
            elif self.infra_buffer:
                infra_frame = self.infra_buffer[0]

                if self.should_flush_unmatched(infra_frame):
                    infra_frame = self.infra_buffer.pop(0)
                    self.publish_infra_only(infra_frame)
                    made_progress = True

    # =========================================================================
    # Matching logic
    # =========================================================================

    def process_matching_pair(self, ego_frame, infra_frame):
        ego_msg = ego_frame["message"]
        infra_msg = infra_frame["message"]

        ego_t = ego_frame["timestamp"]
        infra_t = infra_frame["timestamp"]

        ego_actors = self.get_actors(ego_msg)
        infra_actors = self.get_actors(infra_msg)

        # For association cost only, compare at ego timestamp.
        infra_actors_at_ego_t = self.propagate_actors(infra_actors,from_t=infra_t,to_t=ego_t)
        matched_pairs = self.multi_feature_hungarian_matching(ego_actors,infra_actors_at_ego_t)

        # matched_pairs: ego_id -> infra_id
        assigned_infra_ids = {}

        for ego_id, infra_id in matched_pairs.items():
            assigned_infra_ids[infra_id] = ego_id
            self.infra_to_canonical[infra_id] = {
                "canonical_id": ego_id,
                "last_seen_time": max(ego_t, infra_t)
            }

        # Unmatched infra objects keep previous canonical ID briefly, if available.
        for infra_obj in infra_actors:
            infra_id = self.actor_id(infra_obj)
            if infra_id not in assigned_infra_ids:
                assigned_infra_ids[infra_id] = self.get_infra_canonical_id(infra_id, current_t=infra_t)

        ego_output = self.format_output(ego_msg, ego_actors, assigned_ids={}, source="ego")
        infra_output = self.format_output(infra_msg, infra_actors, assigned_ids=assigned_infra_ids, source="infra")

        self.get_logger().info(f"Published matched-INFRA frame: {infra_output.data}")

        if self.app_sensing_cmd != 'infra_sen':
            self.ego_pub.publish(ego_output)
        if self.app_sensing_cmd != 'ego_sen':
            self.infra_pub.publish(infra_output)

        self.cleanup_id_memory(max(ego_t, infra_t))

        self.get_logger().info(
            f"Matched pair: ego_t={ego_t:.3f}, infra_t={infra_t:.3f}, "
            f"dt={abs(ego_t - infra_t):.3f}, matches={len(matched_pairs)}"
        )

    def multi_feature_hungarian_matching(self, ego_actors, infra_actors):
        matched_pairs = {}

        if not ego_actors or not infra_actors:
            return matched_pairs

        ego_positions = np.array([self.actor_xy(a) for a in ego_actors], dtype=float)
        infra_positions = np.array([self.actor_xy(a) for a in infra_actors], dtype=float)

        ego_velocities = np.array([self.actor_vxy(a) for a in ego_actors], dtype=float)
        infra_velocities = np.array([self.actor_vxy(a) for a in infra_actors], dtype=float)

        pos_cost_matrix = distance.cdist(ego_positions, infra_positions, metric="euclidean")
        vel_cost_matrix = distance.cdist(ego_velocities, infra_velocities, metric="euclidean")

        cost_matrix = (self.position_weight * pos_cost_matrix + self.velocity_weight * vel_cost_matrix)

        # optional class gating, if both have class labels and they differ,
        # make that pair effectively impossible.
        for i, ego_obj in enumerate(ego_actors):
            for j, infra_obj in enumerate(infra_actors):
                ego_class = str(ego_obj.get("class", "")).lower()
                infra_class = str(infra_obj.get("class", "")).lower()

                if ego_class and infra_class and ego_class != infra_class:
                    cost_matrix[i, j] += 1e6

        ego_indices, infra_indices = linear_sum_assignment(cost_matrix)

        for ego_idx, infra_idx in zip(ego_indices, infra_indices):
            cost = cost_matrix[ego_idx, infra_idx]
            self.get_logger().info(f"cost: {cost}")

            if cost < self.match_cost_threshold:
                ego_id = self.actor_id(ego_actors[ego_idx])
                infra_id = self.actor_id(infra_actors[infra_idx])
                matched_pairs[ego_id] = infra_id

        return matched_pairs

    # =========================================================================
    # Single-source modes
    # =========================================================================

    def publish_ego_only(self, ego_frame):
        ego_msg = ego_frame["message"]
        ego_actors = self.get_actors(ego_msg)

        out = self.format_output(ego_msg,ego_actors,assigned_ids={}, source="ego")
        if self.app_sensing_cmd != 'infra_sen':
            self.ego_pub.publish(out)

        self.get_logger().info(f"Published ego-only frame at t={ego_frame['timestamp']:.3f}")

    def publish_infra_only(self, infra_frame):
        infra_msg = infra_frame["message"]
        infra_t = infra_frame["timestamp"]
        infra_actors = self.get_actors(infra_msg)

        assigned_infra_ids = {}

        for infra_obj in infra_actors:
            infra_id = self.actor_id(infra_obj)
            assigned_infra_ids[infra_id] = self.get_infra_canonical_id(infra_id, current_t=infra_t)

        out = self.format_output(infra_msg,infra_actors,assigned_ids=assigned_infra_ids,source="infra")
        if self.app_sensing_cmd != 'ego_sen':
            self.infra_pub.publish(out)
        self.cleanup_id_memory(infra_t)

        self.get_logger().info(f"Published infra-only frame at t={infra_t:.3f}")

    # =========================================================================
    # ID memory
    # =========================================================================

    def get_infra_canonical_id(self, infra_id, current_t):
        if infra_id in self.infra_to_canonical:
            entry = self.infra_to_canonical[infra_id]

            age = current_t - entry["last_seen_time"]

            if age <= self.id_memory_timeout:
                return entry["canonical_id"]

        return infra_id

    def cleanup_id_memory(self, current_t):
        ids_to_delete = []

        for infra_id, entry in self.infra_to_canonical.items():
            if current_t - entry["last_seen_time"] > self.id_memory_timeout:
                ids_to_delete.append(infra_id)

        for infra_id in ids_to_delete:
            del self.infra_to_canonical[infra_id]

    # =========================================================================
    # Helpers
    # =========================================================================

    def get_actors(self, message):
        # keep your existing format
        return message.get("visible_actors", [])

    def actor_id(self, actor):
        return str(actor["uniqueId"])

    def actor_xy(self, actor):
        pos = actor["pos"]

        if "lat" in pos and "lon" in pos:
            return [float(pos["lat"]), float(pos["lon"])]

        if "pos_x" in pos and "pos_y" in pos:
            return [float(pos["pos_x"]), float(pos["pos_y"])]

        raise KeyError(f"Unknown position format: {pos.keys()}")

    def actor_vxy(self, actor):
        vel = actor["vel"]

        if "lat_vel" in vel and "lon_vel" in vel:
            return [float(vel["lat_vel"]), float(vel["lon_vel"])]

        if "vel_x" in vel and "vel_y" in vel:
            return [float(vel["vel_x"]), float(vel["vel_y"])]

        raise KeyError(f"Unknown velocity format: {vel.keys()}")

    def set_actor_xy(self, actor, x, y):
        pos = actor["pos"]

        if "lat" in pos and "lon" in pos:
            pos["lat"] = x
            pos["lon"] = y
            return

        if "pos_x" in pos and "pos_y" in pos:
            pos["pos_x"] = x
            pos["pos_y"] = y
            return

        raise KeyError(f"Unknown position format: {pos.keys()}")

    def propagate_actors(self, actors, from_t, to_t):
        dt = to_t - from_t

        propagated = []

        for actor in actors:
            a = copy.deepcopy(actor)

            x, y = self.actor_xy(a)
            vx, vy = self.actor_vxy(a)

            self.set_actor_xy(
                a,
                x + vx * dt,
                y + vy * dt)

            propagated.append(a)

        return propagated

    def format_output(self, message, actors, assigned_ids, source):
        output = {
            "timestamp": message["timestamp"],
            "visible_actors": []
        }

        for actor in actors:
            raw_id = self.actor_id(actor)
            assigned_id = assigned_ids.get(raw_id, raw_id)

            new_actor = copy.deepcopy(actor)

            # preserving original source-local ID for debugging.
            new_actor["sourceObjectId"] = raw_id
            new_actor["source"] = source

            # ID used by downstream FG node.
            new_actor["uniqueId"] = assigned_id

            output["visible_actors"].append(new_actor)

        msg = String()
        msg.data = json.dumps(output)
        self.get_logger().info(f"source: {source} | data: {output}")
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMatcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
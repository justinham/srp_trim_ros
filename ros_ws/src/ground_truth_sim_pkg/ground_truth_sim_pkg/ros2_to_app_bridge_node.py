#!/usr/bin/env python3
import asyncio, json, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import websockets
import threading
import signal
import sys


INFRA_MAX_AGE_SEC = 0.5  # drop actors if infra data older than this


class Merger(Node):
    def __init__(self):
        super().__init__("ros2_to_app_bridge_node")

        self.lock = threading.Lock()

        # Cached pieces
        self.latest_ego = None        # {"x","y","yaw"}
        self.latest_infra = None      # {"ref": {...}, "actors": [...]}
        self.last_infra_walltime = None
        self.latest_obstacles = set()
        self.latest_frame = None      # full frame dict
        self.frame_seq = 0            # monotonically increasing

        # Subscriptions
        self.create_subscription(String, "ego_states", self.on_ego, 10)
        self.create_subscription(String, "fg_fused_topic", self.on_infra, 10)
        # self.create_subscription(String, "ego_actor_states", self.on_infra, 10)
        self.create_subscription(String, "obstacles_id_data", self.on_obstacles, 10)
        self.app_cmd_pub = self.create_publisher(String, "app_cmd", 10)

        self.latest_path = []
        self.create_subscription(String, "ego_waypoints", self.on_ego_wps, 10)

    def on_ego_wps(self, msg: String):
        try:
            d = json.loads(msg.data)
            wps = d.get("waypoints", [])
            # Downsample: keep every 4th point (but tunable)
            ds = wps[::4]
            with self.lock:
                self.latest_path = ds
            self._rebuild_frame()
        except Exception as e:
            self.get_logger().warn(f"on_ego_wps parse error: {e}")


    # ---------------- ROS Callbacks ----------------

    def on_obstacles(self, msg: String):
        try:
            d = json.loads(msg.data)  # {'ids': [...]}
            ids = d.get("ids", [])
            with self.lock:
                self.latest_obstacles = {str(x) for x in ids}
            # obstacles-only changes to be pushed immediately:
            self._rebuild_frame()
        except Exception as e:
            self.get_logger().warn(f"on_obstacles parse error: {e}")

    def on_ego(self, msg: String):
        try:
            data = json.loads(msg.data)
            ego = data.get("ego_vehicle", {})
            pos = ego.get("pos", {})
            heading = ego.get("heading", {})

            x = float(pos.get("lon", 0.0))
            y = float(pos.get("lat", 0.0))
            # theta is degrees in publisher: theta = -heading*180/pi
            theta_deg = float(heading.get("theta", 0.0))
            yaw = math.radians(theta_deg)

            with self.lock:
                self.latest_ego = {"x": x, "y": y, "yaw": yaw}

            # Ego drives the frame: rebuild every ego update
            self._rebuild_frame()
        except Exception as e:
            self.get_logger().warn(f"on_ego parse error: {e}")

    def on_infra(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"Input: {data}")
            
            ref = data.get("refPos", {"x": 0.0, "y": 0.0})
            if ref is None:
                ref = {"x": 0.0, "y": 0.0}
            actors_in = data.get("visible_actors", [])
            actors = []
            for a in actors_in:
                p = a.get("estimated_position", {})
                actors.append({
                    "id":  str(a.get("uniqueId", "")),
                    "cls": str(a.get("class", "unknown")),
                    "x":   float(p.get("x", 0.0)),
                    "y":   float(p.get("y", 0.0)),
                    "yaw": float(a.get("estimated_heading", 0.0)),  # already rad
                })
            now = time.time()
            with self.lock:
                self.latest_infra = {
                    "ref": {
                        "x": float(ref.get("x", 0.0)),
                        "y": float(ref.get("y", 0.0))
                    },
                    "actors": actors,
                }
                self.last_infra_walltime = now
            self._rebuild_frame()
        except Exception as e:
            self.get_logger().warn(f"on_infra parse error: {e}")

    # ---------------- Frame builder ----------------

    def _rebuild_frame(self):
        """
        Combine latest_ego, latest_infra, obstacles into a single frame.
        Driven primarily by ego, but we are droping stale infra actor states.
        """
        now = time.time()
        with self.lock:
            if self.latest_ego is None:
                return

            ego = dict(self.latest_ego)

            # Default ref and actors
            ref = {"x": 0.0, "y": 0.0}
            actors = []

            # Use infra only if it's not too old
            if self.latest_infra is not None and self.last_infra_walltime is not None:
                age = now - self.last_infra_walltime
                if age <= INFRA_MAX_AGE_SEC:
                    ref = dict(self.latest_infra["ref"])
                    actors = [dict(a) for a in self.latest_infra["actors"]]
                else:
                    # Infra is stale; log once every so often if you like
                    self.get_logger().debug(
                        f"Infra data stale (age={age:.3f}s) -> not sending actors"
                    )

            self.frame_seq += 1
            frame = {
                "seq": self.frame_seq,
                "t": now,
                "ego": ego,
                "actors": actors,
                "ref": ref,
                "obstacle_ids": list(self.latest_obstacles),
                "ego_path": self.latest_path,
            }

            self.latest_frame = frame
            # For debugging:
            print(self.latest_frame)


# ---------------- WebSocket server ----------------

async def ws_server(merger: Merger, host="0.0.0.0", port=8765):
    async def handler(websocket):
        # Do NOT push the last frame immediately on connect.
        # Start from current seq so we only send new frames.
        with merger.lock:
            f0 = dict(merger.latest_frame) if merger.latest_frame else None
        last_seq = f0["seq"] if f0 is not None else -1

        async def recv_loop():
            async for msg in websocket:
                try:
                    d = json.loads(msg)
                    if d.get("type") in ["toggle", "selection"]:
                        out = String()
                        out.data = json.dumps(d)
                        merger.app_cmd_pub.publish(out)
                        merger.get_logger().info(f"App cmd: {out.data}")
                except Exception as e:
                    merger.get_logger().warn(f"Bad app msg: {e}")

        asyncio.create_task(recv_loop())

        while True:
            with merger.lock:
                f = dict(merger.latest_frame) if merger.latest_frame else None

            if f is not None and f.get("seq", -1) != last_seq:
                await websocket.send(json.dumps(f))
                last_seq = f.get("seq", -1)

            await asyncio.sleep(0.02)

    server = await websockets.serve(handler, host, port)
    print(f"WS listening on ws://{host}:{port}")
    return server


def main():
    rclpy.init()
    merger = Merger()

    # Spin ROS in background thread
    th = threading.Thread(target=rclpy.spin, args=(merger,), daemon=True)
    th.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    server = loop.run_until_complete(ws_server(merger))

    def shutdown(*_):
        print("Shutting down bridge cleanly...")
        server.close()
        loop.stop()
        merger.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        loop.run_forever()
    finally:
        shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, math, numpy as np
from collections import defaultdict

import gtsam
from gtsam import noiseModel

# ======= helpers =======

def sym(c, k): return gtsam.symbol(c, k)

def clamp(x, lo, hi):
    return hi if x>hi else lo if x<lo else x

def wrap_to_pi(a):
    """Wrap radian angle to (-pi, pi]."""
    return (a + np.pi) % (2*np.pi) - np.pi

class ActorState:
    """Per-actor state: buffers, graph, iSAM2, last results."""
    def __init__(self, freq_hz: float):
        self.freq = freq_hz
        self.bin = 1.0/freq_hz

        self.buff = {}             # {k: {"t": t_k, "ego": meas, "infra": meas}}
        self.k_times = {}          # {k: t_k}
        self.last_k = None         # last solved k
        self.last_est = None       # dict of latest estimate at last_k
        self.last_heading = None   # radians
        self.ready_any = False

        # GTSAM structures (persistent)
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()

        p = gtsam.ISAM2Params()
        p.setRelinearizeThreshold(0.1)
        p.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(p)

    def keys(self, k):
        return (sym('x',k), sym('y',k), sym('u',k), sym('v',k), sym('h',k))

    def seed_from_prev(self, k, dt, result):
        """Initial guess rollout from k-1."""
        xk, yk, uk, vk, hk = self.keys(k)
        xm, ym, um, vm, hm = self.keys(k-1)

        x_prev = result.atDouble(xm)
        y_prev = result.atDouble(ym)
        u_prev = result.atDouble(um)
        v_prev = result.atDouble(vm)
        h_prev = result.atDouble(hm)

        # Constant-accel: use velocity hold for seed (accel still comes via factors)
        self.initial.insert(xk, x_prev + u_prev*dt)
        self.initial.insert(yk, y_prev + v_prev*dt)
        self.initial.insert(uk, u_prev)
        self.initial.insert(vk, v_prev)
        self.initial.insert(hk, h_prev)

    def seed_from_meas(self, k, meas):
        """Seed k from (any) measurement values (ego or infra)."""
        xk, yk, uk, vk, hk = self.keys(k)
        pos = meas['pos']; vel = meas['vel']; head = meas['heading']

        self.initial.insert(xk, pos['lat'])
        self.initial.insert(yk, pos['lon'])
        self.initial.insert(uk, vel['lat_vel'])
        self.initial.insert(vk, vel['lon_vel'])
        self.initial.insert(hk, head['theta'])

# ======= node =======

class FusionMeasNode(Node):
    def __init__(self):
        super().__init__('estimate_fused_node')

        # --- config ---
        self.FREQ = 10.0                      # publish & bin rate (Hz)
        self.BIN  = 1.0/self.FREQ
        self.WM   = 0.08                      # watermark (s) to wait for late infra
        # process noise (tune!)
        self.Qx = noiseModel.Isotropic.Sigma(1, 0.3)
        self.Qy = noiseModel.Isotropic.Sigma(1, 0.3)
        self.Qu = noiseModel.Isotropic.Sigma(1, 0.5)
        self.Qv = noiseModel.Isotropic.Sigma(1, 0.5)
        self.Qh = noiseModel.Isotropic.Sigma(1, 0.05)

        # per-actor containers
        self.actors = {}                      # actor_id -> ActorState
        self.actor_class = {}                 # actor_id -> class string
        self.ego_id = None

        # pubs/subs
        self.pub = self.create_publisher(String, 'fused_actor_states', 10)
        self.sub_ego   = self.create_subscription(String, 'matched_ego_actors',   self.cb_ego,   50)
        self.sub_infra = self.create_subscription(String, 'matched_infra_actors', self.cb_infra, 50)

        # fixed-rate publishing
        self.timer = self.create_timer(self.BIN, self.on_tick)

        self.get_logger().info("Fusion node up (iSAM2, temporal, dual-source).")

    # ----- time binning -----

    def k_from_t(self, t):
        return int(round(t / self.BIN))

    def ensure_actor(self, actor_id):
        if actor_id not in self.actors:
            self.actors[actor_id] = ActorState(freq_hz=self.FREQ)
        return self.actors[actor_id]

    def put_meas(self, source, t, actor):
        """Place measurement into its time bin."""
        a_id = actor['uniqueId']
        a_cls = actor['class']
        self.actor_class[a_id] = a_cls

        k = self.k_from_t(t)
        ast = self.ensure_actor(a_id)
        binrec = ast.buff.setdefault(k, {"t": k*self.BIN})
        binrec[source] = actor
        ast.k_times[k] = binrec["t"]

        # mark to process soon
        ast.ready_any = True

    # ----- callbacks -----

    def cb_ego(self, msg):
        try:
            data = json.loads(msg.data)
            ego = data.get("ego_vehicle")
            if ego:
                self.ego_id = ego["uniqueId"]
                t = float(data.get("timestamp", self.get_clock().now().nanoseconds/1e9))
                self.put_meas("ego", t, ego)
            for a in data.get("visible_actors", []):
                self.put_meas("ego", float(data.get("timestamp", self.get_clock().now().nanoseconds/1e9)), a)
        except Exception as e:
            self.get_logger().warn(f"ego parse error: {e}")

    def cb_infra(self, msg):
        try:
            data = json.loads(msg.data)
            ego = data.get("ego_vehicle")
            if ego:
                t = float(data.get("timestamp", self.get_clock().now().nanoseconds/1e9))
                self.put_meas("infra", t, ego)
            for a in data.get("visible_actors", []):
                self.put_meas("infra", float(data.get("timestamp", self.get_clock().now().nanoseconds/1e9)), a)
        except Exception as e:
            self.get_logger().warn(f"infra parse error: {e}")

    # ----- graph updates -----

    def add_priors_for_source(self, ast: ActorState, k: int, actor: dict, source_tag: str):
        """Add measurement priors for x,y,u,v,h at time k from a single source."""
        xk, yk, uk, vk, hk = ast.keys(k)
        pos = actor['pos']; vel = actor['vel']; head = actor['heading']

        # clamp stddevs; keep radians
        sx = clamp(pos.get("posStdDev_lat", 0.5), 1e-4, 10.0)
        sy = clamp(pos.get("posStdDev_lon", 0.5), 1e-4, 10.0)
        su = clamp(vel.get("velStdDev_lat", 0.5), 1e-4, 10.0)
        sv = clamp(vel.get("velStdDev_lon", 0.5), 1e-4, 10.0)
        sh = clamp(head.get("thetaStdDev", 0.25), 1e-4, 1.0)

        nm_x = noiseModel.Isotropic.Sigma(1, sx)
        nm_y = noiseModel.Isotropic.Sigma(1, sy)
        nm_u = noiseModel.Isotropic.Sigma(1, su)
        nm_v = noiseModel.Isotropic.Sigma(1, sv)
        nm_h = noiseModel.Isotropic.Sigma(1, sh)

        # unwrap heading consistently per actor
        theta = float(head['theta'])
        if ast.last_heading is not None:
            d = wrap_to_pi(theta - ast.last_heading)
            theta = ast.last_heading + d
        ast.last_heading = theta

        ast.graph.add(gtsam.PriorFactorDouble(xk, float(pos['lat']), nm_x))
        ast.graph.add(gtsam.PriorFactorDouble(yk, float(pos['lon']), nm_y))
        ast.graph.add(gtsam.PriorFactorDouble(uk, float(vel['lat_vel']), nm_u))
        ast.graph.add(gtsam.PriorFactorDouble(vk, float(vel['lon_vel']), nm_v))
        ast.graph.add(gtsam.PriorFactorDouble(hk, theta, nm_h))

    def add_motion(self, ast: ActorState, k_prev: int, k: int):
        """Add constant-accel/constant-yawrate between factors from k-1 to k."""
        x0,y0,u0,v0,h0 = ast.keys(k_prev)
        x1,y1,u1,v1,h1 = ast.keys(k)

        dt = ast.k_times[k] - ast.k_times[k_prev]
        dt = max(dt, 1e-3)

        # Deltas (what between-factors measure)
        # position delta predicted by previous velocity (accel will be absorbed by velocity delta)
        # You can also include 0.5*a*dt^2 via an accel estimate if you maintain it.
        # Here we keep it simple and let Δv capture accel.
        dx = 0.0  # measured delta is modeled as zero-mean with process noise; use velocity link instead
        dy = 0.0

        # Better: encode kinematic expectation explicitly via predicted deltas
        # but keep simple zero-mean with reasonable Q is common and robust:
        ast.graph.add(gtsam.BetweenFactorDouble(x0, x1, dx, self.Qx))
        ast.graph.add(gtsam.BetweenFactorDouble(y0, y1, dy, self.Qy))

        du = 0.0
        dv = 0.0
        dh = 0.0
        ast.graph.add(gtsam.BetweenFactorDouble(u0, u1, du, self.Qu))
        ast.graph.add(gtsam.BetweenFactorDouble(v0, v1, dv, self.Qv))
        ast.graph.add(gtsam.BetweenFactorDouble(h0, h1, dh, self.Qh))

    def seed_initial(self, ast: ActorState, k: int):
        """Seed initial for k from previous result if exists, else from any meas in bin."""
        if ast.last_k is not None:
            # need previous result
            result = ast.isam.calculateEstimate()
            dt = ast.k_times[k] - ast.k_times[ast.last_k]
            dt = max(dt, 1e-3)
            try:
                ast.seed_from_prev(k, dt, result)
                return
            except Exception:
                pass
        # fallback: seed from any available measurement at k
        binrec = ast.buff[k]
        meas = binrec.get("ego", binrec.get("infra"))
        ast.seed_from_meas(k, meas)

    # ----- main tick -----

    def on_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        fused = {
            "timestamp": now,
            "ego_vehicle": [],
            "visible_actors": []
        }

        # build & solve per-actor
        for a_id, ast in list(self.actors.items()):
            if not ast.ready_any:
                # still publish last known at fixed rate if available
                if ast.last_est is not None:
                    self._append_estimate(fused, a_id, ast.last_est)
                continue

            # pick bins ready to close
            ready_ks = sorted(k for k,v in ast.buff.items()
                              if (now - v["t"] >= self.WM) and (ast.last_k is None or k > ast.last_k))
            if not ready_ks:
                # nothing ready; publish last
                if ast.last_est is not None:
                    self._append_estimate(fused, a_id, ast.last_est)
                continue

            # prepare graph additions for all ready ks
            for k in ready_ks:
                binrec = ast.buff[k]
                # add BOTH sources' priors if present (two priors on same key is intended)
                if "ego" in binrec:
                    self.add_priors_for_source(ast, k, binrec["ego"], "ego")
                if "infra" in binrec:
                    self.add_priors_for_source(ast, k, binrec["infra"], "infra")

                # motion factor if previous k exists
                if ast.last_k is not None:
                    self.add_motion(ast, ast.last_k, k)

                # seed
                self.seed_initial(ast, k)

                # mark progressed
                ast.last_k = k

            # one incremental update
            if ast.graph.size() > 0 or len(ast.initial.keys()) > 0:
                ast.isam.update(ast.graph, ast.initial)
                result = ast.isam.calculateEstimate()

                # collect latest estimate at last_k
                xk, yk, uk, vk, hk = ast.keys(ast.last_k)
                try:
                    est = {
                        "x": result.atDouble(xk),
                        "y": result.atDouble(yk),
                        "u": result.atDouble(uk),
                        "v": result.atDouble(vk),
                        "h": wrap_to_pi(result.atDouble(hk))
                    }
                    ast.last_est = est
                    self._append_estimate(fused, a_id, est)
                except Exception as e:
                    self.get_logger().warn(f"extract result failed for {a_id}: {e}")

                # clear pending
                ast.graph.resize(0)
                ast.initial.clear()

            # keep memory tidy: drop very old bins
            # keep only last few around (optional)
            keep_after = max(ready_ks[-1] - int(self.FREQ*5), 0)  # ~5s window
            ast.buff = {k:v for k,v in ast.buff.items() if k >= keep_after}
            ast.k_times = {k:t for k,t in ast.k_times.items() if k >= keep_after}
            ast.ready_any = False  # will be set again by callbacks when new data arrives

        # publish at fixed rate
        out = String()
        out.data = json.dumps(fused)
        self.pub.publish(out)

    def _append_estimate(self, fused, actor_id, est):
        a_cls = self.actor_class.get(actor_id, "unknown")
        rec = {
            "uniqueId": str(actor_id),
            "class": a_cls,
            "estimated_position": {"x": est["x"], "y": est["y"]},
            "estimated_velocity": {"v_x": est["u"], "v_y": est["v"]},
            "estimated_heading": est["h"]  # radians
        }
        if self.ego_id is not None and actor_id == self.ego_id:
            fused["ego_vehicle"].append(rec)
        else:
            fused["visible_actors"].append(rec)

# ======= main =======

def main(args=None):
    rclpy.init(args=args)
    node = FusionMeasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

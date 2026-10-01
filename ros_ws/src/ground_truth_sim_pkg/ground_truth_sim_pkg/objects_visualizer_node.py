import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from geometry_msgs.msg import Pose
import json
import transforms3d.euler as euler
import numpy as np
import time
import math

EGO_HUMMER_YAW_OFFSET=180
PEDESTRIAN_MODEL_YAW_OFFSET=90
GT_YAW_OFFSET=90
CAR_012_YAW_OFFSET=90

def unwrap_heading(prev, current):
    """Unwraps heading to avoid flipping."""
    delta = np.arctan2(np.sin(current - prev), np.cos(current - prev))
    return prev + delta

class ObjectsVisualizerNode(Node):
    def __init__(self):
        super().__init__('objects_visualizer_node')

        self.subscription = None
        self.selected_topic = 'infra_actor_states'

        self.prev_heading = {}

        self.topic_selection_sub = self.create_subscription(String, '/selected_topic', self.topic_selection_callback, 10)
        self.subscribe_to_topic(self.selected_topic)
        self.ground_truth_sub = self.create_subscription(String, 'ground_truth_topic', self.gt_listener_callback, 10)
        self.remote_ground_truth_sub = self.create_subscription(String, 'gt_remote_global', self.gt_remote_listener_callback, 10)

        # self.intersection_id_sub = self.create_subscription(String, 'closest_intersection_basic', self.intersection_gmaps_callback, 10)

        # self.subscription_ego = self.create_subscription(String, 'ego_actor_states', lambda msg: self.ego_infra_sq_listener_callback(msg, "ego"), 10)
        # self.subscription_infra = self.create_subscription(String, 'infra_actor_states', lambda msg: self.ego_infra_sq_listener_callback(msg, "infra"), 10)
        self.remote_subscriber = self.create_subscription(String, 'ego_states', self.ego_listener_callback, 10)
        # self.subscription_fused = self.create_subscription(String, 'fused_actor_states', self.listener_callback, 10)
        self.remote_ground_truth_sub = self.create_subscription(String, 'obstacles_id_data', self.ego_obstacles_logger, 10)
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.vehicle_update_counts = {}  # stores update counts for each vehicle
        self.update_threshold = 2  # no of messages before removal

        self.ego_model_path = '/home/connau/ConnAu/hummer/model.sdf'
        self.others_model_path = '/home/connau/ConnAu/car_008/model.sdf'
        self.others_model_path_red = '/home/connau/ConnAu/car_008_red/model.sdf'
        self.pedestrian_model_path = '/home/connau/ConnAu/person_walking/model.sdf'
        self.pedestrian_model_path_green = '/home/connau/ConnAu/person_walking_green/model.sdf'
        self.pedestrian_model_path_red = '/home/connau/ConnAu/person_walking_red/model.sdf'
        self.gmaps_image_path = '/home/connau/ConnAu/models/'

        # self.others_model_path = self.ego_model_path
        self.gt_spawned_objects = set()
        self.spawned_vehicles = set()
        self.current_ego_obstacles = []
        self.ego_id = 'obj_ego_gt'
        self.current_intersection_id = None
        self.current_gmaps_intersection_id = None
        self.current_spawned_gmaps_id = None
        self.is_ego_spawned = False

        # === Trajectory curve (dash) config ===
        self.segments_per_actor = 5        # how many dashes to pre-spawn per actor
        self.dash_len = 1.0                 # meters; visual dash length
        self.dash_width = 0.08              # meters; thin line look
        self.dash_height = 0.06             # meters; a low-profile bar
        self.dash_hide_z = -10.0            # Z to "hide" unused dashes

        # === Removing residual ===
        self.last_data_time = None
        self.timer = self.create_timer(1.0, self.object_update_check_callback)

        # Colors per source (RGBA)
        self.curve_colors = {
            "gps":   (1.0, 0.4, 0.0, 1.0),
            "infra": (0.0, 0.8, 0.0, 1.0),
            "gt":    (0.2, 0.4, 1.0, 1.0),
            "ego": (0.6, 0.6, 0.6, 1.0),
            "other": (0.6, 0.6, 0.6, 1.0)
        }

        # Ego path: {"waypoints":[{"x":..., "y":...}, ...]}
        self.ego_curve_sub = self.create_subscription(
            String, 'ego_waypoints',
            lambda m: self.curve_update_callback(m, self.ego_id if hasattr(self, 'ego_id') else 'obj_ego', 'gps'),
            10
        )

        # Multi-actor paths:
        # {"paths":[{"id":"obj_123","source":"infra","waypoints":[{"x":..,"y":..}, ...]}, ...]}
        self.actors_curve_sub = self.create_subscription(
            String, 'actors_waypoints', self.curve_multi_update_callback, 10
        )

        # actor_id -> [dash_entity_names...]
        self.actor_curve_segments = {}      

        self.i = 0

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SpawnEntity service...')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetEntityState service...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DeleteEntity service...')

        self.current_gmaps_intersection_id = "1001"
        self.spawn_image(0.0, 0.0)

    def object_update_check_callback(self):
        if self.last_data_time:
            if time.monotonic() - self.last_data_time > 1:
                self.residual_trigger_remove_stale_vehicles()

    def ego_obstacles_logger(self, msg):
        data = json.loads(msg.data)
        self.current_ego_obstacles = data['ids']

    def curve_update_callback(self, msg, actor_id, source='other'):
        try:
            data = json.loads(msg.data)
            pts = data.get('waypoints', [])
            self._update_curve_segments(actor_id, pts, source)
        except Exception as e:
            self.get_logger().error(f'curve_update_callback error: {e}')

    def curve_multi_update_callback(self, msg):
        try:
            data = json.loads(msg.data)
            for path in data.get('paths', []):
                actor_id = f"obj_{path.get('id')}"
                source = path.get('source', 'other')
                pts = path.get('waypoints', [])
                if actor_id and pts is not None:
                    self._update_curve_segments(actor_id, pts, source)
        except Exception as e:
            self.get_logger().error(f'curve_multi_update_callback error: {e}')

    def _dash_model_xml(self, name, rgba):
        r, g, b, a = rgba
        # A box aligned with X; we’ll rotate per dash pose
        return f"""
        <sdf version="1.6">
        <model name="{name}">
            <static>true</static>
            <pose>0 0 0 0 0 0</pose>
            <link name="link">
            <visual name="vis">
                <geometry>
                <box>
                    <size>{self.dash_len} {self.dash_width} {self.dash_height}</size>
                </box>
                </geometry>
                <material>
                <ambient>{r} {g} {b} {a}</ambient>
                <diffuse>{r} {g} {b} {a}</diffuse>
                </material>
            </visual>
            <!-- no collision/inertial for cheap rendering -->
            </link>
        </model>
        </sdf>
        """

    def _ensure_segments_spawned(self, actor_id, source='other'):
        segs = self.actor_curve_segments.get(actor_id, [])
        need = self.segments_per_actor - len(segs)
        if need <= 0:
            return

        rgba = self.curve_colors.get(source, self.curve_colors['other'])
        for i in range(len(segs), self.segments_per_actor):
            name = f"seg_{actor_id}_{i}"
            req = SpawnEntity.Request()
            req.name = name
            req.xml = self._dash_model_xml(name, rgba)
            req.robot_namespace = name
            fut = self.spawn_client.call_async(req)
            fut.add_done_callback(self.handle_spawn_response)
            segs.append(name)

        self.actor_curve_segments[actor_id] = segs

    def _yaw_to_quat(self, yaw):
        # Roll=pitch=0, yaw around Z
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        # z axis rotation quaternion
        return (0.0, 0.0, sy, cy)  # x, y, z, w

    def _set_entity_pose(self, name, x, y, z, yaw):
        qx, qy, qz, qw = self._yaw_to_quat(yaw)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        msg = EntityState()
        msg.name = name
        msg.pose = pose
        msg.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = msg
        fut = self.set_state_client.call_async(req)
        fut.add_done_callback(self.handle_set_state_response)

    def _polyline_samples(self, pts, k):
        """
        Resample polyline (list of dicts {'x','y'}) into k evenly spaced samples
        along arc length. Returns list of (x,y,yaw) with yaw tangent to curve.
        """
        if not pts or k <= 0:
            return []

        xs = [float(p['x']) for p in pts]
        ys = [float(p['y']) for p in pts]
        n = len(xs)
        if n == 1:
            return [(xs[0], ys[0], 0.0)] * k

        # cumulative arc length
        d = [0.0]
        for i in range(1, n):
            d.append(d[-1] + math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]))
        total = d[-1]
        if total < 1e-6:
            return [(xs[0], ys[0], 0.0)] * k

        targets = [i * total / max(k-1, 1) for i in range(k)]
        out = []
        j = 1
        for t in targets:
            while j < n and d[j] < t:
                j += 1
            j = min(j, n-1)
            # interpolate between j-1 and j
            t0, t1 = d[j-1], d[j]
            if t1 - t0 < 1e-9:
                x = xs[j]
                y = ys[j]
            else:
                u = (t - t0) / (t1 - t0)
                x = xs[j-1] + u * (xs[j] - xs[j-1])
                y = ys[j-1] + u * (ys[j] - ys[j-1])

            # yaw from local tangent
            dx = xs[j] - xs[j-1]
            dy = ys[j] - ys[j-1]
            yaw = math.atan2(dy, dx) if (abs(dx) + abs(dy)) > 1e-9 else 0.0
            out.append((x, y, yaw))
        return out

    def _update_curve_segments(self, actor_id, pts, source='other'):
        """
        Ensure segments exist, then place up to N dashes along the polyline.
        Unused dashes are hidden (moved underground).
        """
        # Normalize input
        try:
            pts = [{'x': float(p['x']), 'y': float(p['y'])} for p in pts]
        except Exception:
            self.get_logger().warn(f"Bad pts for {actor_id}; skipping curve update")
            return

        # Spawn (once)
        self._ensure_segments_spawned(actor_id, source)

        # Compute sample poses (≤ N)
        k = min(self.segments_per_actor, max(1, len(pts)))
        samples = self._polyline_samples(pts, k)

        # Center z so the dash just rests on ground
        z = 0.5 + self.dash_height / 2.0

        segs = self.actor_curve_segments.get(actor_id, [])
        # Place active dashes
        for i in range(k):
            name = segs[i]
            x, y, yaw = samples[i]
            self._set_entity_pose(name, x, y, z, yaw)

        # Hide leftovers
        for i in range(k, len(segs)):
            name = segs[i]
            self._set_entity_pose(name, 0.0, 0.0, self.dash_hide_z, 0.0)

    def topic_selection_callback(self, msg):
        new_topic = msg.data
        if new_topic != self.selected_topic:
            self.get_logger().info(f"Switching to topic: {new_topic}")
            self.selected_topic = new_topic
            self.subscribe_to_topic(self.selected_topic)

    def subscribe_to_topic(self, topic_name):
        if self.subscription:
            self.destroy_subscription(self.subscription)

        self.subscription = self.create_subscription(
            String, topic_name, self.listener_callback, 10
        )
        self.get_logger().info(f"Subscribed to {topic_name}")

    def gt_remote_listener_callback(self, msg):
        try:
            obj = json.loads(msg.data)
            # ego_gt = data['ego_vehicle']
            # if ego_gt is None:
            #     return
            # ego_gt['pos'] = {'x': ego_gt['pos']['lat'], 'y': ego_gt['pos']['lon']}
            # if ego_gt['uniqueId'] not in self.spawned_vehicles:
            #     self.gt_spawn_object(ego_gt['uniqueId'], ego_gt['pos'], np.radians(ego_gt['heading']['theta']+GT_YAW_OFFSET), "gt")
            # else:
            #     self.gt_update_object(ego_gt['uniqueId'], ego_gt['pos'], np.radians(ego_gt['heading']['theta']+GT_YAW_OFFSET))
            if obj is None:
                return
            obj_id = obj['uniqueId']
            obj_type = obj['class']
            position = obj['pos']
            position = {'x': position['lat'], 'y': position['lon']}
            heading = np.radians(-obj['heading']+GT_YAW_OFFSET)

            # Check if the object has already been spawned
            if obj_id not in self.spawned_vehicles:
                self.gt_spawn_object(obj_id, position, heading, "gt", obj_type)
            else:
                self.gt_update_object(obj_id, position, heading)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except KeyError as e:
            self.get_logger().error(f"Mis key in JSON data: {e}")

    def gt_listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            objects = data['remote_actors']
            # ego_gt = data['ego_vehicle']
            # if ego_gt is None:
            #     return
            # ego_gt['pos'] = {'x': ego_gt['pos']['lat'], 'y': ego_gt['pos']['lon']}
            # if ego_gt['uniqueId'] not in self.spawned_vehicles:
            #     self.gt_spawn_object(ego_gt['uniqueId'], ego_gt['pos'], np.radians(ego_gt['heading']['theta']+GT_YAW_OFFSET), "gt")
            # else:
            #     self.gt_update_object(ego_gt['uniqueId'], ego_gt['pos'], np.radians(ego_gt['heading']['theta']+GT_YAW_OFFSET))
            if objects is None:
                return
            for obj in objects:
                obj_id = obj['uniqueId']
                obj_type = obj['class']
                position = obj['pos']
                position = {'x': position['lat'], 'y': position['lon']}
                heading = np.radians(-obj['heading']['theta']+GT_YAW_OFFSET)

                # Check if the object has already been spawned
                if obj_id not in self.spawned_vehicles:
                    self.gt_spawn_object(obj_id, position, heading, "gt", obj_type)
                else:
                    self.gt_update_object(obj_id, position, heading)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except KeyError as e:
            self.get_logger().error(f"Mis key in JSON data: {e}")


    def gt_spawn_object(self, obj_id, pos, heading, mode, obj_type):
        request = SpawnEntity.Request()
        mode_color_code = [0, 0, 1]
        if mode == "ego":
            mode_color_code = [1, 0, 0]
        elif mode == "infra":
            mode_color_code = [0, 1, 0]
        obj_size = [5.5, 2.15]
        if obj_type == 'vru':
            obj_size = [1.0, 1.0]

        request.name = obj_id
        request.xml = f"""
        <sdf version="1.6">
            <model name="{obj_id}">
                <pose>{pos['x']} {pos['y']} 0.45 0 0 {heading}</pose>
                <static>false</static>
                <link name="link">
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>{obj_size[0]} {obj_size[1]} 0.01</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>{mode_color_code[0]} {mode_color_code[1]} {mode_color_code[2]} 1</ambient>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        """
        request.robot_namespace = obj_id
        request.initial_pose.position.x = pos['x']
        request.initial_pose.position.y = pos['y']
        request.initial_pose.position.z = 0.5
        # request.initial_pose.orientation.z = heading

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.handle_spawn_response)
        self.spawned_vehicles.add(obj_id)

    def ego_infra_sq_listener_callback(self, msg, mode):
        if self.selected_topic != "fused_actor_states":
            return
        try:
            data = json.loads(msg.data)
            vehicles = []
            for member in data['ego_vehicle']:
                vehicles.append(member)
                # self.ego_id = "obj_" + member['uniqueId']

            if len(data['visible_actors']) > 0:
                for member in data['visible_actors']:
                    vehicles.append(member)

            for obj in vehicles:
                obj_id = "obj_" + obj['uniqueId'] + f"_{mode}"
                self.vehicle_update_counts[obj_id] = 0
                position = obj['estimated_position']
                heading = obj['estimated_heading']

                # Check if the object has already been spawned
                if obj_id not in self.spawned_vehicles:
                    self.gt_spawn_object(obj_id, position, heading, mode)
                else:
                    self.gt_update_object(obj_id, position, heading)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except KeyError as e:
            self.get_logger().error(f"Missing key in JSON data: {e}")

    def intersection_spawn_square(self, pos_x, pos_y):
        request = SpawnEntity.Request()

        obj_id = f"inter_{pos_x}_{pos_y}"
        request.name = obj_id
        request.xml = f"""
        <sdf version="1.6">
            <model name="{obj_id}">
                <pose>{pos_x} {-pos_y} 0.1 0 0 0</pose>
                <static>true</static>
                <link name="link">
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>80 80 0.01</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0 1 0 1</ambient>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        """
        request.robot_namespace = obj_id
        # request.initial_pose.position.x = pos_x
        # request.initial_pose.position.y = -pos_y
        # request.initial_pose.orientation.z = heading

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.handle_spawn_response)
        self.gt_spawned_objects.add(obj_id)
        self.current_intersection_id = obj_id

    def gt_update_object(self, obj_id, pos, heading):
        # self.get_logger().info("gt_spawn_getting called")
        heading1 = heading
        if obj_id not in self.prev_heading:
            self.prev_heading[obj_id] = heading
        heading = unwrap_heading(self.prev_heading[obj_id], heading)
        self.prev_heading[obj_id] = heading
        # self.get_logger().info(f"hb: {heading1*180/np.pi}, ha: {heading*180/np.pi}")
        state_msg = EntityState()
        state_msg.name = obj_id
        state_msg.pose.position.x = pos['x']
        state_msg.pose.position.y = pos['y']
        state_msg.pose.position.z = 0.5
        qw, qx, qy, qz = euler.euler2quat(0, 0, heading)
        state_msg.pose.orientation.x = qx
        state_msg.pose.orientation.y = qy
        state_msg.pose.orientation.z = qz
        state_msg.pose.orientation.w = qw
        state_msg.reference_frame = 'world'

        request = SetEntityState.Request()
        request.state = state_msg

        future = self.set_state_client.call_async(request)
        future.add_done_callback(self.handle_set_state_response)

    def spawn_image(self, x, y):
        # Create the request
        # self.get_logger().info("here-spawn image1")
        request = SpawnEntity.Request()
        image_path = self.gmaps_image_path + "gmaps_" + self.current_gmaps_intersection_id + "/model.sdf"
        request.xml = open(image_path, 'r').read()
        # qw, qx, qy, qz = euler.euler2quat(0, 0, -yaw)
        request.robot_namespace = "gmaps_" + self.current_gmaps_intersection_id
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.4  # Adjust if necessary
        # request.initial_pose.orientation.x = qx
        # request.initial_pose.orientation.y = qy
        # request.initial_pose.orientation.z = qz
        # request.initial_pose.orientation.w = qw

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.handle_spawn_response)
        # self.get_logger().info("here-spawn image2")

        # image_path = "/home/connau/Documents/gazebo_models/cricket_ball"+ "/model.sdf"

        # request.xml = open(image_path, 'r').read()
        # qw, qx, qy, qz = euler.euler2quat(0, 0, -yaw)
        # request.robot_namespace = "cball"
        # request.initial_pose.position.x = x
        # request.initial_pose.position.y = y
        # request.initial_pose.position.z = 0.4  # Adjust if necessary
        # request.initial_pose.orientation.x = qx
        # request.initial_pose.orientation.y = qy
        # request.initial_pose.orientation.z = qz
        # request.initial_pose.orientation.w = qw

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.handle_spawn_response)

    def intersection_gmaps_callback(self, msg):
        data = json.loads(msg.data)
        self.current_gmaps_intersection_id = data['id']
        self.spawn_image(data['pos']['x'], data['pos']['y'])

    def ego_listener_callback(self, msg):
        data = json.loads(msg.data)
        vehicle = data['ego_vehicle']
        vehicle_id = "obj_" + vehicle['uniqueId']
        actor_type = vehicle['class']
        x = vehicle['pos']['lat']
        y = vehicle['pos']['lon']
        yaw = np.radians(vehicle['heading']['theta']+EGO_HUMMER_YAW_OFFSET)

        if vehicle_id not in self.prev_heading:
            self.prev_heading[vehicle_id] = yaw
        yaw = unwrap_heading(self.prev_heading[vehicle_id], yaw)
        self.prev_heading[vehicle_id] = yaw
        if not self.is_ego_spawned:
            self.spawn_vehicle(vehicle_id, x, y, yaw, actor_type)
            self.spawned_vehicles.add(vehicle_id)
            self.is_ego_spawned = True
        else:
            qw, qx, qy, qz = euler.euler2quat(0, 0, yaw)
            state_msg = EntityState()
            state_msg.name = str(vehicle_id)  # ensure it's a string
            state_msg.pose.position.x = x
            state_msg.pose.position.y = y
            state_msg.pose.position.z = 0.5 
            state_msg.pose.orientation.x = qx
            state_msg.pose.orientation.y = qy
            state_msg.pose.orientation.z = qz
            state_msg.pose.orientation.w = qw
            state_msg.reference_frame = 'world'
            request = SetEntityState.Request()
            request.state = state_msg
            future = self.set_state_client.call_async(request)
            future.add_done_callback(self.handle_set_state_response)

            request = SetEntityState.Request()
            request.state = state_msg
            future = self.set_state_client.call_async(request)
            future.add_done_callback(self.handle_set_state_response)

    def listener_callback(self, msg):
        self.last_data_time = time.monotonic()
        for vehicle_id in self.vehicle_update_counts.keys():
            self.vehicle_update_counts[vehicle_id] += 1
        # self.get_logger().info("callback started")
        try:
            data = json.loads(msg.data)
            # self.get_logger().info("message from estimate-ego node " + msg.data)
            vehicles = []

            if self.selected_topic == 'infra_actor_states' or self.selected_topic == "fused_actor_states":
                if data['refPos'] is not None:
                    pos_x = data['refPos']['x']
                    pos_y = data['refPos']['y']
                    if self.current_intersection_id != f'inter_{pos_x}_{pos_y}':
                        if self.current_intersection_id is not None:
                            self.delete_vehicle(self.current_intersection_id)
                        self.intersection_spawn_square(pos_x, pos_y)
            else:
                if self.current_intersection_id is not None:
                    self.delete_vehicle(self.current_intersection_id)
                    self.current_intersection_id = None

            # for member in data['ego_vehicle']:
            #     vehicles.append(member)
                # self.ego_id = member['uniqueId']

            if len(data['visible_actors']) > 0:
                for member in data['visible_actors']:
                    vehicles.append(member)
            
            updated_vehicle_ids = []

            for vehicle in vehicles:
                vehicle_id = "obj_" + vehicle['uniqueId']
                actor_type = vehicle['class']
                x = vehicle['estimated_position']['x']
                y = vehicle['estimated_position']['y']
                # v_x = vehicle['estimated_velocity']['v_x']
                # v_y = vehicle['estimated_velocity']['v_y']
                # yaw = np.atan(v_y/v_x)
                yaw = vehicle['estimated_heading']
                # qz = vehicle['estimated_quat']['qz']
                # qw = vehicle['estimated_quat']['qw']
                # qx = 0
                # qy = 0

                if actor_type == "vru":
                    yaw += PEDESTRIAN_MODEL_YAW_OFFSET

                updated_vehicle_ids.append(vehicle_id)
                self.vehicle_update_counts[vehicle_id] = 0
                # if actor_type == "vru":
                #     self.vehicle_update_counts[f'{vehicle_id}_red'] = 0
                self.vehicle_update_counts[f'{vehicle_id}_red'] = 0

                if vehicle_id not in self.prev_heading:
                    self.prev_heading[vehicle_id] = yaw
                yaw = unwrap_heading(self.prev_heading[vehicle_id], yaw)
                self.prev_heading[vehicle_id] = yaw
                
                if vehicle_id not in self.spawned_vehicles:
                    self.spawn_vehicle(vehicle_id, x, y, yaw, actor_type)
                    self.spawned_vehicles.add(vehicle_id)
                    # if actor_type == "vru":
                    #     self.spawned_vehicles.add(f"{vehicle_id}_red")
                    self.spawned_vehicles.add(f"{vehicle_id}_red")
                else:
                    qw, qx, qy, qz = euler.euler2quat(0, 0, yaw)
                    if str(vehicle['uniqueId']) in self.current_ego_obstacles:
                        self.get_logger().info(f"obj obstacles are: {str(vehicle['uniqueId'])}, {str(vehicle_id)}")
                        
                        state_msg = EntityState()
                        state_msg.name = str(vehicle_id)  # Ensure it's a string
                        state_msg.pose.position.x = 0.0
                        state_msg.pose.position.y = 200.0
                        state_msg.pose.position.z = 0.5  # Adjust if necessary
                        state_msg.pose.orientation.x = qx
                        state_msg.pose.orientation.y = qy
                        state_msg.pose.orientation.z = qz
                        state_msg.pose.orientation.w = qw
                        state_msg.reference_frame = 'world'
                        vehicle_id = str(vehicle_id) + "_red"
                        request = SetEntityState.Request()
                        request.state = state_msg
                        future = self.set_state_client.call_async(request)
                        future.add_done_callback(self.handle_set_state_response)
                    state_msg = EntityState()
                    state_msg.name = str(vehicle_id)  # Ensure it's a string
                    state_msg.pose.position.x = x
                    state_msg.pose.position.y = y
                    state_msg.pose.position.z = 0.5  # Adjust if necessary
                    state_msg.pose.orientation.x = qx
                    state_msg.pose.orientation.y = qy
                    state_msg.pose.orientation.z = qz
                    state_msg.pose.orientation.w = qw
                    state_msg.reference_frame = 'world'
                    request = SetEntityState.Request()
                    request.state = state_msg
                    future = self.set_state_client.call_async(request)
                    future.add_done_callback(self.handle_set_state_response)
                    if str(vehicle['uniqueId']) not in self.current_ego_obstacles:
                        vehicle_id = str(vehicle_id) + "_red"
                        state_msg = EntityState()
                        state_msg.name = str(vehicle_id)  # Ensure it's a string
                        state_msg.pose.position.x = 0.0
                        state_msg.pose.position.y = 200.0
                        state_msg.pose.position.z = 0.5  # Adjust if necessary
                        state_msg.pose.orientation.x = qx
                        state_msg.pose.orientation.y = qy
                        state_msg.pose.orientation.z = qz
                        state_msg.pose.orientation.w = qw
                        state_msg.reference_frame = 'world'
                        request = SetEntityState.Request()
                        request.state = state_msg
                        future = self.set_state_client.call_async(request)
                        future.add_done_callback(self.handle_set_state_response)

                    request = SetEntityState.Request()
                    request.state = state_msg
                    future = self.set_state_client.call_async(request)
                    future.add_done_callback(self.handle_set_state_response)
            self.remove_stale_vehicles()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
        except KeyError as e:
            self.get_logger().error(f'Missing key in JSON data: {e}')
        
    def spawn_vehicle(self, vehicle_id, x, y, yaw, actor_type):
        # self.get_logger().info("we are in the beginning of spawning")
        request = SpawnEntity.Request()
        request.name = vehicle_id
        if actor_type == "vru":
            # request.xml = open(self.pedestrian_model_path, 'r').read()
            request.xml = open(self.pedestrian_model_path_green, 'r').read()
        else:
            if vehicle_id == self.ego_id:
                request.xml = open(self.ego_model_path, 'r').read()
            else:
                request.xml = open(self.others_model_path, 'r').read()
        # self.get_logger().info("read is complete")
        qw, qx, qy, qz = euler.euler2quat(0, 0, yaw)
        request.initial_pose.orientation.x = qx
        request.initial_pose.orientation.y = qy
        request.initial_pose.orientation.z = qz
        request.initial_pose.orientation.w = qw
        request.robot_namespace = vehicle_id
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.5
        request.reference_frame = 'world'

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.handle_spawn_response)

        if actor_type == "vru":
            request2 = SpawnEntity.Request()
            request2.name = vehicle_id + "_red"
            request2.xml = open(self.pedestrian_model_path_red, 'r').read()
            qw, qx, qy, qz = euler.euler2quat(0, 0, yaw)
            request2.robot_namespace = vehicle_id
            request2.initial_pose.position.x = 0.0
            request2.initial_pose.position.y = 200.0
            request2.initial_pose.position.z = -2.0
            request2.reference_frame = 'world'

            future = self.spawn_client.call_async(request2)
            future.add_done_callback(self.handle_spawn_response)

        elif vehicle_id != self.ego_id:
            request2 = SpawnEntity.Request()
            request2.name = vehicle_id + "_red"
            request2.xml = open(self.others_model_path_red, 'r').read()
            qw, qx, qy, qz = euler.euler2quat(0, 0, yaw)
            request2.robot_namespace = vehicle_id
            request2.initial_pose.position.x = 0.0
            request2.initial_pose.position.y = 200.0
            request2.initial_pose.position.z = -2.0
            request2.reference_frame = 'world'

            future = self.spawn_client.call_async(request2)
            future.add_done_callback(self.handle_spawn_response)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        # self.get_logger().info("future_complete is done")
        # if future.result() is not None:
        #     self.get_logger().info(f'spawned new vehicle: {vehicle_id}')
        # else:
        #     self.get_logger().error(f'failed to spawn vehicle {vehicle_id}: {future.exception()}')

    def handle_spawn_response(self, future):
        if future.result() is not None:
            # self.get_logger().info(f'spawned new vehicle')
            pass
        else:
            self.get_logger().error(f'failed to spawn vehicle: {future.exception()}')

    def handle_set_state_response(self, future):
        if future.result() is not None:
            # self.get_logger().info(f'changed vehicle position')
            pass
        else:
            self.get_logger().error(f'failed to change vehicle position: {future.exception()}')

    def remove_stale_vehicles(self):
        vehicles_to_remove = [vehicle_id for vehicle_id, count in self.vehicle_update_counts.items()
                              if count >= self.update_threshold]

        for vehicle_id in vehicles_to_remove:
            state_msg = EntityState()
            state_msg.name = str(vehicle_id)  # Ensure it's a string
            state_msg.pose.position.x = 0.0
            state_msg.pose.position.y = 210.0
            state_msg.pose.position.z = 0.5  # Adjust if necessary
            state_msg.pose.orientation.x = 0.0
            state_msg.pose.orientation.y = 0.0
            state_msg.pose.orientation.z = 0.0
            state_msg.pose.orientation.w = 1.0
            state_msg.reference_frame = 'world'
            request = SetEntityState.Request()
            request.state = state_msg
            future = self.set_state_client.call_async(request)
            future.add_done_callback(self.handle_set_state_response)
            self.delete_vehicle(vehicle_id)
            # if not vehicle_id.endswith("_red"):
            #     self._delete_segments(vehicle_id)
            self.spawned_vehicles.remove(vehicle_id)
            del self.vehicle_update_counts[vehicle_id]
            # self.get_logger().info(f'removed stale vehicle: {vehicle_id}')

    def residual_trigger_remove_stale_vehicles(self):
        vehicles_to_remove = [vehicle_id for vehicle_id in self.vehicle_update_counts]
        for vehicle_id in vehicles_to_remove:
            state_msg = EntityState()
            state_msg.name = str(vehicle_id)  # Ensure it's a string
            state_msg.pose.position.x = 0.0
            state_msg.pose.position.y = 210.0
            state_msg.pose.position.z = 0.5  # Adjust if necessary
            state_msg.pose.orientation.x = 0.0
            state_msg.pose.orientation.y = 0.0
            state_msg.pose.orientation.z = 0.0
            state_msg.pose.orientation.w = 1.0
            state_msg.reference_frame = 'world'
            request = SetEntityState.Request()
            request.state = state_msg
            future = self.set_state_client.call_async(request)
            future.add_done_callback(self.handle_set_state_response)
            self.delete_vehicle(vehicle_id)
            # if not vehicle_id.endswith("_red"):
            #     self._delete_segments(vehicle_id)
            self.spawned_vehicles.remove(vehicle_id)
            del self.vehicle_update_counts[vehicle_id]

    # def delete_vehicle(self, vehicle_id):
    #     request = DeleteEntity.Request()
    #     request.name = vehicle_id
    #     future = self.delete_client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() is not None:
    #         self.get_logger().info(f'deleted vehicle: {vehicle_id}')
    #     else:
    #         self.get_logger().error(f'failed to delete vehicle {vehicle_id}: {future.exception()}')

    def delete_vehicle(self, vehicle_id):
        # self.get_logger().info(f"deleting vehicle {vehicle_id}")

        request = DeleteEntity.Request()
        request.name = vehicle_id

        future = self.delete_client.call_async(request)

        future.add_done_callback(lambda f: self.handle_delete_response(f, vehicle_id))

    def handle_delete_response(self, future, vehicle_id):
        if future.result() is not None:
            if vehicle_id in self.spawned_vehicles:
                self.spawned_vehicles.remove(vehicle_id)
            if vehicle_id in self.vehicle_update_counts:
                del self.vehicle_update_counts[vehicle_id]
        else:
            self.get_logger().error(f"failed to delete vehicle {vehicle_id}: {future.exception()}")

    def handle_wps_delete_response(self, future, vehicle_id):
        pass

    def _delete_segments(self, actor_id):
        segs = self.actor_curve_segments.pop(actor_id, [])
        self.get_logger().info(f"here deleting {actor_id}")
        for name in segs:
            self.get_logger().info(name)
            req = DeleteEntity.Request()
            req.name = name
            future = self.delete_client.call_async(req)
            future.add_done_callback(lambda f: self.handle_wps_delete_response(f, name))

def main(args=None):
    rclpy.init(args=args)
    position_subscriber = ObjectsVisualizerNode()
    rclpy.spin(position_subscriber)
    position_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import carla
import rclpy
from rclpy.node import Node
import numpy as np
import json
from std_msgs.msg import String
import random
import math
from rosgraph_msgs.msg import Clock
import time

class GroundTruthNode(Node):
    def __init__(self):
        super().__init__('ground_truth_node')

        self.gt_publisher_ = self.create_publisher(String, 'ground_truth_topic1', 10)

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.load_world('Town05')
        self.blueprint_library = self.world.get_blueprint_library()

        self.loop_count = 0
        self.is_autopilot_on = False

        self.scenario2()

        # vehicle_bp_all = self.blueprint_library.filter('vehicle.*')
        # spawn_points = self.world.get_map().get_spawn_points()
        # random.shuffle(spawn_points)

        # self.other_vehicles = []
        # for i in range(10):
        #     other_vehicle = random.choice(vehicle_bp_all)
        #     if other_vehicle == vehicle_bp:
        #         continue

        #     if i < len(spawn_points):
        #         spawn_point = spawn_points[i]
        #     else:
        #         spawn_point = random.choice(spawn_points)

        #     vehicle_spawn = self.world.try_spawn_actor(other_vehicle, spawn_point)
        #     if vehicle_spawn:
        #         self.other_vehicles.append(vehicle_spawn)
        #         vehicle_spawn.set_autopilot(True)

        

        self.timer = self.create_timer(0.2, self.update_and_publish_data)
        #self.clock_call_sub = self.create_subscription(Clock, "clock", self.wrapper ,50)
        # self.world.on_tick(self.update_and_publish_data)

    def scenario1(self):
        vehicle_bp = self.blueprint_library.filter('vehicle.chevrolet.impala')[0]
        spawn_point = self.world.get_map().get_spawn_points()[21]
        spawn_point.location.x -= 0
        spawn_point.location.y += 3.5
        spawn_point.rotation.yaw = 180
        self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        self.ego_vehicle.set_autopilot(True)
        transform = self.ego_vehicle.get_transform()
        # "x": -29.740592832937928, "y": -4.4275463201497764
        spectator = self.world.get_spectator()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(x=-45,z=50),carla.Rotation(pitch=-90, yaw=180)))

        vehicle_firetruck_bp = self.blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
        spawn_point = self.world.get_map().get_spawn_points()[21]
        spawn_point.location.x -= 12
        self.vehicle = self.world.spawn_actor(vehicle_firetruck_bp, spawn_point)
        self.vehicle.set_autopilot(True)

    def scenario2(self):
        vehicle_bp = self.blueprint_library.filter('vehicle.chevrolet.impala')[0]
        spawn_point = self.world.get_map().get_spawn_points()[21]
        spawn_point.location.x -= 47
        spawn_point.location.y += 10.5
        spawn_point.rotation.yaw = 0
        self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        # self.ego_vehicle.set_autopilot(True)
        transform = self.ego_vehicle.get_transform()
        # "x": -29.740592832937928, "y": -4.4275463201497764
        spectator = self.world.get_spectator()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(x=-45,z=50),carla.Rotation(pitch=-90, yaw=0)))

        vehicle_firetruck_bp = self.blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
        spawn_point = self.world.get_map().get_spawn_points()[21]
        spawn_point.location.x -= 12
        self.vehicle1 = self.world.spawn_actor(vehicle_firetruck_bp, spawn_point)
        # self.vehicle1.set_autopilot(True)

        vehicle_police_bp = self.blueprint_library.filter('vehicle.dodge.charger_police')[0]
        spawn_point = self.world.get_map().get_spawn_points()[21]
        spawn_point.location.x -= 37
        spawn_point.location.y -= 25
        spawn_point.rotation.yaw = 90
        self.vehicle2 = self.world.spawn_actor(vehicle_police_bp, spawn_point)
        # self.vehicle2.set_autopilot(True)

        time.sleep(5)

        # blueprintsWalkers = self.world.get_blueprint_library().filter("walker.pedestrian.*")
        # walker_bp = random.choice(blueprintsWalkers)
        # spawn_point = self.world.get_map().get_spawn_points()[21]
        # spawn_point.location.x -= 41
        # spawn_point.location.y += 15
        # spawn_point.location.z += 0.5
        # spawn_point.rotation.yaw = 270
        #self.walker = self.world.spawn_actor(walker_bp, spawn_point)

        # walker_bp = random.choice(blueprintsWalkers)
        # spawn_point = self.world.get_map().get_spawn_points()[21]
        # spawn_point.location.x -= 23.25
        # spawn_point.location.y += 16
        # spawn_point.location.z += 0.5
        # spawn_point.rotation.yaw = 180
        # self.walker = self.world.spawn_actor(walker_bp, spawn_point)

    def wrapper(self, args):
        self.update_and_publish_data()

    def update_and_publish_data(self):

        # self.loop_count += 1
        # if self.loop_count > 40:
        #     if not self.is_autopilot_on:
        #         self.ego_vehicle.set_autopilot(True)
        #         self.vehicle1.set_autopilot(True)
        #         self.vehicle2.set_autopilot(True)
        #         self.is_autopilot_on = True

        timestamp = self.world.get_snapshot().timestamp.elapsed_seconds

        if self.ego_vehicle is None:
            # self.get_logger().info("Ego vehicle not set.")
            return

        actors = self.world.get_actors()
        gt_ego = None
        gt_actors = []

        for actor in actors:
            if (actor.type_id.startswith("vehicle") or actor.type_id.startswith("walker")):
                actor_location = actor.get_transform().location
                actor_heading = actor.get_transform().rotation.yaw
                actor_velocity = actor.get_velocity()
                actor_class = "vehicle" if actor.type_id.startswith("vehicle") else "vru"
                gt_actor_data = {
                    "uniqueId": actor.id,
                    "class": actor_class,
                    "pos": {
                        "x": actor_location.x,
                        "y": actor_location.y
                    },
                    "heading": actor_heading,
                    "vel": {
                        "vx": actor_velocity.x,
                        "vy": actor_velocity.y
                    },
                    "halfDim": {
                        "halfLen": actor.bounding_box.extent.x,
                        "halfBre": actor.bounding_box.extent.y,
                        "halfHei": actor.bounding_box.extent.z
                    }
                }
                if actor.id == self.ego_vehicle.id:
                    gt_ego = gt_actor_data
                gt_actors.append(gt_actor_data)

        gt_json_data = {
            "timestamp": timestamp,
            "ego_id": self.ego_vehicle.id,
            "ego_vehicle": gt_ego,
            "all_actors": gt_actors
        }

        json_gt_msg = String()
        json_gt_msg.data = json.dumps(gt_json_data)
        self.gt_publisher_.publish(json_gt_msg)
        self.get_logger().info(f"Published ground truth data: {json_gt_msg.data}")
        # self.get_logger().info(f"{self.ego_vehicle.id}")

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
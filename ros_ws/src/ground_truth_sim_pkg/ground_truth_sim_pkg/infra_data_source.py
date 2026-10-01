import carla
import rclpy
from rclpy.node import Node
import numpy as np
import json
import time
from std_msgs.msg import String

from fusion.msg import Track

class InfraDataSource(Node):
    def __init__(self):
        super().__init__('infra_data_source')

        self.publisher_ = self.create_publisher(String, 'infra_local_measures', 10)
        self.ground_truth_sub = self.create_subscription(String, 'ground_truth_topic1', self.gt_listener_callback, 10)
        # self.track_publisher_ = self.create_publisher(Track, "track", 100)
        self.subscriber_closest_intersection = self.create_subscription(String, 'closest_intersection', self.closest_intersection_callback, 10)

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.ref_pos = {"x": -49.0, "y": 1.0} # default intersection
        self.halfBoundLength = 40.0

        self.bounding_box_x = [self.ref_pos["x"]-self.halfBoundLength, self.ref_pos["x"]+self.halfBoundLength]  # X range (meters)
        self.bounding_box_y = [self.ref_pos["y"]-self.halfBoundLength, self.ref_pos["y"]+self.halfBoundLength]  # Y range (meters)

        self.debug_helper = self.world.debug

        self.draw_bounding_box()

        self.std_dev_position_infra = 0.3
        self.std_dev_velocity_infra = 0.04
        self.std_dev_heading_infra = 0.008  # in radians

        self.tracks = {}
        self.track_ids = 0

        # self.timer = self.create_timer(0.1, self.update_and_publish_data)

    def draw_bounding_box(self):
        z_height = 1.0
        ref_x = self.ref_pos["x"]
        ref_y = self.ref_pos["y"]
        corner1 = carla.Location(x=self.bounding_box_x[0], y=self.bounding_box_y[0], z=z_height)
        corner2 = carla.Location(x=self.bounding_box_x[1], y=self.bounding_box_y[0], z=z_height)
        corner3 = carla.Location(x=self.bounding_box_x[1], y=self.bounding_box_y[1], z=z_height)
        corner4 = carla.Location(x=self.bounding_box_x[0], y=self.bounding_box_y[1], z=z_height)

        self.debug_helper.draw_line(corner1, corner2, thickness=0.2, color=carla.Color(255, 0, 0), life_time=0.2)
        self.debug_helper.draw_line(corner2, corner3, thickness=0.2, color=carla.Color(255, 0, 0), life_time=0.2)
        self.debug_helper.draw_line(corner3, corner4, thickness=0.2, color=carla.Color(255, 0, 0), life_time=0.2)
        self.debug_helper.draw_line(corner4, corner1, thickness=0.2, color=carla.Color(255, 0, 0), life_time=0.2)

    def closest_intersection_callback(self, msg):
        data = json.loads(msg.data)
        if data == None:
            self.ref_pos = None
            self.halfBoundLength = 0
        elif data["id"]["id"] < 0:
            self.ref_pos = data['refPoint']['carla_xy']
            # self.get_logger().info(f"infra refPos is {self.ref_pos}")
            self.halfBoundLength = data["halfBoundLength"]
            # bounding box dimensions relative to refPos
            self.bounding_box_x = [self.ref_pos["x"]-self.halfBoundLength, self.ref_pos["x"]+self.halfBoundLength]  # X range (m)
            self.bounding_box_y = [self.ref_pos["y"]-self.halfBoundLength, self.ref_pos["y"]+self.halfBoundLength]  # Y range (m)

            self.debug_helper = self.world.debug
            self.draw_bounding_box()

    def gt_listener_callback(self, msg):
        data = json.loads(msg.data)
        timestamp = data['timestamp']
        actors = data['all_actors']

        # JSON message structure
        json_data = {
            "timestamp": timestamp,
            "refPos": self.ref_pos,
            "sdsmData": []
        }

        for actor in actors:
            # Get the actor's current transform (position, heading)
            transform = actor['pos']
            velocity = actor['vel']
            heading = actor['heading']

            # Compute offset relative to refPos
            offset_x = transform['x']- self.ref_pos["x"]
            offset_y = -transform['y'] - self.ref_pos["y"]

            # check if the actor is within the bounding box
            if self.bounding_box_x[0] <= transform['x'] <= self.bounding_box_x[1] and \
                self.bounding_box_y[0] <= transform['y'] <= self.bounding_box_y[1]:
                
                # self.get_logger().info(actor.type_id)

                actor_class = actor['class']

                # Add Gaussian noise to simulate measurement errors
                noisy_offset_x = self.add_gaussian_noise(offset_x, self.std_dev_position_infra)
                noisy_offset_y = self.add_gaussian_noise(offset_y, self.std_dev_position_infra)
                noisy_speed = np.abs(self.add_gaussian_noise(np.linalg.norm([velocity['vx'], velocity['vy']]), self.std_dev_velocity_infra))
                noisy_heading = self.add_gaussian_noise(heading, self.std_dev_heading_infra)

                actor_data = {
                    "class": actor_class,
                    "uniqueId": actor['uniqueId'],
                    "pos": {
                        "x": noisy_offset_x,
                        "y": noisy_offset_y
                    },
                    "posStdDev": self.std_dev_position_infra,
                    "speed": noisy_speed,
                    "speedStdDev": self.std_dev_velocity_infra,
                    "heading": -noisy_heading,
                    "headingStdDev": self.std_dev_heading_infra,
                    "halfDim": actor['halfDim']
                }

                json_data["sdsmData"].append(actor_data)

        json_msg = String()
        json_msg.data = json.dumps(json_data)
        # self.get_logger().info(f"Infrastructure 1 : {json_data}")
        self.publisher_.publish(json_msg)

    def add_gaussian_noise(self, value, std_dev):
        return value + np.random.normal(0, std_dev)


def main(args=None):
    rclpy.init(args=args)
    node = InfraDataSource()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
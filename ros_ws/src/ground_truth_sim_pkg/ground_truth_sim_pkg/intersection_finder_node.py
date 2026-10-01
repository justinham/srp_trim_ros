import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
import numpy as np
from fusion.msg import GPS

# haversine formula to calculate distance between two lat/lng points
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # radius of the earth in km
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c  # distance in m

# load local JSON database
def load_database(json_file):
    with open(json_file, 'r') as f:
        return json.load(f)

# ROS2 node to publish closest intersection id
class IntersectionFinderNode(Node):
    def __init__(self):
        super().__init__('intersection_finder_node')
        # self.subscriber_ = self.create_subscription(String, 'ego_pos', self.timer_callback, 10)
        self.publisher_ = self.create_publisher(String, 'closest_intersection_id', 10)
        self.gps_subscriber = self.create_subscription(GPS, 'gps', self.gps_callback, 10)
        # self.timer = self.create_timer(5.0, self.timer_callback)  # runs every 5 sec

        # default vehicle coordinates
        self.vehicle_latlng = [42.51502812839981, -83.04454857255062]
        self.vehicle_pos = [0, 0]
        self.max_distance = 50  # max lookahead distance in m

        # load the JSON database for maps

        self.json_data_gmaps = load_database('/home/connau/ConnAu/Data/Intersections/intersection_database_gmaps.json')
        self.json_data_carlat05 = load_database('/home/connau/ConnAu/Data/Intersections/intersection_database_carlat05.json')


    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        heading = msg.heading
        self.vehicle_latlng = [latitude, longitude]
        closest_intersection, distance = self.find_closest_intersection_gmaps()
        if closest_intersection is not None:
            msg_publish = String()
            intersection_id = closest_intersection['id']['id']
            msg_publish.data = str(intersection_id)
            self.publisher_.publish(msg_publish)

    def timer_callback(self, msg):
        # find the closest intersection
        data = json.loads(msg.data)
        # self.get_logger().info(f"data is {data}")
        try:
            self.vehicle_pos = [data['lat'], data['lon']]
            closest_intersection, distance = self.find_closest_intersection_carlat05()
        except KeyError as e:
            self.vehicle_pos = [data['lat'], data['lon']]
            # closest_intersection, distance = find_closest_intersection_gmap(self.vehicle_latlng, self.json_data, self.max_distance_km)

        # if closest_intersection:
        intersection_id = closest_intersection['id']['id']
        msg = String()
        msg.data = json.dumps(closest_intersection)
        self.publisher_.publish(msg)
            # self.get_logger().info(f'published intersection ID: {intersection_id}')

    def find_closest_intersection_carlat05(self):
        min_distance = float('inf')
        closest_intersection = None

        for intersection in self.json_data_carlat05:
            # self.get_logger().info(f"intersection is {intersection}")
            ref_point = intersection['refPoint']['carla_xy']
            distance = np.sqrt((self.vehicle_pos[0] - ref_point['x'])**2 + (self.vehicle_pos[1] - ref_point['y'])**2)

            if distance < min_distance and distance <= self.max_distance:
                min_distance = distance
                closest_intersection = intersection

        return closest_intersection, min_distance if closest_intersection else None

    def find_closest_intersection_gmaps(self):
        min_distance = float('inf')
        closest_intersection = None

        for intersection in self.json_data_gmaps:
            ref_point = intersection['refPoint']['latlng']
            distance = haversine(self.vehicle_latlng[0], self.vehicle_latlng[1], ref_point[0], ref_point[1])

            if distance < min_distance and distance <= self.max_distance:
                min_distance = distance
                closest_intersection = intersection

        return closest_intersection, min_distance if closest_intersection else None

def main(args=None):
    rclpy.init(args=args)
    node = IntersectionFinderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
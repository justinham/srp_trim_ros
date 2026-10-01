#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from fusion.msg import GPS  # Replace with your actual custom message type
import pyproj  # Library for geodetic calculations
import math
import numpy as np
import pyproj
from pyproj import Transformer
from fusion.msg import TrackedObjectList  # Import your custom message type
 
def convert_lat_lon_to_xy(ref_lat_rad, ref_lon_rad, ref_heading_rad, lat_rad,lon_rad):
    """
    Computes xy distances between two sets of lat/lon
    """
    f = 0.003353
    a = 6378137
    f1 = pow((f*(2-f)),0.5)
    f2 = a*(1-pow(f1,2))/pow((1-pow(f1,2)*pow(np.sin(ref_lat_rad),2)),3.0/2.0)
    f3 = a/pow((1-pow(f1,2)*pow((np.sin(ref_lat_rad)),2)),(1.0/2.0))
    E = f3 * np.cos(ref_lat_rad) * (lon_rad - ref_lon_rad)
    N = f2 * (lat_rad - ref_lat_rad)
    x_m = N*np.cos(ref_heading_rad) + E*np.sin(ref_heading_rad)
    y_m = - N*np.sin(ref_heading_rad)+ E*np.cos(ref_heading_rad)
    d2d_m = pow((pow(x_m,2)+pow(y_m,2)),0.5)
    return [x_m, y_m,d2d_m]

# load local JSON database
def load_database(json_file):
    with open(json_file, 'r') as f:
        return json.load(f)

class RealEgoGPStoXY(Node):
    def __init__(self):
        super().__init__('real_ego_gps_to_xy')

        self.origin_set = True
        self.origin_lat = 42.5150354710  # RAB SDSM ref point

        self.origin_lon = -83.0439986570

        # Set up a projection: WGS84 to a local UTM projection
        self.gps_subscriber = self.create_subscription(GPS, 'gps', self.gps_callback, 10)
        self.gps_can_subscriber = self.create_subscription(GPS, 'gps_can', self.gps_can_callback, 10)
        self.xy_publisher = self.create_publisher(String, 'ego_states', 10)
        self.publisher_ = self.create_publisher(String, 'closest_intersection_basic', 10)
        self.remote_subscriber = self.create_subscription(GPS, 'GT', self.gt_remote_callback, 10)
        self.gt_global_publisher = self.create_publisher(String, 'gt_remote_global', 10)

        self.mode_arg = self.declare_parameter('mode_arg', 'real').value


        self.std_dev_position_ego = 0.1
        self.std_dev_speed_ego = 0.04
        self.std_dev_heading_ego = 0.008  # in radians
        self.can_heading = 0.0

        self.vehicle_latlng = [42.5150296666667,-83.0437986666667]
        self.max_distance = 50  # max lookahead distance in m

        # load the JSON database for maps

        self.json_data_gmaps = load_database('/home/connau/ConnAu/Data/Intersections/intersection_database_gmaps.json')

        self.current_intersection_id = None

    def gt_remote_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        heading_f = msg.heading

        # theta = theta*180/np.pi - self.ego_theta

        local_x, local_y, _ = convert_lat_lon_to_xy(self.origin_lat*np.pi/180, self.origin_lon*np.pi/180, 0, latitude*np.pi/180, longitude*np.pi/180)

        actor_data = {
            "uniqueId": "remote",
            "timestamp": None,
            "class": "vru",
            "pos": {
                "lat": local_y,
                "lon": local_x
            },
            "heading": -heading_f*180/np.pi,
            "halfDim": None
        }
            
        json_str = json.dumps(actor_data)
        json_msg_pub = String()
        json_msg_pub.data = json_str
        self.gt_global_publisher.publish(json_msg_pub)

    def gps_can_callback(self, msg):
        self.can_heading = msg.heading_f

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude

        # heading = msg.heading
        heading = self.can_heading


        if not self.origin_set:
            self.origin_lat = latitude
            self.origin_lon = longitude
            self.origin_set = True
            self.get_logger().info(f"Origin set to: Lat {self.origin_lat}, Lon {self.origin_lon}")

        local_x, local_y, _ = convert_lat_lon_to_xy(self.origin_lat*np.pi/180, self.origin_lon*np.pi/180, 0, latitude*np.pi/180, longitude*np.pi/180)

        # Calculate local x-y coordinates relative to the origin
        # local_x = utm_x - origin_x
        # local_y = utm_y - origin_y

        json_data = {
            "timestamp": None,
            "ego_vehicle": {
                "uniqueId": "ego_gt",
                "carlaId": None,
                "class": "vehicle",
                "pos": {
                    "lat": local_y,
                    "lon": local_x,
                    "posStdDev_lat": self.std_dev_position_ego,
                    "posStdDev_lon": self.std_dev_position_ego
                },
                "vel": {
                    "lat_vel": 0.0,
                    "lon_vel": 0.0,
                    "velStdDev_lat": self.std_dev_speed_ego,
                    "velStdDev_lon": self.std_dev_speed_ego
                },
                "heading": {
                    "theta": -heading*180/np.pi,
                    "theta_valid": True,
                    "thetaStdDev": self.std_dev_heading_ego
                },
                "halfDim": None
            },
            "visible_actors": [],
            "is_visible_actors_present": True,
            "remote_timestamp": 0
        }
        # self.get_logger().info(f"theta is: {json_data['ego_vehicle']['heading']['theta']}")

        json_msg = String()
        json_msg.data = json.dumps(json_data)
        self.xy_publisher.publish(json_msg)

        # self.get_logger().info(f"{json_msg.data}")


        # self.vehicle_latlng = [latitude, longitude]
        # closest_intersection, distance = self.find_closest_intersection_gmaps()
        # if closest_intersection is not None:
        #     intersection_id = closest_intersection['id']['id']
        #     if intersection_id == self.current_intersection_id:
        #         return
        #     self.current_intersection_id = intersection_id
        #     ref_point = closest_intersection['refPoint']['latlng']
        #     local_x, local_y, radial_dist = convert_lat_lon_to_xy(self.origin_lat*np.pi/180, self.origin_lon*np.pi/180, 0, ref_point[0]*np.pi/180, ref_point[1]*np.pi/180)
        #     # local_x = utm_x - origin_x
        #     # local_y = utm_y - origin_y
        #     int_publish = String()
        #     json_data = {
        #         "id": str(intersection_id),
        #         "pos": {
        #             "x": local_y,
        #             "y": local_x
        #         }
        #     }

        #     int_publish.data = json.dumps(json_data)
        #     self.publisher_.publish(int_publish)
        #     self.get_logger().info(f"Intersectrion origin: {int_publish.data}")

    def find_closest_intersection_gmaps(self):
        min_distance = float('inf')
        closest_intersection = None

        for intersection in self.json_data_gmaps:
            ref_point = intersection['refPoint']['latlng']
            _, _, distance = convert_lat_lon_to_xy(self.vehicle_latlng[0]*np.pi/180, self.vehicle_latlng[1]*np.pi/180, 0, ref_point[0]*np.pi/180, ref_point[1]*np.pi/180)

            if distance < min_distance and distance <= self.max_distance:
                min_distance = distance
                closest_intersection = intersection

        return closest_intersection, min_distance if closest_intersection else None

def main(args=None):
    rclpy.init(args=args)
    node = RealEgoGPStoXY()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

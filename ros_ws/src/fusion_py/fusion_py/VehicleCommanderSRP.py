import can
import cantools
import sys

import cantools.database
import rclpy
from rclpy.node import Node
import rclpy
import rclpy.duration
import time
import json
import numpy as np

# import matplotlib.pyplot as plt

from fusion.msg import VehicleCommand
from fusion.msg import TripleVectorWps

## Justin ######

import paho.mqtt.client as mqtt
import base64
import struct
# import collections
import math


## configuration
# gateway_ip = '10.135.229.51' # pi address (MQTT server connect to bridge sensor)
gateway_ip = '192.168.5.51' # pi address on hummer (MQTT server connect to bridge sensor)
port = 1883

tag1 = '0c39' # left
tag2 = '879c' # right 
anc = '442a'


fn1 = 'uwb_recent_d1.txt'
fn2 = 'uwb_recent_d2.txt'

topic1 = 'dwm/node/' + tag1 + '/uplink/data'
topic2 = 'dwm/node/' + tag2 + '/uplink/data'

# --- Trailer UWB GEOMETRY CONSTANTS ---
Rv, Ra = 1.0, 1.0    # Half-width of trailer tags
L_sv = 1  # Sensor is 1m ahead of hinge
L_ht = 1.5  # Hinge to 2 Tag Line center distance 

yh = -3.0   # Hinge position relative to vehicle center
Lt = 3.0    # Hinge to Trailer center

L_total = L_sv + L_ht

# history = collections.defaultdict(lambda: collections.deque(maxlen=10))

## Filter
def is_outlier(val, window, threshold=1.0):
    """Simple outlier detection using Standard Deviation."""
    if len(window) < 5:  # Not enough data to judge yet
        return False
    
    arr = np.array(window)
    mean = np.mean(arr)
    std = np.std(arr)
    
    # If the reading is more than 'threshold' std devs away, it's an outlier
    if abs(val - mean) > threshold * std:
        return True
    return False

## localization: 1 anchor 2 tag (tag on veh)
def estimate_trailer_pos_dual_vehicle_sensors(d1, d2):
    
    # 1. Total effective lever arm
    # The gain now depends on the vehicle sensor width (Rv)
    denominator = 4 * Rv * L_sv
    
    # 2. Solve for Theta
    # We swap d1 and d2 here to maintain: Right Turn = Positive Angle
    sin_theta = (d2**2 - d1**2) / denominator
    sin_theta = np.clip(sin_theta, -1.0, 1.0)
    theta_rad = np.arcsin(sin_theta)
    
    # 3. Calculate Trailer Position
    tx = Lt * np.sin(theta_rad)
    ty = yh - Lt * np.cos(theta_rad)
    
    return np.degrees(theta_rad), (tx, ty)


## MQTT
def on_connect(client, userdata, flags, rc, properties=None):
    if rc==0:
        client.subscribe(topic1)
        client.subscribe(topic2)
        # client.subscribe(topic3)
        print('conn sub distance topic')
    else:
        print('conn fail')



def make_on_message(ros_node, uwb_data):

    def on_message(client, userdata, msg):

        try:
            time_ms = int(time.time()*1000)
            payload = json.loads(msg.payload.decode('utf-8'))
            raw_data = payload.get('data')
            sid = None
            if tag1 in msg.topic:
                sid = tag1
            elif tag2 in msg.topic:
                sid = tag2
            
            if not raw_data:
                return

            # Unpack the 6 bytes
            dec_byte = base64.b64decode(raw_data)
            addr_l, addr_h, d1, d2, d3, d4 = struct.unpack('BBBBBB', dec_byte)

            # dec_byte = base64.b64decode(raw_data)
            # addr_l, addr_h, d1, d2, d3, d4 = dec_byte[:6]
            
            addr = f'{addr_h:02x}{addr_l:02x}'
            
            # Calculate distance in meters
            dis = (d1 + d2*256 + d3*256**2 + d4*256**3) / 1000.0
            
            # --- FILTERING LOGIC ---

            # 1. Basic None/Zero/Negative filtering
            if dis <= 0 or dis is None:
                return

            # 2. Outlier filtering based on history
            # sensor_history = history[sid]
            
            # if is_outlier(dis, sensor_history):
            #     print(f"Skipping Outlier: {dis} for {sid}")
            #     return
                
            # 3. Add to dictionary if valid
            # sensor_history.append(dis)
            # d1 = history[tag1]
            # d2 = history[tag2]  
            uwb_data[sid] = dis
            
            d1 = uwb_data[tag1]
            d2 = uwb_data[tag2]

            # 4. theta estimation (using most recent data)
            theta_est, pos = estimate_trailer_pos_dual_vehicle_sensors(d1=d1, d2=d2)

            # 5. update value to ros
            data = [d1, d2, theta_est]
            ros_node.call_back_J(data)
    
            
            # --- LOGGING ---
            print("mqtt msg:", msg.topic, time_ms, addr, dis)
            
            # fn = 'uwb_recent.txt'
            # if tag1 in msg.topic: fn = fn1
            # elif tag2 in msg.topic: fn = fn2
            
            # with open(fn, 'w') as f:
            #     f.write('[%d,%.3f]' % (time_ms, dis))
                
        except Exception as e:
            print(f'mqtt msg err: {e}')

    return on_message

################



traj_filename = '/home/connau/srp/birdview/hummer_path/pathx5_can_heading.txt'


def read_nested_list_from_file_json(filename):
    with open(filename, 'r') as file:
        content = file.read()
        nested_list = json.loads(content)
        return nested_list

class VehicleCommanderSRP(Node):

    def __init__(self, argv):
        super().__init__('VehicleCommanderSRP')

        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = "can0"
        can.rc['bitrate'] = 500_000

        self.is_traj_track_ready = False
        self.points = None

        self.can_Bus = can.Bus()
        self.dbc = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/TrailerReverse_PoC.dbc') # defining dbc
        self.output_MSG_226 = self.dbc.get_message_by_frame_id(226) # defining frame id 226
        self.output_MSG_234 = self.dbc.get_message_by_frame_id(234) # defining frame id 234
        self.output_MSG_233 = self.dbc.get_message_by_frame_id(233) # defining frame id 233
        self.output_MSG_231 = self.dbc.get_message_by_frame_id(231) # defining frame id 231
        self.output_MSG_230 = self.dbc.get_message_by_frame_id(230) # defining frame id 230
        self.output_MSG_229 = self.dbc.get_message_by_frame_id(229) # defining frame id 229
        self.output_MSG_228 = self.dbc.get_message_by_frame_id(228) # defining frame id 228


        # initializing TrailerReverseRequestStatus to 0 (Inactive)
        self.data_dict_226 = {"VehUWBSens2_reading":0.0, "VehUWBSens1_reading":0.0, "HitchInclination":0.0, "HitchAngle":0.0, "TrailerReverseRequestStatus":0}
        self.data_dict_233 = {"TrailerDestination_y" : 28.0, "TrailerDestination_x" : 4.0, "TrailerDestination_Heading" : 10.0}
        self.data_dict_231 = {"TrailerUWBsens_z" : 1.0, "TrailerUWBsens_y" : 2.54 , "TrailerUWBsens_x" : -2.56}
        self.data_dict_230 = {"VehUWBsens2_z" : 1.0, "VehUWBsens2_y" : 2.54 , "VehUWBsens2_x" : -2.56, "VehUWBsens1_z": 1.1, "VehUWBsens1_y": 0.8, "VehUWBsens1_x": -1.6}
        self.data_dict_229 = {"TrailerHtch2Axle3" : 1.0, "TrailerHtch2Axle2" : 1.0 , "TrailerHtch2Axle1" : 1.0}
        self.data_dict_228 = {"VehRrAxl2HtchBall" : 1.1, "TrailerWidth" : 2.54 , "TrailerWheelbase" : 0.56, "TrailerPresent":1, "TrailerLength":5.5}
     

        data_228 = self.output_MSG_228.encode(self.data_dict_228)
        msg_228 = can.Message(arbitration_id=self.output_MSG_228.frame_id, data=data_228, is_extended_id = False)
        data_229 = self.output_MSG_229.encode(self.data_dict_229)
        msg_229 = can.Message(arbitration_id=self.output_MSG_229.frame_id, data=data_229, is_extended_id = False)
        data_231 = self.output_MSG_231.encode(self.data_dict_231)
        msg_231 = can.Message(arbitration_id=self.output_MSG_231.frame_id, data=data_231, is_extended_id = False)
        data_230 = self.output_MSG_230.encode(self.data_dict_230)
        msg_230 = can.Message(arbitration_id=self.output_MSG_230.frame_id, data=data_230, is_extended_id = False)
        try:
            self.can_Bus.send(msg_228)
            self.can_Bus.send(msg_229)
            self.can_Bus.send(msg_231)
            self.can_Bus.send(msg_230)
            print(f"Message sent on {self.can_Bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")
            print(f"{can.CanError}")
        
        self.write_ManeuverControl()

        self.timer = self.create_timer(1.0, self.send_new_traj)

        # self._data_lock = threading.Lock()

    def write_ManeuverControl(self): # frame id 226
        data_226 = self.output_MSG_226.encode(self.data_dict_226)
        msg_226 = can.Message(arbitration_id=self.output_MSG_226.frame_id, data=data_226, is_extended_id = False)
        try:
            self.can_Bus.send(msg_226)
            print(f"Message sent on {self.can_Bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")
            print(f"{can.CanError}")


    ### Justin
    def call_back_J(self, msg): # PLACEHOLDER .CALLBACK FUNCTION FOR JUSTIN TO BRING SENSOR DATA
        # with self._data_lock:
        self.data_dict_226['VehUWBSens2_reading'] = msg[0]
        self.data_dict_226['VehUWBSens1_reading'] = msg[1]
        self.data_dict_226['HitchAngle'] = msg[2]

        print("validate ros receive UWB data", msg)
        self.write_ManeuverControl()

    ########

    def send_new_traj(self):
        points = self.points
        try:
            new_points = read_nested_list_from_file_json(traj_filename)
        except json.JSONDecodeError:
            print(f"Error: The file '{traj_filename}' is empty or contains invalid JSON.")
            return
        except FileNotFoundError:
            print(f"Error: The file '{traj_filename}' was not found.")
            return
        if points == new_points:
            return
        points = new_points
        self.points = points
        self.get_logger().info(f"Got points from Justinapp: {points}")
        traj_x, traj_y, traj_h = [], [], []
        for i, pt in enumerate(points):
            traj_x.append(pt[0])
            traj_y.append(pt[1])
            # h_math = np.deg2rad(90.0 - pt[2])
            traj_h.append(pt[2]) # must be in deg

        self.is_traj_track_ready = False
        
        # setting TrailerReverseRequestStatus to 4 (SendingTrajectory)
        self.data_dict_226["TrailerReverseRequestStatus"] = 4
        self.write_ManeuverControl()

        time.sleep(0.1)

        traj_maxID = int(len(traj_x)-1)
        if traj_maxID > 255:
            traj_maxID = 255
        for i in range(len(traj_x)):
            if i > 255:
                break
            data_dict_234 = {"InitialTrajectory_y":traj_y[i], "InitialTrajectory_x":traj_x[i], "InitialTrajectory_MaxID":traj_maxID, "InitialTrajectory_ID":i, "InitialTrajectory_Heading":traj_h[i]}
            data = self.output_MSG_234.encode(data_dict_234)
            msg = can.Message(arbitration_id=self.output_MSG_234.frame_id, data=data, is_extended_id = False)
            if i==len(traj_x)-2:
                self.data_dict_233 = {"TrailerDestination_y" : traj_y[i], "TrailerDestination_x" : traj_x[i], "TrailerDestination_Heading" : traj_h[i]}
            

            try:
                self.can_Bus.send(msg)
            except can.CanError:
                print("Message NOT sent")
                print(f"{can.CanError}")
            time.sleep(0.02)

        time.sleep(0.1)


        data_233 = self.output_MSG_233.encode(self.data_dict_233)
        msg_233 = can.Message(arbitration_id=self.output_MSG_233.frame_id, data=data_233, is_extended_id = False)
        try:
            self.can_Bus.send(msg_233)
            print(f"Message sent on {self.can_Bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")
            print(f"{can.CanError}")

        time.sleep(0.1)

        # is_traj_sent_success = False
        # while not is_traj_sent_success:
        #     with can.Bus() as bus:
        #         for frame_msg in bus:
        #             frame_id = frame_msg.arbitration_id
        #             if frame_id == 195:
        #                 decoded = self.dbc.decode_message(frame_msg.arbitration_id, frame_msg.data)
        #                 if decoded['TrajectoryReceived'] == 1:
        #                     is_traj_sent_success = True
        #                     break

        # time.sleep(0.1)

        self.data_dict_226["TrailerReverseRequestStatus"] = 0
        self.write_ManeuverControl()

        time.sleep(0.1)

        is_traj_track_ready = False
        while not is_traj_track_ready:
            with can.Bus() as bus:
                for frame_msg in bus:
                    frame_id = frame_msg.arbitration_id
                    if frame_id == 195:
                        decoded = self.dbc.decode_message(frame_msg.arbitration_id, frame_msg.data)
                        if decoded['TrailerReverseStatus'] == 1:
                            self.is_traj_track_ready = True
                            break

    def reverse_req_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    publisher = VehicleCommanderSRP(sys.argv)

    ## Justin ##
    uwb_data = {
        tag1: 0.0,
        tag2: 0.0  
    } 

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = make_on_message(publisher, uwb_data)

    client.connect(gateway_ip, port, 60)
    client.loop_start() # move MQTT to background thread (non-blocking)

    try:
        rclpy.spin(publisher)
        # client.loop_forever() # need to check conflict between two spins

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:

         # Clean up MQTT thread 
        client.loop_stop()
        client.disconnect()

        publisher.destroy_node()
        rclpy.shutdown()

    ##############

if __name__ == '__main__':
    main(sys.argv)


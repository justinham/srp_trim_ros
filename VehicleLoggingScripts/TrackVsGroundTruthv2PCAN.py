import sys
import datetime
from datetime import timedelta
from datetime import timezone
import time
import calendar
import math
import numpy as np
import os

SENSOR_COMP = True # Change only if you have disabled sensor compensation in data_publisher/can_to_message

def main(argv):
    c = 0
    if len(argv) < 3:
        print("ERROR: Did not provide two files, first file is the tracks log from ego vehicle, second is log from gps device converted to ego perspective")
        exit(1)

    tracks_file = open(argv[1], 'r')
    converted_file = open(argv[2], 'r')

    # Gets the vehicle spy timestamps in ms since epoch
    tracks = tracks_file.readlines()
    timestamp = tracks[0]
    timestamp_split = (timestamp.split(' '))[2:]
    date = timestamp_split[0].split('-')
    timestamp_split = ((timestamp_split[1]).split(","))[0].split(':')

    # Read the two files
    tracks = tracks[1:]
    for i in range(len(tracks)):
        tracks[i] = tracks[i][:-1].split(',')
        tracks[i][0] = int(float(tracks[i][0])*1000)

    converted_tracks = converted_file.readlines()
    converted_tracks = converted_tracks[1:]
    for i in range(len(converted_tracks)):
        converted_tracks[i] = converted_tracks[i][:-1].split(',')

    tracks_file.close()
    converted_file.close()

    output = []
    output.append(["TimeStamp in ms since epoch","sensor_id","track_id","track_status","confidence","object_type","lat offset from ego (m)","lon offset from ego (m)","lat_vel w.r.t ego (m/s)","lon_vel w.r.t. ego (m/s)","theta w.r.t. ego","theta_valid","theta_cov","lat covariance","lon covariance","lat_vel covariance","lon_vel covariance","interpolated ground truth x in m", "interpolated ground truth y in m", "interpolated x velocity in m/s", "interpolated y velocity in m/s",  "lat error", "lon error", "lat vel error", "lon vel error","interpolated heading in radians", "heading error", "int ego lattitude", "int ego longitude", "int ego heading", "remote latitude", "remote_longitude", "remote_heading"])
    for i in range(len(tracks)):
    
        # Is a pedestrian track
        end_t = 0
        start_t = 0
        for j in range(1, len(converted_tracks)):
            # Check Timestamps to make sure it is in the valid range of stamps
            if int(converted_tracks[j][1]) > int(tracks[i][0]):
                end_t = int(converted_tracks[j][1])
                start_t = int(converted_tracks[j-1][1])
                break
        delta_t_ms = end_t - start_t
        if end_t == 0 and start_t == 0 or delta_t_ms > 1100: # Uses 1.1 seconds to account for delays
            print(f"Track not covered by gps data: {start_t}")
            continue
        
        delta_t_s = delta_t_ms/1000.0
        delta_target_t_s = (tracks[i][0] - start_t)/1000.0
        
        # Target
        lat_v = (float(converted_tracks[j][8]) - float(converted_tracks[j-1][8]))/delta_t_s
        lon_v = (float(converted_tracks[j][9]) - float(converted_tracks[j-1][9]))/delta_t_s
        lat_i = float(converted_tracks[j-1][8]) + lat_v * delta_target_t_s
        lon_i = float(converted_tracks[j-1][9]) + lon_v * delta_target_t_s
        roation_speed = (float(converted_tracks[j][11]) - float(converted_tracks[j-1][11]))/delta_t_s
        heading_i = (roation_speed * delta_target_t_s) + float(converted_tracks[j][11])

        # EGO
        lat_v_ego = (float(converted_tracks[j][5]) - float(converted_tracks[j-1][5]))/delta_t_s
        lon_v_ego = (float(converted_tracks[j][6]) - float(converted_tracks[j-1][6]))/delta_t_s
        lat_i_ego = float(converted_tracks[j-1][5]) + lat_v_ego * delta_target_t_s
        lon_i_ego = float(converted_tracks[j-1][6]) + lon_v_ego * delta_target_t_s

        roation_speed_ego = (float(converted_tracks[j][7]) - float(converted_tracks[j-1][7]))/delta_t_s
        heading_i_ego = (roation_speed_ego * delta_target_t_s) + float(converted_tracks[j][7])
        
        
        lat_v_wrt_ego = lat_v
        lon_v_wrt_ego = lon_v

        lat_obs = float(tracks[i][6])
        lon_obs = float(tracks[i][7])

        # Sensor Ids: 0 - Infra, 1 - LRR, 2 - FCM, 3- SRRLF, 4 - SRRLR
        # Setup output
        tracks[i].append(lat_i)
        tracks[i].append(lon_i)
        tracks[i].append(lat_v_wrt_ego)
        tracks[i].append(lon_v_wrt_ego)
        tracks[i].append(abs(lat_obs-lat_i))
        tracks[i].append(abs(lon_obs-lon_i))
        tracks[i].append(abs(float(tracks[i][8])-lat_v_wrt_ego))
        tracks[i].append(abs(float(tracks[i][9])-lon_v_wrt_ego))
        tracks[i].append(heading_i) # Interpolated heading
        if tracks[i][11] == "True":
            tracks[i].append(abs(float(tracks[i][10])-heading_i)) # Heading Error
        else:
            tracks[i].append("N/A")
            tracks[i][10] = "N/A"
            tracks[i][12] = "N/A"
        tracks[i].append(lat_i_ego) 
        tracks[i].append(lon_i_ego)
        tracks[i].append(heading_i_ego)
        tracks[i].append(converted_tracks[j][2])
        tracks[i].append(converted_tracks[j][3])
        tracks[i].append(converted_tracks[j][4])
        
        output.append(tracks[i])
    #print(output)
    out_file = open(os.path.dirname(argv[1]) + "/Comparison.csv", 'w+')
    for row in output:
        for i in range(len(row)):
            out_file.write(f"{row[i]}")
            if i != len(row) - 1:
                out_file.write(',')
        out_file.write('\n')

    out_file.close()

if __name__ == '__main__':
    main(sys.argv)
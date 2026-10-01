import sys
import datetime
from datetime import timedelta
from datetime import timezone
import time
import calendar
import math
import numpy as np
import os

DEG2RAD_FACTOR = math.pi/180.0
DEGREE2RADIAN_FACTOR = math.pi / 180.0
EARTH_RADIUS_KM = 6378.137
EARTH_RADIUS_M = 6378137
FEET2METER_FACTOR = 0.3048
METER2FEET_FACTOR = 3.28084

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

def main(argv):
    if len(argv) < 3:
        print("ERROR: Did not provide two files, first file is the log from ego vehicle, second is log from gps device for pedestrian")
        exit(1)
    ped_ego_file = open(argv[1], 'r')
    ped_gps_file = open(argv[2], 'r')

    ped_gps = ped_gps_file.readlines()
    ped_gps[1:]
    for i in range(len(ped_gps)):
        ped_gps[i] = ped_gps[i][:-1].split(',')
    
    ped_ego = ped_ego_file.readlines()
    timestamp = ped_ego[:1]
    ped_ego = ped_ego[2:]
    for i in range(len(ped_ego)):
        ped_ego[i] = ped_ego[i][:-1].split(',')
    #print(ped_ego)

    ped_ego_file.close()
    ped_gps_file.close()

    # Convert ego timestamps to ms since epoch currently utilize GPS timestamp
    
    
    gps_datetime = datetime.datetime(int(ped_ego[0][5]), 1, 1) + timedelta(int(ped_ego[0][6]))
    ego_ref_time = calendar.timegm(gps_datetime.timetuple())*1000
    timestamp_split = (timestamp[0].split(' '))[2:]
    date = timestamp_split[0].split('-')
    timestamp_split = ((timestamp_split[1]).split(","))[0].split(':')

    vehicle_datetime = datetime.datetime(int(date[0]), int(date[1]), int(date[2]), int(timestamp_split[0]), int(timestamp_split[1]), int(timestamp_split[2]),  int(timestamp_split[3]), tzinfo=timezone.utc)
    

    #print(timestamp_split)
    #print(vehicle_datetime)
    #print((vehicle_datetime.timetuple()))
    

    for i in range(len(ped_ego)):
        # set the end of the arrays to be the utc timestamp in ms
        ped_ego[i].append(ego_ref_time + int(ped_ego[i][7]))
    
        # convert lat lon and theta to radians
        ped_ego[i][1] = float(ped_ego[i][1]) / 3600000.0 / 180.0 * math.pi
        ped_ego[i][2] = float(ped_ego[i][2]) / 3600000.0 / 180.0 * math.pi
        ped_ego[i][3] = float(ped_ego[i][3]) / 180.0 * math.pi

        temp = ((vehicle_datetime + timedelta(seconds=float(ped_ego[i][0]))))
        ped_ego[i][0] = int(temp.timestamp()*1000)
    
    
    # Ped EGO Vehicle Spy time, lat, lon, heading, speed, day, year, ms since midnight, GPS time
    # Time, speed, bearing, lat, lon

    in_range = []
    for i in range(1, len(ped_gps)):
        if float(ped_gps[i][0]) < ped_ego[0][8] or ped_ego[-1][8] < float(ped_gps[i][0]):
            pass
        else:

            # NOTE for some reason when preforming the degrees to rad conversion in one line it caused the result to be reduced by a factor of 10
            # print(ped_gps[i])
            ped_gps[i][3] = float(ped_gps[i][3])
            ped_gps[i][4] = float(ped_gps[i][4])
            # print(ped_gps[i])
            ped_gps[i][3] /= 180.0
            ped_gps[i][4] /= 180.0
            # print(ped_gps[i])
            ped_gps[i][3] *= math.pi
            ped_gps[i][4] *= math.pi
            # print(ped_gps[i])
            in_range.append(ped_gps[i])
            # print(in_range[-1])
    
    print(len(in_range))
    out = []
    for i in range(len(in_range)):
        
        
        j = 0
        while j < len(ped_ego):
            if ped_ego[j][8] > float(in_range[i][0]):
                break
            else:
                j += 1
        
        vehicle_time_dif = (ped_ego[j][0] - ped_ego[j-1][0])/1000.0
        delta_t_target = (float(in_range[i][0]) - ped_ego[j-1][8])/1000.0
        delta_t = (ped_ego[j][8] - ped_ego[j-1][8])/1000.0
        vehicle_time = int(((delta_t_target/delta_t) * vehicle_time_dif) * 1000 + ped_ego[j-1][0])
        lat_v = (ped_ego[j][1] - ped_ego[j-1][1])/delta_t
        lon_v = (ped_ego[j][2] - ped_ego[j-1][2])/delta_t
        heading_yaw = (ped_ego[j][3] - ped_ego[j-1][3])/delta_t

        lat_inter = ped_ego[j-1][1] + delta_t_target * lat_v
        lon_inter = ped_ego[j-1][2] + delta_t_target * lon_v
        heading_inter = ped_ego[j-1][3] + delta_t_target * heading_yaw

        res = convert_lat_lon_to_xy(lat_inter, lon_inter, heading_inter, in_range[i][3], in_range[i][4])

        out.append([in_range[i][0], vehicle_time, in_range[i][3], in_range[i][4], lat_inter, lon_inter, heading_inter, -res[1], -res[0], res[2]])
        #print(out[-1])

    #path = argv[:argv[1].rindex('/')]
    out_file = open(os.path.dirname(argv[1]) + "/ConvertedDistancesPedestrain.csv", 'w+')
    out_file.write("time in ms since epoch - gps, vehicle spy time in ms since epoch, pedestrian lat, pedestrain lon, ego interpolated lat, ego interpolated lon, ego interpolated heading, x in meters, y in meters, distance in meters\n")
    for row in out:
        for i in range(len(row)):
            out_file.write(f"{row[i]}")
            if i != len(row) - 1:
                out_file.write(',')
        out_file.write('\n')
    out_file.close()
    




if __name__ == '__main__':
    main(sys.argv)
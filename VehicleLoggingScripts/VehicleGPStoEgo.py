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
        print("ERROR: Did not provide two files, first file is the log from ego vehicle, second is log from gps device for vehicle")
        exit(1)
    ego_file = open(argv[1], 'r')
    veh_gps_file = open(argv[2], 'r')

    veh_gps = veh_gps_file.readlines()
    veh_gps = veh_gps[1:]
    for i in range(len(veh_gps)):
        veh_gps[i] = veh_gps[i][:-1].split(',')
    
    ego_gps = ego_file.readlines()
    timestamp = ego_gps[:1]
    print(timestamp)
    ego_gps = ego_gps[2:]
    for i in range(len(ego_gps)):
        ego_gps[i] = ego_gps[i][:-1].split(',')
    #print(ego_gps)

    ego_file.close()
    veh_gps_file.close()

    # Convert ego timestamps to ms since epoch currently utilize GPS timestamps
    
    # VEHGPS - TimeStamp,TimeValue,Log SN,GNSS SN,GNSS Time,FrozenState,LatitudeDeg,LongitudeDeg,AltitudeM,HeadingDeg,FilteredHeadingDeg,SpeedMps,Coasted,FixQuality,DiffApplied,RtcmVer,RtcmBytesRxOTA,RtcmBytesToUblox
    # EGO GPS - Vehicle Spy time, lat, lon, heading, speed, day, year, ms since midnight

    gps_datetime = datetime.datetime(int(ego_gps[0][5]), 1, 1) + timedelta(int(ego_gps[0][6]))
    ego_ref_time = calendar.timegm(gps_datetime.timetuple())*1000
    print(ego_ref_time)
    timestamp_split = (timestamp[0].split(' '))[2:]
    date = timestamp_split[0].split('-')
    timestamp_split = ((timestamp_split[1]).split(","))[0].split(':')

    vehicle_datetime = datetime.datetime(int(date[0]), int(date[1]), int(date[2]), int(timestamp_split[0]), int(timestamp_split[1]), int(timestamp_split[2]),  int(timestamp_split[3]), tzinfo=timezone.utc)
    
    for i in range(len(ego_gps)):
        # set the end of the arrays to be the utc timestamp in ms
        ego_gps[i].append(ego_ref_time + int(ego_gps[i][7]))
    
        # convert lat lon and theta to radians
        ego_gps[i][1] = float(ego_gps[i][1]) / 180.0 * math.pi
        ego_gps[i][2] = float(ego_gps[i][2]) / 180.0 * math.pi
        ego_gps[i][3] = float(ego_gps[i][3])

        temp = ((vehicle_datetime + timedelta(seconds=float(ego_gps[i][0]))))
        ego_gps[i][0] = int(temp.timestamp()*1000)
    
    # VEHGPS - TimeStamp,TimeValue,Log SN,GNSS SN,GNSS Time,FrozenState,LatitudeDeg,LongitudeDeg,AltitudeM,HeadingDeg,FilteredHeadingDeg,SpeedMps,Coasted,FixQuality,DiffApplied,RtcmVer,RtcmBytesRxOTA,RtcmBytesToUblox
    # EGO GPS - Vehicle Spy time, lat, lon, heading, speed, day, year, ms since midnight, gps time

    in_range = []
    for i in range(0, len(veh_gps)):
        if float(veh_gps[i][1]) < ego_gps[0][8] and ego_gps[-1][8] < float(veh_gps[i][1]):
            pass
        else:

            # NOTE for some reason when preforming the degrees to rad conversion in one line it caused the result to be reduced by a factor of 10
            veh_gps[i][6] = float(veh_gps[i][6])
            veh_gps[i][7] = float(veh_gps[i][7])
            veh_gps[i][6] /= 180.0
            veh_gps[i][7] /= 180.0
            veh_gps[i][6] *= math.pi
            veh_gps[i][7] *= math.pi
            in_range.append(veh_gps[i])

    out = []
    for i in range(len(in_range)):
        
        
        j = 0
        while j < len(ego_gps):
            if ego_gps[j][8] > float(in_range[i][1]):
                break
            else:
                j += 1

        if j == 0 or j == len(ego_gps):
            continue
        
        # Calculate Deltas for time
        vehicle_time_dif = (ego_gps[j][0] - ego_gps[j-1][0])/1000.0 # Delta Vehicle Spy Time
        delta_t_target = (float(in_range[i][1]) - ego_gps[j-1][8])/1000.0 # Change Delta between Target GPS Time and EGO GPS Time
        delta_t = (ego_gps[j][8] - ego_gps[j-1][8])/1000.0 # Delta EGO GPS Time
        
        # Calculat new Vehicle Spy time
        vehicle_time = int(((delta_t_target/delta_t) * vehicle_time_dif) * 1000 + ego_gps[j-1][0])
        
        # Rate of changes
        lat_v = (ego_gps[j][1] - ego_gps[j-1][1])/delta_t
        lon_v = (ego_gps[j][2] - ego_gps[j-1][2])/delta_t
        heading_yaw = (ego_gps[j][3] - ego_gps[j-1][3])/delta_t

        # Interpolated Information
        lat_inter = ego_gps[j-1][1] + delta_t_target * lat_v
        lon_inter = ego_gps[j-1][2] + delta_t_target * lon_v
        heading_inter = ego_gps[j-1][3] + delta_t_target * heading_yaw

        res = convert_lat_lon_to_xy(lat_inter, lon_inter, heading_inter, in_range[i][6], in_range[i][7])

        # Hummer GPS Correction
        x_gps_offset = 0
        y_gps_offset = -2.39649

        # This is for the MY21 Escalade
        #x_gps_offset = -0.86487
        #y_gps_offset = 0.55753

        # Account for orientation
        gps_lat_offset_correction = x_gps_offset * np.cos(-heading_inter) - y_gps_offset * np.sin(-heading_inter)
        gps_lon_offset_correction = x_gps_offset * np.sin(-heading_inter) + y_gps_offset * np.cos(-heading_inter)
        lat_dif = (-res[1]) + gps_lat_offset_correction
        lon_dif = res[0] + gps_lon_offset_correction
        tot_dist = pow((pow(lat_dif,2)+pow(lon_dif,2)),0.5)
        out.append([in_range[i][1], vehicle_time, in_range[i][6] / math.pi * 180, in_range[i][7] / math.pi * 180, (float(in_range[i][9]) / 180 * math.pi), lat_inter / math.pi * 180, lon_inter / math.pi * 180, heading_inter, lat_dif, lon_dif, tot_dist, (float(in_range[i][9]) / 180 * math.pi)-heading_inter])

    out_file = open(os.path.dirname(argv[1]) + "/ConvertedDistancesVehicle.csv", 'w+')
    out_file.write("time in ms since epoch - gps,vehicle spy time in ms since epoch,vehicle lat,vehicle lon,vehicle heading,ego interpolated lat,ego interpolated lon,ego interpolated heading,x in meters,y in meters,distance in meters,relative heading\n")
    for row in out:
        for i in range(len(row)):
            out_file.write(f"{row[i]}")
            if i != len(row) - 1:
                out_file.write(',')
        out_file.write('\n')
    out_file.close()
    




if __name__ == '__main__':
    main(sys.argv)
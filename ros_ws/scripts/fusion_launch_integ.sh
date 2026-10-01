#!/bin/bash

function shutdown {
    echo "Shutting down ROS2 nodes (gracefully)..."
    #pkill gzserver
    #pkill gzclient
    pkill -f ros2
    #pkill -f CarlaUE4
    # sudo sync; sudo sysctl -w vm.drop_caches=3
    ros2 daemon stop
    ros2 daemon start
    wait 
    echo "Shutdown complete"
    exit 0
}

trap shutdown SIGINT

sudo modprobe peak_usb
sudo modprobe peak_pci

sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can2 down

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 500000 sample-point 0.8 dbitrate 2000000 dsample-point 0.8 fd on
sudo ip link set can2 up type can bitrate 500000

sudo stty -F /dev/ttyUSB0 115200
sudo chmod 777 /dev/ttyUSB0

source install/setup.sh

# ros2 run fusion ControlGen > CONTROLGEN_LOG.log 2>&1
ros2 run fusion ControlGen &
# ros2 run fusion Fusion &
ros2 run fusion_py WpsFromJustinApp > wpsfromjustin.log 2>&1 &
ros2 run fusion_py SerialGPSReader > ublox_gps_log.log 2>&1 &
ros2 run fusion_py CANGPSReader > can_gps_log.log 2>&1 &
# ros2 run fusion_py CANTrackReader &

sleep 2

ros2 run fusion_py VehicleCommander

wait
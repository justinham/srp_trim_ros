#!/bin/bash

function shutdown {
    echo "Shutting down Carla and ROS2 nodes (gracefully)..."
    pkill -f ros2
    # pkill -f CarlaUE4
    pkill gzserver
    pkill gzclient

    # sudo sync; sudo sysctl -w vm.drop_caches=3
    ros2 daemon stop
    ros2 daemon start
    wait 
    echo "Shutdown complete"
    exit 0
}

trap shutdown SIGINT

# echo "Starting Carla in headless mode..."
# /home/yz4d3h/carla15/CarlaUE4.sh -prefernvidia -carla-port=2000 > carla.log 2>&1 &

# Wait for a few seconds to ensure Carla has started
sleep 5

source /opt/ros/humble/setup.bash
source ~/ConnAu/ros_ws/install/setup.bash

# Launch the GT ROS 2 package
echo "Launching ground_truth_sim_pkg package..."
ros2 launch ground_truth_sim_pkg ground_truth_sim.launch.py > ros2_ground_truth_sim_pkg.log 2>&1 &

echo "Launching fusion Node"
ros2 run fusion Fusion > fusion.log 2>&1 &

sleep 5

# ros2 run ground_truth_sim_pkg infra_playback_node &
# sleep 69.07
# echo "Launching Ego Playback"
# ros2 run fusion_py ComparisonVisualizerv2 /home/yz4d3h/Documents/vehicle_process/HEV_logs_06_25/hev_stationary/filterted_calibrated.csv 

# Wait for all background processes to finish
wait

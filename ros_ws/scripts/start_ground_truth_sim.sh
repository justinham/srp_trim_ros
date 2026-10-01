#!/bin/bash

function shutdown {
    echo "Shutting down Carla and ROS2 nodes (gracefully)..."
    pkill gzserver
    pkill gzclient
    pkill -f ros2
    pkill -f CarlaUE4
    # sudo sync; sudo sysctl -w vm.drop_caches=3
    ros2 daemon stop
    ros2 daemon start
    wait 
    echo "Shutdown complete"
    exit 0
}

trap shutdown SIGINT

echo "Starting Carla in headless mode..."

/home/connau/carla15/CarlaUE4.sh -prefernvidia -carla-port=2000> carla.log 2>&1 &

# Wait for a few seconds to ensure Carla has started
sleep 10

source /opt/ros/humble/setup.bash
source ~/Workspace/ros-bridge/install/setup.bash

# Launch the GT ROS 2 package
echo "Launching ground_truth_sim_pkg package..."
ros2 launch ground_truth_sim_pkg ground_truth_sim.launch.py > ros2_ground_truth_sim_pkg.log 2>&1 &

# Wait for all background processes to finish
wait

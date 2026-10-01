#!/bin/bash
# Add this before launching nodes to ensure a fresh start
ros2 daemon stop > /dev/null 2>&1
ros2 daemon start > /dev/null 2>&1

# Function to handle cleanup on Ctrl+C
function shutdown {
    echo -e "\n[SHUTDOWN] Terminating all ROS2 nodes..."
    
    # 1. Kill everything started by this script
    jobs -p | xargs -r kill -9 2>/dev/null
    
    # 2. Force kill any remaining ROS2 or Python nodes
    # This targets the actual node executors
    pkill -9 -f "ros2"
    pkill -9 -f "SerialGPSReader"
    pkill -9 -f "python3"
    
    # 3. Wipe the DDS shared memory (The "Ghost" Fix)
    # This clears the discovery cache that makes nodes appear in 'ros2 node list'
    ros2 daemon stop > /dev/null 2>&1
    sudo rm -rf /dev/shm/fastrtps_* 2>/dev/null
    
    echo "[SHUTDOWN] Cleanup complete. Exiting."
    exit 0
}


# Ensure a mode (real/sim) was provided
if [ -z "$1" ]; then
    echo "Usage: ./mode_launch.sh <mode_arg>"
    exit 1
fi

MODE_ARG="$1"

# Trap Ctrl+C (SIGINT) and call the shutdown function
trap shutdown SIGINT SIGTERM

# Cache sudo credentials so it doesn't hang later
echo "Checking sudo permissions..."
sudo -v

# --- HW Setup ---
echo "Configuring CAN and Serial interfaces..."
sudo modprobe peak_usb
sudo modprobe peak_pci

for i in 0 1 2; do sudo ip link set can$i down 2>/dev/null; done

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 500000 sample-point 0.8 dbitrate 2000000 dsample-point 0.8 fd on
sudo ip link set can2 up type can bitrate 500000

# sudo stty -F /dev/ttyUSB0 115200
# sudo chmod 777 /dev/ttyUSB0
sudo stty -F /dev/ttyACM0 115200
sudo chmod 777 /dev/ttyACM0

# --- ROS2 Environment ---
source /opt/ros/humble/setup.bash
source ~/ConnAu/ros_ws/install/setup.bash

# --- Launch Nodes ---
echo "Starting ROS2 Nodes (logs redirected)..."

echo "Launching main package..."
export LD_LIBRARY_PATH="$HOME/.local/gtsam-4.2/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ros2 launch ground_truth_sim_pkg ground_truth_sim.launch.py mode_arg:="$MODE_ARG" > LOG_ros2_ground_truth_sim_pkg.log 2>&1 &



echo "launching GPS nodes"
ros2 run fusion_py SerialGPSReader > LOG_ros2_ublox_gps.log 2>&1 &
ros2 run fusion_py CANGPSReader > LOG_ros2_can_gps.log 2>&1 &    

echo "Launching AAOS bridge nodes"
ros2 run ground_truth_sim_pkg ros2_to_app_bridge_node > LOG_ros2_to_app_bridge_node.log 2>&1 &
ros2 run ground_truth_sim_pkg app_responder_node > LOG_app_responder_node.log 2>&1 &

echo "Launching EGO Sensing and Fusion nodes"
ros2 run fusion Fusion > LOG_EGO_fusion.log 2>&1 &
ros2 run fusion_py CANTrackReader > LOG_EGO_sensing.log 2>&1 &

echo "Launching V2X interfacing node"
ros2 run ground_truth_sim_pkg obu_interface_node2 > LOG_obu_interface_node2.log 2>&1 &

echo "Launching Steering Control node"
ros2 run fusion ControlGen3 > LOG_ControlGen3.log 2>&1 &

sleep 5

echo "Launching Collision Avoidance node"
ros2 run ground_truth_sim_pkg ego_speed_test_node_ff3_new > LOG_ego_speed_test_node_ff3_new.log 2>&1 &

echo "Launching Object Factor Graph Fusion node"
ros2 run fusion object_factor_graph_fusion_node > LOG_object_factor_graph_fusion_node.log 2>&1 &

echo "All nodes launched. Press Ctrl+C to stop."

# wait for background processes to keep the script alive
wait

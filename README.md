# ConnAu - Connected Autonomy
Goal of the project is to enable a Vehicle to utilize offboard/infrastructure based sensors. We can then leverage these new sensing capabilities in a multitude of ways. Specifically, in this project we aim to enable a Vehicle to safely traverse an Adverse intersection, in which Non-Ego participants interfere with a standard trajectory.

## File Structure
- DBC-ARXML, Folder contains the DBC files for reading the CAN data from the vehicles, as well as the VehicleControl dbc
- Data, Contains Some map data and other misc data
- SimScenarios, contains jupyter notebooks that run our CARLA Simulation examples
- VehicleLoggingScripts, contains scripts to post process the data collect on vehicle
- car_008, contains files needed for Gazebo
- ros_ws, This contains all of the ros nodes that make up the core logic of our project. Here you will find a mixture of C++ and Python Ros nodes that power our work

## How to use our project
- Go to project root folder and run './ros_ws/scripts/mode_launch.sh'
- This command launches following functionalities
- - UBlox (SerialGPSReader) and CAN GPS (CANGPSReader) publishers
- - Commnication bridge b/w ubuntu and VCU via VCU's hotspot uusing WebSocket
- - - ros2_to_app_bridge_node - sends ego gps, object and obstacle data to VCU
- - - app_responder_node - sends user input (sensing, auto mode) from VCU to ubuntu
- - OBU interface and SDSM decoding (obu_interface_node2)
- - Vehicle Control
- - - Steering (ControlGen2)
- - - Speed and Collision Avoidance (ego_speed_test_node_ff3) (NOTE: ego_speed_test_node_ff3 fetch basic RAB map lane data from ConnAu/Data/INtersections/intersection_database_gmaps.json)
- - CAN Object tracks (CANTrackReader)
- - Object data processing
- - - Ego level fusion (fusion_node)
- - - Ego and infra data axes transformation (ego_to_global, infra_to_global)
- - - Ego-Infra object id matching (object_matcher_node)
- - - Ego-infra level factor graph fusion (object_factor_graph_fusion_node)
- - - Data formatting (ego_msg_to_json, real_infra_source_from_gt)
- - Gazebo visualizer (object_visualizer_node) - DISABLED/BACKUP
- - Playback nodes
- - - Processed infra data along with Ego GPS (infra_playback_node2, infra_playback_node2)
- - - Processed ego object tracks along with its GPS (ComparisonVisualizerv2, ComparisonVisualizerv3)
- - - Sync plays both processed ego and infra (ego_infra_playback_node) - make sure fg node is running with use_sim_time set to true

## SOFTWARE ARCHITECTURE
![PUB-SUB STRUCTURE](images/connint_arch_updated_0702.png)

## DATA PROCESSING FOR VALUDATION/SENSOR CALIBRATION
- Ego (refer to 'VehiceleLoggingScripts/README.md)
- Infra (refer to 'InfraLoggingScripts/README.md)

## RAB Trajectory
![North & East Ingress](images/l1_l3_paths_dense_xd.png)
![South & West Ingress](images/l5_l7_paths_dense_xd.png)

You might find 3 different RAB trajectory files under /ConnAu/Data/offline_path_files/
- 'RAB_paths' has all 12 trajectories but doesn't align with road map along the turns (100 interpolated points b/w anchor points)
- 'RAB_paths_revised_curves' is 'RAB_paths' with corrected curves to fit within road map (100 interpolated points b/w anchor points) - CURRENTLY USED - plotted above
- 'RAB_paths_sparse' is 'RAB_paths_revised_curves' with lesser density (10 interpolated points b/w anchor points)
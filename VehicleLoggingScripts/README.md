# ConnAu

## Vehicle Data Processing Scripts:

Data Flow:
1. VehicleSpy Export as Asc - currently filter to only networks 5 and 8: 
    - Input: Vehicle Spy Recording
    - Output: VSpy Ascii file of data
2. FileTransformer.py
    - Input: VSpy Ascii file of data
    - Output: Scrapped Text file containing important data from VSpy Ascii
3. data_publisher ROS Node
    - Input: Scrapped Text file
    - Output: Make sure to add timestamp!!!!!! Can be obtained from VehicleSpy first message after filtering to the desired networks!
        1. Scrapped Text File -gps.txt, contains only gps data from Ego Vehicle
        2. Scrapped Text File -tracks.txt, contains LRR, FCM, SRRRF, SRRLF tracks
        3. ROS messages - Not used in Ground Truthing
4. VehicleGPStoEgo.py
    - Input:
        1. Scrapped Text File -gps.txt
        2. GPS Data from GPS Phone.csv - Chaun's or Data from Target Vehicle
        time,speed,bearing,latitude,longitude
    - Output: ConvertedDistances.csv
5. TrackVsGroundTruth.py
    - Input:
        1. Scrapped Text File -tracks.txt
        2. ConvertedDistances.csv
    - Output: Comparison.csv



### TrackVsGroundTruth.py
Takes the output of VehicleGPStoEGO.py and DataPublisher, which then will interpolate the Pedestrain positions with the tracks to get set of tracks for comparison.

### VehicleGPStoEGO.py
Takes the GPS log file from the Ego Vehicle as output by data_publisher ros node with the added line and the GPS log file from the Target's GPS and exports a csv. Contains EGo frame of refference pedestrian locations.

### data_publisher ROS Node
Takes an ASCII export from Vehicle Spy that was run through FileTransformer.py and outputs a GPS log of the vehicle in CSV format, and a track log in CSV format, as well as its other duties to the ROS environment. Please add a line at the top of both files with the date and time in the format:
Start Time: 2024-10-29 17:37:30:018085

This is needed as the exports do not export the decimal point for seconds and as such we miss the milliseconds

Sensor Ids: 0 - Infra, 1 - LRR, 2 - FCM, 3- SRRLF, 4 - SRRRF

### FileTransformer.py
Takes an ASCII Export from Vehicle and removes all of the unecessary information and outputs the required information.

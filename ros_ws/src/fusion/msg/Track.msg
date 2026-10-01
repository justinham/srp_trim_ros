# Header
builtin_interfaces/Time timestamp #Should be when data was recorded or as close too it
int8 sensor_id
int8 track_id
uint64 u_id # A uniqe ID typically assigned by the Publisher of said data into the ROS environment

uint8 track_status # 1 - New Object, 2 - Latent Track not measured this cycle, 3 - Measured this cycle
uint8 confidence # 0 - Highly Speculative, 1 - Speculative, 2 - Confident, 3 - Highly Confident

# Object Type
bool object_type # 0 - Vehicle, 1 - Pedestrain/VRU

# Vehicle Information Relative to EGO
float32 lat
float32 lon

float32 lat_vel
float32 lon_vel 

float32 theta 0# Orientation/Direction
bool theta_valid False
float64 theta_cov 100

float64[16] covariance
# covariance[0] = lat cov
# covariance[5] = lon cov
# covariance[10] = lat vel cov
# covariance[15] = lon vel cov

float32 length # -1 means invalid, Comes from LRR
float32 track_width # -1 means invalid, Comes from FCM

uint64 carla_id # only used during simulation

# FCM
uint8 turn_signal_status # 0 - Unknown
uint8 brake_light_status # 0 - Unknown

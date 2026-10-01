# Header
builtin_interfaces/Time timestamp #Should be when data was recorded or as close too it
int32 object_id

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

uint64[] carla_ids #Only used in simulation
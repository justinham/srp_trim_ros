# Header
builtin_interfaces/Time timestamp
fusion/RemoteObjectState[] states

int32 object_id

float32 width
float32 hieght
bool theta_valid

# Object Type
bool object_type
int8 object_class_count 0

float32 r
float32 g
float32 b

# Information regarding tracks
uint8[] tracking_sensors
uint8[] tracking_sensors_track_id

fusion/Track[] tracks_to_refine

# Infrastructure
bool tracked_by_infrastructure
int32 infrastructure_track_id

uint64[] carla_ids # Only used in simulation

int8 delay
builtin_interfaces/Time last_seen
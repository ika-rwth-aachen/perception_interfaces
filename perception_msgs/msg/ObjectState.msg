std_msgs/Header header

# to ensure that the model_id is unique, all existing models an their corresponding IDs are listed here:
#   EGO:           1
#   EGORWS:        2
#   ISCACTR:       16
#   HEXAMOTION:    17
#   TRAFFICLIGHT:  20
uint8 model_id

# IDs of the sensors having measured this state
# In case of a fused object, this vector will have multiple entries
uint64[] sensor_id

# continuous state vector (N)
float64[] continuous_state

# discrete state vector (M) for discrete int/bool/string quantitites
int64[] discrete_state

# continuous state covariance matrix (N*N flattened)
# CONTINUOUS_STATE_COVARIANCE_INVALID (-1): state value is invalid / not set
# CONTINUOUS_STATE_COVARIANCE_UNKNOWN (max): state value is set, but covariance is unknown
float64[] continuous_state_covariance

# classification incl. probabilities
ObjectClassification[] classifications

# reference point for object position
ObjectReferencePoint reference_point

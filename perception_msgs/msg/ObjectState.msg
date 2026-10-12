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

# continuous state covariance matrix (N*N flattened); empty array is equivalent to diagonal of `CONTINUOUS_STATE_COVARIANCE_INVALID`
float64 CONTINUOUS_STATE_COVARIANCE_INIT=0.0 # initial value of non-diagonal terms
float64 CONTINUOUS_STATE_COVARIANCE_INVALID=-1.0 # state value invalid or not set
float64 CONTINUOUS_STATE_COVARIANCE_UNKNOWN=1.7976931348623157e+308 # state value set, but covariance unknown (float64 max)
float64[] continuous_state_covariance

# classification incl. probabilities
ObjectClassification[] classifications

# reference point for object position
ObjectReferencePoint reference_point

std_msgs/Header header

# Registered state model IDs:
#   EGO:           1
#   EGORWS:        2
#   ISCACTR:       16
#   HEXAMOTION:    17
#   BOX2D:         22
uint8 model_id

# IDs of the sensors having measured this state
# In case of a fused object, this vector will have multiple entries
uint64[] sensor_id

# continuous state vector (N); NaN marks a component with no available estimate
float64[] continuous_state

# discrete state vector (M) for discrete int/bool/string quantitites
int64[] discrete_state

# continuous state covariance matrix (N*N flattened); empty means no covariance provided
# Covariance availability does not determine whether a component has an estimate.
float64 CONTINUOUS_STATE_COVARIANCE_INIT=0.0 # initial value of non-diagonal terms
float64 CONTINUOUS_STATE_COVARIANCE_INVALID=-1.0 # covariance not provided or invalid
float64 CONTINUOUS_STATE_COVARIANCE_UNKNOWN=1.7976931348623157e+308 # uncertainty unknown (float64 max)
float64[] continuous_state_covariance

# classification incl. probabilities
ObjectClassification[] classifications

# reference point for object position
ObjectReferencePoint reference_point

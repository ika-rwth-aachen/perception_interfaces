# Message for the reference point of an objects position

# possible reference points
# As default the GEOMETRIC_CENTER is used
# This is the geometric center center center point
uint8 GEOMETRIC_CENTER=0
uint8 BACK=1
uint8 BACK_LEFT=2
uint8 LEFT=3
uint8 FRONT_LEFT=4
uint8 FRONT=5
uint8 FRONT_RIGHT=6
uint8 RIGHT=7
uint8 BACK_RIGHT=8

# additional reference points for vehicles
uint8 GRAVITY_CENTER=10
uint8 REAR_AXLE_GROUND=11
uint8 UNKNOWN=100
uint8 UNKNOWN_EDGE=101
uint8 UNKNOWN_CORNER=102

# the actual value
uint8 value

# translation from given reference point to geometric center
# This vector is given relative to a Cartesian coordinate system which is located in the given reference point.
# The x-axis is parallel to the longitudinal axis of the vehicle!
geometry_msgs/Vector3 translation_to_geometric_center
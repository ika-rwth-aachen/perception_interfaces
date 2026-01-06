###########################################################################################################################################################
#  EGO Bicycle Model with rear wheel steering (EGORWS)
###########################################################################################################################################################
uint8 MODEL_ID=2
uint8 CONTINUOUS_STATE_SIZE=13
uint8 DISCRETE_STATE_SIZE=4

#   The EGORWS state model (N=13 & M=4) has the following entries
#   continuous_state:
uint8 X=0                           # x position of reference point in header frame [m]
uint8 Y=1                           # y position of reference point in header frame [m]
uint8 Z=2                           # z position of reference point in header frame [m]
uint8 VEL_LON=3                     # longitudinal velocity in object frame [m/s]
uint8 VEL_LAT=4                     # lateral velocity in object frame [m/s]
uint8 ACC_LON=5                     # longitudinal acceleration in object frame [m/s^2]
uint8 ACC_LAT=6                     # lateral acceleration in object frame [m/s^2]
uint8 ROLL=7                        # roll angle in header frame [rad] [-pi, +pi)
uint8 PITCH=8                       # pitch angle in header frame [rad] [-pi, +pi)
uint8 YAW=9                         # yaw angle in header frame [rad] [-pi, +pi)
uint8 YAW_RATE=10                   # yaw angle rate [rad/s]
uint8 STEERING_ANGLE_FRONT=11       # steering angle of front wheels [rad]
uint8 STEERING_ANGLE_REAR=12        # steering angle of rear wheels [rad]
#   discrete_state:
uint8 STANDSTILL=0                  # flag indicating standstill of the vehicle
uint8 TURN_INDICATOR=1              # turn indicator state (0=unknown, 1=off, 2=left, 3=right, 4=hazard)
uint8 BRAKE_LIGHT=2                 # brake light state (0=unknown, 1=off, 2=on)
uint8 REVERSE_LIGHT=3               # reverse light state (0=unknown, 1=off, 2=on)

# discrete state enums
int64 TURN_INDICATOR_UNKNOWN=0
int64 TURN_INDICATOR_OFF=1
int64 TURN_INDICATOR_LEFT=2
int64 TURN_INDICATOR_RIGHT=3
int64 TURN_INDICATOR_HAZARD=4

int64 LIGHT_UNKNOWN=0
int64 LIGHT_OFF=1
int64 LIGHT_ON=2
###########################################################################################################################################################

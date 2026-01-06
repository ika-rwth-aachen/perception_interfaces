###########################################################################################################################################################
#   ISCACTR-extent
###########################################################################################################################################################
uint8 MODEL_ID=16
uint8 CONTINUOUS_STATE_SIZE=12
uint8 DISCRETE_STATE_SIZE=3

#   The ISCACTR state model with extent (N=12 & M=3) has the following entries
#   continuous_state:
uint8 X=0          # x position of reference point in header frame [m]
uint8 Y=1          # y position of reference point in header frame [m]
uint8 Z=2          # z position of reference point in header frame [m]
uint8 VEL_LON=3    # longitudinal velocity in object frame [m/s]
uint8 VEL_LAT=4    # lateral velocity in object frame [m/s]
uint8 ACC_LON=5    # longitudinal acceleration in object frame [m/s^2]
uint8 ACC_LAT=6    # lateral acceleration in object frame [m/s^2]
uint8 YAW=7        # yaw angle in header frame [rad] (-pi, +pi)
uint8 YAW_RATE=8   # yaw angle rate [rad/s]
uint8 WIDTH=9      # width in object frame [m]
uint8 LENGTH=10    # length in object frame [m]
uint8 HEIGHT=11    # height in object frame [m]
#   discrete_state:
uint8 TURN_INDICATOR=0  # turn indicator state (0=unknown, 1=off, 2=left, 3=right, 4=hazard)
uint8 BRAKE_LIGHT=1     # brake light state (0=unknown, 1=off, 2=on)
uint8 REVERSE_LIGHT=2   # reverse light state (0=unknown, 1=off, 2=on)

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

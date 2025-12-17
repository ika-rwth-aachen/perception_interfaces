###########################################################################################################################################################
#   HEXAMOTION-extent
###########################################################################################################################################################
uint8 MODEL_ID=17
uint8 CONTINUOUS_STATE_SIZE=16
uint8 DISCRETE_STATE_SIZE=1

#   The HEXAMOTION state model (N=16 & M=1) has the following entries
#   continuous_state:
uint8 X=0             # x position of reference point in header frame [m]
uint8 Y=1             # y position of reference point in header frame [m]
uint8 Z=2             # z position of reference point in header frame [m]
uint8 VEL_LON=3       # longitudinal velocity in object frame [m/s]
uint8 VEL_LAT=4       # lateral velocity in object frame [m/s]
uint8 ACC_LON=5       # longitudinal acceleration in object frame [m/s^2]
uint8 ACC_LAT=6       # lateral acceleration in object frame [m/s^2]
uint8 ROLL=7          # roll angle in header frame [rad] (-pi, +pi)
uint8 PITCH=8         # pitch angle in header frame [rad] (-pi, +pi)
uint8 YAW=9           # yaw angle in header frame [rad] (-pi, +pi)
uint8 ROLL_RATE=10    # roll angle rate [rad/s]
uint8 PITCH_RATE=11   # pitch angle rate [rad/s]
uint8 YAW_RATE=12     # yaw angle rate [rad/s]
uint8 LENGTH=13       # length in object frame [m]
uint8 WIDTH=14        # width in object frame [m]
uint8 HEIGHT=15       # height in object frame [m]
#   discrete_state:
uint8 TURN_INDICATOR=0  # turn indicator state (0=unknown, 1=off, 2=left, 3=right, 4=hazard)

# discrete state enums
uint8 TURN_INDICATOR_UNKNOWN=0
uint8 TURN_INDICATOR_OFF=1
uint8 TURN_INDICATOR_LEFT=2
uint8 TURN_INDICATOR_RIGHT=3
uint8 TURN_INDICATOR_HAZARD=4
###########################################################################################################################################################
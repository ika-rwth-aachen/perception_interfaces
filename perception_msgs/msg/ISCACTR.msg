###########################################################################################################################################################
#   ISCACTR-extent
###########################################################################################################################################################
uint8 MODEL_ID=16
uint8 CONTINUOUS_STATE_SIZE=12
uint8 DISCRETE_STATE_SIZE=0

#   The ISCACTR state model with extent (N=12) has the following entries
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
#   currently none
###########################################################################################################################################################
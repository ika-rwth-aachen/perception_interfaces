###########################################################################################################################################################
#   EGO Bicycle Model with Ackermann steering
###########################################################################################################################################################
uint8 MODEL_ID=1
uint8 CONTINUOUS_STATE_SIZE=13
uint8 DISCRETE_STATE_SIZE=1

#   The EGO state model (N=13 & M=1) has the following entries
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
uint8 STEERING_ANGLE_ACK=11         # steering angle according to ackermann model [rad]
uint8 STEERING_ANGLE_RATE_ACK=12    # steering angle rate according to ackermann model [rad/s]
#   discrete_state:
uint8 STANDSTILL=0                  # flag indicating standstill of the vehicle
###########################################################################################################################################################

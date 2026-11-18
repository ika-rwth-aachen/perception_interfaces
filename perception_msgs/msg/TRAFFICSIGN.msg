uint8 MODEL_ID = 21
uint8 CONTINUOUS_STATE_SIZE = 8
uint8 DISCRETE_STATE_SIZE = 1

# continuous state
uint8 X = 0       # x position of reference point in header frame [m]
uint8 Y = 1       # y position of reference point in header frame [m]
uint8 Z = 2       # z position of reference point in header frame [m]

uint8 ROLL = 3    # roll angle in header frame [rad]
uint8 PITCH = 4   # pitch angle in header frame [rad]
uint8 YAW = 5     # yaw angle in header frame [rad]

uint8 WIDTH = 6   # sign face width along local y [m]
uint8 HEIGHT = 7  # sign face height along local z [m]

# discrete state
uint8 TYPE = 0

# discrete state enums
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_STOP = 1
uint8 TYPE_GIVE_WAY = 2
uint8 TYPE_PRIORITY_ROAD = 3
uint8 TYPE_PRIORITY_AT_NEXT_INTERSECTION = 4
uint8 TYPE_NO_ENTRY = 5
uint8 TYPE_NO_VEHICLES = 6
uint8 TYPE_NO_GOODS_VEHICLES_OVER_3_5_T = 7
uint8 TYPE_NO_OVERTAKING = 8
uint8 TYPE_NO_OVERTAKING_GOODS_VEHICLES = 9
uint8 TYPE_END_NO_OVERTAKING = 10
uint8 TYPE_END_NO_OVERTAKING_GOODS_VEHICLES = 11
uint8 TYPE_END_ALL_SPEED_AND_OVERTAKING_RESTRICTIONS = 12
uint8 TYPE_SPEED_LIMIT_20_KMH = 13
uint8 TYPE_SPEED_LIMIT_30_KMH = 14
uint8 TYPE_SPEED_LIMIT_50_KMH = 15
uint8 TYPE_SPEED_LIMIT_60_KMH = 16
uint8 TYPE_SPEED_LIMIT_70_KMH = 17
uint8 TYPE_SPEED_LIMIT_80_KMH = 18
uint8 TYPE_SPEED_LIMIT_100_KMH = 19
uint8 TYPE_SPEED_LIMIT_120_KMH = 20
uint8 TYPE_END_SPEED_LIMIT_80_KMH = 21
uint8 TYPE_DANGER = 22
uint8 TYPE_CURVE_LEFT = 23
uint8 TYPE_CURVE_RIGHT = 24
uint8 TYPE_DOUBLE_CURVE_FIRST_LEFT = 25
uint8 TYPE_UNEVEN_ROAD = 26
uint8 TYPE_SLIPPERY_ROAD = 27
uint8 TYPE_ROAD_NARROWS_RIGHT = 28
uint8 TYPE_ROAD_WORKS = 29
uint8 TYPE_TRAFFIC_SIGNALS = 30
uint8 TYPE_PEDESTRIANS = 31
uint8 TYPE_CHILDREN = 32
uint8 TYPE_CYCLISTS = 33
uint8 TYPE_ICE_OR_SNOW = 34
uint8 TYPE_WILD_ANIMALS = 35
uint8 TYPE_TURN_RIGHT = 36
uint8 TYPE_TURN_LEFT = 37
uint8 TYPE_STRAIGHT_AHEAD = 38
uint8 TYPE_STRAIGHT_OR_RIGHT = 39
uint8 TYPE_STRAIGHT_OR_LEFT = 40
uint8 TYPE_KEEP_RIGHT = 41
uint8 TYPE_KEEP_LEFT = 42
uint8 TYPE_ROUNDABOUT = 43

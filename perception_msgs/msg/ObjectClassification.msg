# One class with its respective probability

# Message for the classification of an object

# Possible Types (enum, to be extended)
uint8 UNCLASSIFIED=0
uint8 PEDESTRIAN=1
uint8 BICYCLE=2
uint8 MOTORBIKE=3
uint8 CAR=4
uint8 TRUCK=5
uint8 VAN=6
uint8 BUS=7
uint8 ANIMAL=8
uint8 ROAD_OBSTACLE=9
uint8 TRAIN=10
uint8 TRAILER=11

# Union message types (use in case detector can not differentiate between classes):
uint8 CAR_UNION=50 # CAR or VAN
uint8 TRUCK_UNION=51 # TRUCK or BUS or TRAIN
uint8 BIKE_UNION=52 # BICYCLE or MOTORBIKE

# If object class is definitely none of the upper, these classes are to be used
uint8 UNKNOWN=100

# classification type
uint8 type

# Class probability [0, 1]
float64 probability

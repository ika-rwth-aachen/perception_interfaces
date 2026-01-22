uint8 UNCLASSIFIED=0    # unknown classification
uint8 PEDESTRIAN=1      # persons on foot (walking, standing, running, sitting, lying) without a dedicated mobility device such as a wheelchair or stroller
uint8 BICYCLE=2         # bicycles and their riders
uint8 MOTORBIKE=3       # --- DEPRECATED; USE MOTORCYCLE
uint8 MOTORCYCLE=3      # motorized two-wheel vehicles and their riders
uint8 CAR=4             # standard passenger cars for personal transport
uint8 TRUCK=5           # --- DEPRECATED; USE UTILITY
uint8 UTILITY=5         # commercial & utility vehicles like trucks, trailers, construction vehicles, agricultural/utility vehicles, and other large road vehicles
uint8 VAN=6             # --- DEPRECATED; USE CAR
uint8 BUS=7             # vehicles designed to carry many passengers such as urban buses, articulated buses, and bus trailers
uint8 ANIMAL=8          # animals that may influence traffic
uint8 ROAD_OBSTACLE=9   # --- DEPRECATED
uint8 TRAIN=10          # --- DEPRECATED; USE UTILITY
uint8 TRAILER=11        # --- DEPRECATED; USE UTILITY
uint8 VRU=12            # vulnerable road users that typically move on sidewalks such as wheelchairs and strollers, except pedestrians
uint8 MICRO=13          # micro-mobility devices typically used on bike lanes or the road (e.g., kick-scooters, Segways, hoverboards) with their riders
uint8 CAR_UNION=50      # --- DEPRECATED; USE CAR
uint8 TRUCK_UNION=51    # --- DEPRECATED; USE UTILITY
uint8 BIKE_UNION=52     # --- DEPRECATED; USE MULTIPLE PROBABILISTIC CLASSIFICATIONS
uint8 UNKNOWN=100       # definitely none of the other defined classes
uint8 type

float64 probability

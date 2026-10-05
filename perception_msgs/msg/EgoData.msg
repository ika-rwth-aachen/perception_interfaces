# Message for the state of the ego vehicle

std_msgs/Header header

# Dynamic motion state estimate of the ego vehicle
ObjectState state

# Planned trajectory of the ego_vehicle
ObjectState[] trajectory_planned

# Past trajectory of the ego_vehicle
ObjectState[] trajectory_past

# Planned route of the ego_vehicle
geometry_msgs/Point[] route_planned

# Parameters
uint64 vehicle_id           # Globally unique vehicle id
float64 length              # value of vehicle length [m]
float64 width               # value of vehicle width [m]
float64 height              # value of vehicle height [m]
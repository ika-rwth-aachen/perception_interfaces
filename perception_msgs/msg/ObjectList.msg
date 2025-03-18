# Message for a list of objects

# header
# frame_id of the reference frame (common: "base_link", "map",...), and timestamp
# The frame is where the position and orientation is published.
# Probably this will be in vehicle or global coordinates and there should be a transform in between
# The timestamp is the time of the newest sample
std_msgs/Header header

# List of objects
Object[] objects

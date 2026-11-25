# Payload layout and acquisition header only; geometry lives in BOX2D.
# Either payload may be empty. mask is mono8 (0 background, 255 object).
# Both cover BOX2D's ROI; their dimensions may differ due to resizing.
sensor_msgs/Image crop
sensor_msgs/Image mask

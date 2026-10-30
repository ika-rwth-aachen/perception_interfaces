# Attribute observations from one source image, including empty observations.
# The header is the source image acquisition time and optical frame.
std_msgs/Header header

# Every detection header must match the array header.
Detection2DAttributes[] detections

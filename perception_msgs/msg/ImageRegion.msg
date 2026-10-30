# An original-image detection with the image crop used for further processing.
# All headers must match the source image acquisition time and optical frame.
vision_msgs/Detection2D detection

# Rectangle actually extracted from the source image, including any padding.
# Nonzero width/height, inside the source dimensions; do_rectify must be false.
# Coordinates refer to the supplied image, whether rectified or unrectified.
sensor_msgs/RegionOfInterest roi

# Nonempty crop of roi, optionally resized without rotation or letterboxing.
# Encoding follows sensor_msgs/Image; consumers specify supported encodings.
# The crop header matches detection.header, not a new acquisition or frame.
sensor_msgs/Image crop

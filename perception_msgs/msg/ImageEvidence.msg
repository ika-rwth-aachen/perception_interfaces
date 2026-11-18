# Message for the evidence of an object in a camera image

# size of the source image [px]
uint32 image_width
uint32 image_height

# bounding box in source image pixel coordinates (x right, y down)
vision_msgs/BoundingBox2D bounding_box

# source image region covered by crop and mask
sensor_msgs/RegionOfInterest roi

# optional pixels, empty data if absent; each may be resized and has the evidence header
sensor_msgs/Image crop  # image crop
sensor_msgs/Image mask  # mono8 instance mask: 0 background, 255 object

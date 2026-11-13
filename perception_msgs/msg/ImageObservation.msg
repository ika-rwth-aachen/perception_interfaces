# source image
std_msgs/Header header  # acquisition time and camera optical frame; unchanged by metric TF
uint64 sensor_id        # camera ID, also present in ObjectState.sensor_id
uint32 image_width     # original image width [px]
uint32 image_height    # original image height [px]

# image geometry
vision_msgs/BoundingBox2D bounding_box  # source pixel centers: x right, y down
sensor_msgs/RegionOfInterest roi       # crop/mask source rectangle; do_rectify=false

# optional pixels: empty data means absent; headers match the source header
# Crop and mask cover roi without rotation; each may be resized independently.
# roi is required inside the source image when either payload is present.
sensor_msgs/Image crop  # image crop in a packed encoding
sensor_msgs/Image mask  # mono8 instance mask: 0 background, 255 object

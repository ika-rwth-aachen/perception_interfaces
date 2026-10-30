# Selected regions from one source image. An empty list means no selected regions.
# The header is the source image acquisition time and optical frame.
std_msgs/Header header

# Full source image dimensions before cropping or resizing.
uint32 image_width
uint32 image_height

# Every region's detection and crop headers must match the array header.
# Detection boxes stay in source-image pixel coordinates.
ImageRegion[] regions

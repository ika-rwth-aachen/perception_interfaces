# Message for the evidence of an object from a single sensor

# acquisition time and sensor frame; unchanged by transformations of the object
std_msgs/Header header

# ID of the sensor, also contained in object.state.sensor_id
uint64 sensor_id

# evidence of exactly one of the following kinds
ImageEvidence[<=1] image
PointCloudEvidence[<=1] point_cloud

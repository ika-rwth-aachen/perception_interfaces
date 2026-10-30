# Copyright (c) 2026 Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

"""Check generated image interfaces and their serialized wire representation."""

from perception_msgs.msg import (
    Detection2DAttributes,
    Detection2DAttributesArray,
    ImageRegion,
    ImageRegionArray,
    ObjectAttribute,
)
from rclpy.serialization import deserialize_message, serialize_message
from vision_msgs.msg import ObjectHypothesis, ObjectHypothesisWithPose


def round_trip(message):
    """Serialize and deserialize using the generated ROS type support."""
    return deserialize_message(serialize_message(message), type(message))


def test_resized_region():
    """Keep source geometry, identity, and padded image bytes across transport."""
    message = ImageRegionArray(image_width=1280, image_height=720)
    message.header.frame_id = 'camera_optical'
    message.header.stamp.sec = 42
    region = ImageRegion()
    region.detection.header = message.header
    region.detection.id = 'session/camera/123'
    region.detection.bbox.center.position.x = 100.0
    region.detection.bbox.center.position.y = 80.0
    region.detection.bbox.size_x = 80.0
    region.detection.bbox.size_y = 40.0
    region.roi.x_offset, region.roi.y_offset = 50, 50
    region.roi.width, region.roi.height = 100, 60
    region.crop.header = message.header
    region.crop.encoding = 'bgr8'
    region.crop.width, region.crop.height = 10, 6
    region.crop.step = 32  # Non-contiguous rows are legal sensor_msgs/Image data.
    region.crop.data = bytes(range(192))
    message.regions = [region]
    restored = round_trip(message)
    assert restored == message
    assert restored.regions[0].roi.width != restored.regions[0].crop.width
    assert restored.regions[0].detection.id == 'session/camera/123'
    assert round_trip(ImageRegionArray()).regions == []


def test_independent_attributes():
    """Preserve primary class, independent left/right states, and unknown values."""
    message = Detection2DAttributesArray()
    message.header.frame_id = 'camera_optical'
    observation = Detection2DAttributes()
    observation.detection.header = message.header
    observation.detection.id = 'session/camera/car'
    classification = ObjectHypothesisWithPose()
    classification.hypothesis = ObjectHypothesis(class_id='CAR', score=0.99)
    observation.detection.results = [classification]
    observation.attributes = [
        ObjectAttribute(name='vehicle.indicator.left', hypotheses=[
            ObjectHypothesis(class_id='flashing', score=0.9),
            ObjectHypothesis(class_id='off', score=0.1),
        ]),
        ObjectAttribute(name='vehicle.indicator.right'),
    ]
    message.detections = [observation]
    restored = round_trip(message)
    assert restored == message
    assert restored.detections[0].detection.results[0].hypothesis.class_id == 'CAR'
    assert restored.detections[0].attributes[1].hypotheses == []
    assert round_trip(Detection2DAttributesArray()).detections == []

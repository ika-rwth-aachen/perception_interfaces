# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT
"""Contracts for dimensionality, source geometry, payload layout and compatibility."""

import copy
import math

import pytest
from perception_msgs.msg import (
    BOX2D,
    HEXAMOTION,
    TRAFFICLIGHT,
    ObjectWithObservation,
    ImageObservation,
    PointCloudObservation,
    ObjectClassification,
)
from perception_msgs_utils import (
    initialize_state,
    sanity_check_object_observation,
    to_metric_object,
    to_legacy_metric_light,
    traffic_light_class,
)


def image_object():
    entry = ObjectWithObservation()
    entry.object.id = 2**40 + 7
    entry.object.existence_probability = 0.75
    initialize_state(entry.object, BOX2D.MODEL_ID)
    state = entry.object.state
    state.header.frame_id = "camera_optical"
    state.header.stamp.sec = 5
    state.sensor_id = [11]
    state.continuous_state = [320.0, 200.0, 60.0, 180.0, 0.0]
    state.discrete_state = [640, 480, 290, 110, 60, 180]
    entry.observation.image = [ImageObservation()]
    pixels = entry.observation.image[0].crop
    pixels.header = copy.deepcopy(state.header)
    pixels.width, pixels.height, pixels.step, pixels.encoding = 30, 90, 90, "bgr8"
    pixels.data = bytes([7]) * 8100
    return entry


def test_resized_payload_preserves_original_pixel_geometry():
    entry = image_object()
    sanity_check_object_observation(entry)
    assert list(entry.object.state.discrete_state[4:]) == [60, 180]
    assert entry.observation.image[0].crop.width == 30
    assert entry.object.id > 2**32


@pytest.mark.parametrize(
    "change",
    [
        lambda e: setattr(e.object, "existence_probability", math.nan),
        lambda e: setattr(e.object.state, "model_id", HEXAMOTION.MODEL_ID),
        lambda e: setattr(e.object.state, "sensor_id", [11, 12]),
        lambda e: e.object.state.discrete_state.__setitem__(2, 1000),
        lambda e: e.object.state.continuous_state.__setitem__(2, math.nan),
        lambda e: setattr(e.observation.image[0].crop, "step", 89),
        lambda e: setattr(e.observation.image[0].crop.header, "frame_id", "other_camera"),
        lambda e: setattr(e.observation, "point_cloud", [PointCloudObservation()]),
        lambda e: setattr(e.observation, "image", []),
    ],
)
def test_rejects_invalid_observation(change):
    entry = image_object()
    change(entry)
    with pytest.raises(ValueError):
        sanity_check_object_observation(entry)


def test_cloud_requires_metric_state_and_valid_fields():
    entry = image_object()
    entry.observation.image = []
    entry.observation.point_cloud = [PointCloudObservation()]
    points = entry.observation.point_cloud[0].points
    points.header = copy.deepcopy(entry.object.state.header)
    with pytest.raises(ValueError):
        sanity_check_object_observation(entry)
    initialize_state(entry.object, HEXAMOTION.MODEL_ID, math.nan)
    sanity_check_object_observation(entry)
    points.width, points.height, points.point_step, points.row_step = 1, 1, 12, 12
    points.data = bytes(11)
    with pytest.raises(ValueError):
        sanity_check_object_observation(entry)


def test_metric_projection_preserves_confidence_identity_and_position_covariance():
    entry = image_object()
    obj = entry.object
    initialize_state(obj, TRAFFICLIGHT.MODEL_ID, math.nan)
    obj.state.continuous_state = [1.0, 2.0, 3.0]
    obj.state.discrete_state = [1, 0]
    obj.state.continuous_state_covariance = [1.0, 0.2, 0.0, 0.2, 2.0, 0.0, 0.0, 0.0, 3.0]
    obj.state.classifications = [ObjectClassification(type=traffic_light_class("left,red,straight"), probability=0.9)]
    metric = to_metric_object(obj)
    assert metric.state.model_id == HEXAMOTION.MODEL_ID
    assert math.isnan(metric.state.continuous_state[3])
    assert metric.state.continuous_state_covariance[17] == 2.0
    assert metric.state.continuous_state_covariance[51] == -1.0
    restored = to_legacy_metric_light(metric)
    assert restored.id == obj.id and restored.existence_probability == 0.75
    assert list(restored.state.continuous_state_covariance) == list(obj.state.continuous_state_covariance)
    assert restored.state.classifications[0].type == 109
    assert metric.state.classifications[0].probability == 0.9


def test_pixels_cannot_be_cast_into_metric_coordinates():
    with pytest.raises(ValueError):
        to_metric_object(image_object().object)


def test_taxonomy_covers_classifier_compounds_without_color_collapse():
    assert traffic_light_class("left,red") != traffic_light_class("red")
    assert traffic_light_class("red,right,straight") == 113
    assert traffic_light_class("unknown") != traffic_light_class("off")

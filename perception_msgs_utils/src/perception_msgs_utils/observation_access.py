# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT
"""Observation validation and explicit compatibility projections."""

import copy
import math

from perception_msgs.msg import BOX2D, HEXAMOTION, TRAFFICLIGHT, TRAFFICSIGN, ObjectWithObservation, ImageObservation
from .utils import get_continuous_state_size, get_discrete_state_size
from .class_taxonomy import traffic_light_class


def sanity_check_box2d(state):
    if (
        state.model_id != BOX2D.MODEL_ID
        or len(state.continuous_state) != 5
        or len(state.discrete_state) != 6
        or not state.header.frame_id
        or state.header.stamp.sec < 0
        or state.header.stamp.nanosec >= 1000000000
        or len(state.sensor_id) != 1
    ):
        raise ValueError("BOX2D needs one acquisition sensor, five coordinates and six dimensions")
    c, d = state.continuous_state, state.discrete_state
    if (
        not all(math.isfinite(v) for v in c)
        or c[2] <= 0
        or c[3] <= 0
        or not 0 < d[0] <= 0xFFFFFFFF
        or not 0 < d[1] <= 0xFFFFFFFF
        or min(d[2:]) < 0
        or d[2] + d[4] > d[0]
        or d[3] + d[5] > d[1]
        or len(state.continuous_state_covariance) not in (0, 25)
    ):
        raise ValueError("Invalid BOX2D geometry or covariance")


def _check_pixels(pixels, state, mask):
    if not pixels.data:
        if pixels.width or pixels.height or pixels.step:
            raise ValueError("Absent image must have zero dimensions and stride")
        return
    if (
        pixels.header != state.header
        or not pixels.width
        or not pixels.height
        or not state.discrete_state[4]
        or not state.discrete_state[5]
        or pixels.step * pixels.height != len(pixels.data)
        or pixels.is_bigendian not in (0, 1)
    ):
        raise ValueError("Invalid image storage or acquisition header")
    import re

    sizes = {
        "mono8": 1,
        "mono16": 2,
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "bgr16": 6,
        "rgb16": 6,
        "bgra16": 8,
        "rgba16": 8,
        "bayer_rggb8": 1,
        "bayer_bggr8": 1,
        "bayer_gbrg8": 1,
        "bayer_grbg8": 1,
        "bayer_rggb16": 2,
        "bayer_bggr16": 2,
        "bayer_gbrg16": 2,
        "bayer_grbg16": 2,
        "yuv422": 2,
        "yuv422_yuy2": 2,
    }
    size = sizes.get(pixels.encoding)
    match = re.fullmatch(r"(8[US]|16[US]|32[USF]|64F)C([1-9][0-9]*)", pixels.encoding)
    if match:
        size = int(match[1][:-1]) // 8 * int(match[2])
    if size is None or pixels.step < pixels.width * size or (mask and pixels.encoding != "mono8"):
        raise ValueError("Invalid image encoding or stride; mask must be mono8")


def sanity_check_object_observation(entry):
    observation, state = entry.observation, entry.object.state
    if not math.isfinite(entry.object.existence_probability) or not 0 <= entry.object.existence_probability <= 1:
        raise ValueError("Invalid detector existence probability")
    if any(not math.isfinite(c.probability) or not 0 <= c.probability <= 1 for c in state.classifications):
        raise ValueError("Invalid semantic probability")
    if len(observation.image) + len(observation.point_cloud) != 1:
        raise ValueError("Exactly one image or point cloud observation is required")
    if observation.image:
        sanity_check_box2d(state)
        _check_pixels(observation.image[0].crop, state, False)
        _check_pixels(observation.image[0].mask, state, True)
        return
    if (
        state.model_id not in (1, 2, 16, 17, 20, 21)
        or len(state.continuous_state) != get_continuous_state_size(state.model_id)
        or len(state.discrete_state) != get_discrete_state_size(state.model_id)
        or len(state.sensor_id) != 1
        or not state.header.frame_id
        or state.header.stamp.sec < 0
        or state.header.stamp.nanosec >= 1000000000
    ):
        raise ValueError("Point observations require a metric state and one acquisition sensor")
    if len(state.continuous_state_covariance) not in (0, len(state.continuous_state) ** 2):
        raise ValueError("Invalid metric covariance size")
    points = observation.point_cloud[0].points
    if (
        points.header != state.header
        or points.row_step * points.height != len(points.data)
        or points.row_step < points.point_step * points.width
        or (points.width and (not points.point_step or not points.height or not points.fields))
    ):
        raise ValueError("Invalid cloud layout or acquisition header")
    sizes = (0, 1, 1, 2, 2, 4, 4, 4, 8)
    for field in points.fields:
        if (
            not 0 < field.datatype < len(sizes)
            or not field.count
            or not field.name
            or field.offset + field.count * sizes[field.datatype] > points.point_step
        ):
            raise ValueError("Invalid point field")


def to_image_observation(source):
    if len(source.evidence) != 1 or len(source.evidence[0].image) != 1 or source.evidence[0].point_cloud:
        raise ValueError("Cannot implicitly select a fused observation")
    evidence = source.evidence[0]
    image = evidence.image[0]
    result = ObjectWithObservation()
    result.object = copy.deepcopy(source.object)
    state = result.object.state
    state.model_id = BOX2D.MODEL_ID
    state.header = copy.deepcopy(evidence.header)
    state.sensor_id = [evidence.sensor_id]
    box = image.bounding_box
    state.continuous_state = [box.center.position.x, box.center.position.y, box.size_x, box.size_y, box.center.theta]
    roi = image.roi
    state.discrete_state = [image.image_width, image.image_height, roi.x_offset, roi.y_offset, roi.width, roi.height]
    state.continuous_state_covariance = []
    result.object.state_history = []
    result.object.state_predictions = []
    result.observation.image = [ImageObservation(crop=copy.deepcopy(image.crop), mask=copy.deepcopy(image.mask))]
    sanity_check_object_observation(result)
    return result


def best_class(state):
    return max(state.classifications, key=lambda c: c.probability).type if state.classifications else 0


def legacy_light_color(class_id):
    if class_id == 106:
        return TRAFFICLIGHT.STATE_GREEN
    if class_id == 105:
        return TRAFFICLIGHT.STATE_YELLOW
    if class_id == 103:
        return TRAFFICLIGHT.STATE_OFF
    if class_id == 107:
        return TRAFFICLIGHT.STATE_RED_YELLOW
    if class_id == 104 or 108 <= class_id <= 114:
        return TRAFFICLIGHT.STATE_RED
    return TRAFFICLIGHT.STATE_UNKNOWN


def to_metric_object(source):
    if source.state.model_id == BOX2D.MODEL_ID:
        raise ValueError("Pixel geometry requires localization before metric conversion")
    if source.state.model_id not in (TRAFFICLIGHT.MODEL_ID, TRAFFICSIGN.MODEL_ID):
        return copy.deepcopy(source)
    old = source.state
    indices = [0, 1, 2, 7, 8, 9, 14, 15] if old.model_id == TRAFFICSIGN.MODEL_ID else [0, 1, 2]
    if len(old.continuous_state) != len(indices):
        raise ValueError("Invalid legacy metric state")
    result = copy.deepcopy(source)
    state = result.state
    state.model_id = HEXAMOTION.MODEL_ID
    state.continuous_state = [math.nan] * 16
    state.discrete_state = [0] * 3
    for i, target in enumerate(indices):
        state.continuous_state[target] = old.continuous_state[i]
    state.continuous_state_covariance = []
    if old.continuous_state_covariance:
        n = len(indices)
        if len(old.continuous_state_covariance) != n * n:
            raise ValueError("Invalid legacy covariance")
        covariance = [0.0] * 256
        for i in range(16):
            covariance[i * 16 + i] = -1.0
        for i, ti in enumerate(indices):
            for j, tj in enumerate(indices):
                covariance[ti * 16 + tj] = old.continuous_state_covariance[i * n + j]
        state.continuous_state_covariance = covariance
    return result


def to_legacy_metric_light(source):
    if source.state.model_id == TRAFFICLIGHT.MODEL_ID:
        return copy.deepcopy(source)
    state = source.state
    if (
        state.model_id != HEXAMOTION.MODEL_ID
        or len(state.continuous_state) != 16
        or not 101 <= best_class(state) <= 114
    ):
        raise ValueError("Expected generic 3D traffic light")
    result = copy.deepcopy(source)
    result.state.model_id = TRAFFICLIGHT.MODEL_ID
    result.state.continuous_state = list(state.continuous_state[:3])
    result.state.discrete_state = [legacy_light_color(best_class(state)), TRAFFICLIGHT.TYPE_DEFAULT]
    result.state.continuous_state_covariance = []
    if len(state.continuous_state_covariance) == 256:
        result.state.continuous_state_covariance = [
            state.continuous_state_covariance[i * 16 + j] for i in range(3) for j in range(3)
        ]
    return result

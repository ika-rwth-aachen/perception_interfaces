# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

"""
Image observation access functions and metric-state availability.
"""

import math
import re

from typing import Sequence, TypeVar, Union

from perception_msgs.msg import ObjectState, Object, EgoData, ImageObservation
from .utils import get_continuous_state_size
from .state_index import index_x, index_y, index_z, has_x, has_y, has_z

T = TypeVar('T', bound=Union[Object, ObjectState, EgoData])

__all__ = [
    'has_valid_position', 'has_metric_state', 'is_image_only',
    'sanity_check_image_observation', 'sanity_check_image_observations',
    'get_image_observations', 'set_image_observations', 'add_image_observation'
]


def _state(obj: T) -> ObjectState:
    return obj if isinstance(obj, ObjectState) else obj.state


def _state_size(state: ObjectState) -> int:
    n = get_continuous_state_size(state.model_id)
    if len(state.continuous_state) != n or (state.continuous_state_covariance and
                                           len(state.continuous_state_covariance) != n * n):
        raise ValueError('Invalid state storage for metric availability check')
    return n


def _measured(state: ObjectState, n: int, index: int) -> bool:
    if not state.continuous_state_covariance:
        return False
    variance = state.continuous_state_covariance[n * index + index]
    return math.isfinite(state.continuous_state[index]) and math.isfinite(variance) and variance >= 0


def has_valid_position(obj: T) -> bool:
    """Check if all position components are measured."""
    state = _state(obj)
    n = _state_size(state)
    return (has_x(state.model_id) and has_y(state.model_id) and has_z(state.model_id)
            and all(_measured(state, n, index(state.model_id)) for index in (index_x, index_y, index_z)))


def has_metric_state(obj: T) -> bool:
    """Whether any finite continuous-state component has a valid covariance diagonal."""
    state = _state(obj)
    n = _state_size(state)
    return any(_measured(state, n, i) for i in range(n))


def is_image_only(obj: T) -> bool:
    """Whether no continuous-state component is measured."""
    return not has_metric_state(obj)


def _pixel_bytes(encoding: str) -> int:
    # Match the standard sensor_msgs image encodings, including generic OpenCV types.
    named = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1,
             'rgb16': 6, 'bgr16': 6, 'rgba16': 8, 'bgra16': 8, 'mono16': 2,
             'yuv422': 2, 'yuv422_yuy2': 2}
    if encoding in named:
        return named[encoding]
    if re.fullmatch(r'bayer_(rggb|bggr|gbrg|grbg)(8|16)', encoding):
        return 2 if encoding.endswith('16') else 1
    match = re.fullmatch(r'(8[US]|16[US]|32[SF]|64F)C([1-9][0-9]*)', encoding)
    if match:
        return int(match[1][:-1]) // 8 * int(match[2])
    raise ValueError('Unsupported image encoding: ' + encoding)


def sanity_check_image_observation(image: ImageObservation) -> None:
    """Validate geometry and byte layout without copying or scanning pixel data."""
    box = image.bounding_box
    center = getattr(box.center, 'position', box.center)
    if (not image.header.frame_id or image.header.stamp.sec < 0 or image.header.stamp.nanosec >= 1000000000
            or not image.image_width or not image.image_height
            or not all(math.isfinite(v) for v in (center.x, center.y, box.center.theta, box.size_x, box.size_y))
            or box.size_x <= 0 or box.size_y <= 0):
        raise ValueError('Invalid image observation header or bounding box')
    roi = image.roi
    rectangle = roi.width > 0 and roi.height > 0
    if (roi.do_rectify
            or (rectangle and (roi.x_offset + roi.width > image.image_width
                               or roi.y_offset + roi.height > image.image_height))
            or (not rectangle and any((roi.width, roi.height, roi.x_offset, roi.y_offset)))):
        raise ValueError('Invalid source image ROI')
    for pixels, mask in ((image.crop, False), (image.mask, True)):
        if len(pixels.data) == 0:
            if pixels.width or pixels.height or pixels.step:
                raise ValueError('Empty image payload has nonzero dimensions')
            continue
        if (not rectangle or not pixels.width or not pixels.height or pixels.header != image.header
                or pixels.is_bigendian > 1 or (mask and pixels.encoding != 'mono8')):
            raise ValueError('Invalid crop or mask metadata')
        if (pixels.step < pixels.width * _pixel_bytes(pixels.encoding)
                or pixels.step * pixels.height != len(pixels.data)):
            raise ValueError('Invalid crop or mask buffer layout')


def _source(image: ImageObservation):
    return image.sensor_id, image.header.frame_id, image.header.stamp.sec, image.header.stamp.nanosec


def sanity_check_image_observations(obj: T) -> None:
    """Check observation provenance and reject duplicate source/acquisition pairs."""
    state = _state(obj)
    sources = set()
    for image in state.image_observations:
        sanity_check_image_observation(image)
        if image.sensor_id not in state.sensor_id:
            raise ValueError('Image observation sensor is absent from state.sensor_id')
        source = _source(image)
        if source in sources:
            raise ValueError('Duplicate image observation for the same source and acquisition')
        sources.add(source)


def get_image_observations(obj: T) -> Sequence[ImageObservation]:
    """Get image observations without copying their pixel buffers."""
    return _state(obj).image_observations


def set_image_observations(obj: T, images: Sequence[ImageObservation]) -> None:
    """Replace observations, retaining existing sensor provenance; validate first."""
    state = _state(obj)
    candidate = ObjectState()
    candidate.sensor_id = list(state.sensor_id)
    candidate.image_observations = list(images)
    for image in candidate.image_observations:
        if image.sensor_id not in candidate.sensor_id:
            candidate.sensor_id.append(image.sensor_id)
    sanity_check_image_observations(candidate)
    state.image_observations = candidate.image_observations
    state.sensor_id = candidate.sensor_id


def add_image_observation(obj: T, image: ImageObservation) -> None:
    """Append an observation and camera provenance, preserving the object's ID."""
    state = _state(obj)
    sanity_check_image_observation(image)
    if any(_source(previous) == _source(image) for previous in state.image_observations):
        raise ValueError('Duplicate image observation for the same source and acquisition')
    state.image_observations.append(image)
    if image.sensor_id not in state.sensor_id:
        state.sensor_id.append(image.sensor_id)

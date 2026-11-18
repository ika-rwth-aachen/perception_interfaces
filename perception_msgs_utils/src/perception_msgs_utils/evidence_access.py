# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

"""
Evidence access functions.

This module provides functions to validate and add the sensor evidence of objects.
"""

import math

from perception_msgs.msg import Evidence, ImageEvidence, ObjectWithEvidence, PointCloudEvidence
from std_msgs.msg import Header

__all__ = ['sanity_check_image_evidence', 'sanity_check_point_cloud_evidence', 'sanity_check_evidence',
           'sanity_check_object_evidence', 'add_evidence']


def sanity_check_image_evidence(image: ImageEvidence, header: Header) -> None:
    """Check if image evidence is valid.

    Args:
        image: ImageEvidence instance to check
        header: Header of the enclosing evidence

    Raises:
        ValueError: If the image evidence is invalid
    """
    box = image.bounding_box
    if not image.image_width or not image.image_height:
        raise ValueError('Image evidence without image size')
    values = (box.center.position.x, box.center.position.y, box.center.theta, box.size_x, box.size_y)
    if not all(math.isfinite(v) for v in values) or box.size_x <= 0 or box.size_y <= 0:
        raise ValueError('Invalid image evidence bounding box')
    roi = image.roi
    if roi.x_offset + roi.width > image.image_width or roi.y_offset + roi.height > image.image_height:
        raise ValueError('Image evidence ROI exceeds the source image')
    for pixels in (image.crop, image.mask):
        if len(pixels.data) == 0:
            continue
        if not roi.width or not roi.height or pixels.header != header or \
                pixels.step * pixels.height != len(pixels.data):
            raise ValueError('Invalid image evidence crop or mask')
    if len(image.mask.data) and (image.mask.encoding != 'mono8' or image.mask.step < image.mask.width):
        raise ValueError('Image evidence mask must be mono8')


def sanity_check_point_cloud_evidence(point_cloud: PointCloudEvidence, header: Header) -> None:
    """Check if point cloud evidence is valid.

    Args:
        point_cloud: PointCloudEvidence instance to check
        header: Header of the enclosing evidence

    Raises:
        ValueError: If the point cloud evidence is invalid
    """
    points = point_cloud.points
    if points.header != header or points.row_step * points.height != len(points.data):
        raise ValueError('Invalid point cloud evidence')


def sanity_check_evidence(evidence: Evidence) -> None:
    """Check if evidence is valid.

    Args:
        evidence: Evidence instance to check

    Raises:
        ValueError: If the evidence is invalid
    """
    if not evidence.header.frame_id or len(evidence.image) + len(evidence.point_cloud) != 1:
        raise ValueError('Evidence needs a frame_id and exactly one kind')
    for image in evidence.image:
        sanity_check_image_evidence(image, evidence.header)
    for point_cloud in evidence.point_cloud:
        sanity_check_point_cloud_evidence(point_cloud, evidence.header)


def sanity_check_object_evidence(obj: ObjectWithEvidence) -> None:
    """Check the evidence of an object.

    The sensor of each evidence must be contained in the object state.

    Args:
        obj: ObjectWithEvidence instance to check

    Raises:
        ValueError: If any evidence is invalid
    """
    for evidence in obj.evidence:
        sanity_check_evidence(evidence)
        if evidence.sensor_id not in obj.object.state.sensor_id:
            raise ValueError('Evidence sensor is missing in object.state.sensor_id')


def add_evidence(obj: ObjectWithEvidence, evidence: Evidence) -> None:
    """Add evidence to an object and register its sensor in the object state.

    Args:
        obj: ObjectWithEvidence instance
        evidence: Evidence to add
    """
    sanity_check_evidence(evidence)
    if evidence.sensor_id not in obj.object.state.sensor_id:
        obj.object.state.sensor_id.append(evidence.sensor_id)
    obj.evidence.append(evidence)

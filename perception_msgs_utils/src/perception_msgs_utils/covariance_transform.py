# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

import copy
import tf2_geometry_msgs
from tf_transformations import quaternion_matrix
from .constants import CONTINUOUS_STATE_COVARIANCE_INVALID, CONTINUOUS_STATE_COVARIANCE_UNKNOWN


def transform_pose_with_covariance(input_pose, transform):
    """Transform a stamped pose and rotate covariance while preserving validity markers.

    Marker entries are replaced with zero for numerical rotation, then propagated
    to output entries that depend on them. An INVALID contribution makes an output
    diagonal INVALID; otherwise, any INVALID or UNKNOWN contribution makes the
    output entry UNKNOWN. INVALID is recognized only on the input diagonal, so
    negative cross covariance, including -1, is rotated numerically.
    Rotation coefficients with magnitude at most 1e-12 are ignored when propagating
    markers to avoid rounding residue at exact axis rotations.

    Args:
        input_pose (geometry_msgs.msg.PoseWithCovarianceStamped): Stamped pose and
            covariance in the source frame. The input is not modified.
        transform (geometry_msgs.msg.TransformStamped): Transform from the source
            frame to the target frame.

    Returns:
        geometry_msgs.msg.PoseWithCovarianceStamped: Transformed pose and covariance,
        with the transform's header.
    """
    covariance = input_pose.pose.covariance
    invalid = [i % 7 == 0 and value == CONTINUOUS_STATE_COVARIANCE_INVALID
               for i, value in enumerate(covariance)]
    unknown = [value == CONTINUOUS_STATE_COVARIANCE_UNKNOWN for value in covariance]
    numerical = copy.deepcopy(input_pose)
    numerical.pose.covariance = [0.0 if invalid[i] or unknown[i] else value
                                 for i, value in enumerate(covariance)]
    output = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(numerical, transform)
    if not any(invalid) and not any(unknown):
        return output

    q = transform.transform.rotation
    rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    for i in range(6):
        for j in range(6):
            invalid_contribution = False
            unknown_contribution = False
            for k in range(3):
                # Ignore rounding residue at exact axis rotations (e.g. pi/2).
                if abs(rotation[i % 3, k]) <= 1e-12:
                    continue
                for l in range(3):
                    if abs(rotation[j % 3, l]) <= 1e-12:
                        continue
                    index = 6 * (3 * (i // 3) + k) + 3 * (j // 3) + l
                    invalid_contribution |= invalid[index]
                    unknown_contribution |= unknown[index]
            if i == j and invalid_contribution:
                output.pose.covariance[6 * i + j] = CONTINUOUS_STATE_COVARIANCE_INVALID
            elif invalid_contribution or unknown_contribution:
                output.pose.covariance[6 * i + j] = CONTINUOUS_STATE_COVARIANCE_UNKNOWN
    return output

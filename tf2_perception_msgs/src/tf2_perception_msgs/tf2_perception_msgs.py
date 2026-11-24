# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

import math

from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion

from perception_msgs.msg import BOX2D, ObjectState, Object, ObjectList, EgoData, ObjectWithEvidence, ObjectListWithEvidence
from perception_msgs_utils.convenience_state_getters import get_pose, get_pose_with_covariance
from perception_msgs_utils.convenience_state_setters import set_pose_from_gm_pose, set_pose_with_covariance_from_gm_pose_with_covariance
from perception_msgs_utils.covariance_transform import transform_pose_with_covariance
from perception_msgs_utils.state_index import has_roll, has_pitch, has_yaw, index_roll, index_pitch, index_yaw
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs


def to_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_to_msg(ObjectState, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(Object, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(ObjectList, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(EgoData, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(ObjectWithEvidence, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(ObjectListWithEvidence, to_msg_msg)


def from_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_from_msg(ObjectState, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(Object, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(ObjectList, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(EgoData, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(ObjectWithEvidence, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(ObjectListWithEvidence, from_msg_msg)

def _replace_unmeasured_orientation(pose) -> bool:
    """Replace an unmeasured (NaN) orientation by identity, which tf2 can transform, and report it."""
    q = pose.orientation
    if all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
        return False
    pose.orientation = Quaternion()
    return True

def do_transform_state(
        state: ObjectState,
        transform: TransformStamped) -> ObjectState:
    """
    Apply a `Transform` or `TransformStamped` to `ObjectState`.

    The pose with covariance is transformed into a different frame,
    while the rest of the ObjectState is kept untouched.

    :param state: The object state that should be transformed
    :param transform: The transform which will be applied to the object state
    :returns: The transformed object state
    """
    if state.model_id == BOX2D.MODEL_ID:
        raise ValueError("BOX2D pixels cannot undergo a metric TF transform")
    state_out = state
    state_out.header.stamp = transform.header.stamp
    state_out.header.frame_id = transform.header.frame_id

    if not state.continuous_state_covariance:
        xyz_rpy = PoseStamped()
        xyz_rpy.pose = get_pose(state)
        unmeasured_orientation = _replace_unmeasured_orientation(xyz_rpy.pose)
        xyz_rpy_tf = tf2_geometry_msgs.do_transform_pose_stamped(xyz_rpy, transform)
        set_pose_from_gm_pose(state_out, xyz_rpy_tf.pose, False)
    else:
        xyz_rpy_cov = PoseWithCovarianceStamped()
        xyz_rpy_cov.pose = get_pose_with_covariance(state)
        unmeasured_orientation = _replace_unmeasured_orientation(xyz_rpy_cov.pose.pose)
        xyz_rpy_cov_tf = transform_pose_with_covariance(xyz_rpy_cov, transform)
        set_pose_with_covariance_from_gm_pose_with_covariance(state_out, xyz_rpy_cov_tf.pose)

    if unmeasured_orientation:
        for has, index in ((has_roll, index_roll), (has_pitch, index_pitch), (has_yaw, index_yaw)):
            if has(state_out.model_id):
                state_out.continuous_state[index(state_out.model_id)] = math.nan
    return state_out

tf2_ros.TransformRegistration().add(ObjectState, do_transform_state)

def do_transform_object(
        object: Object,
        transform: TransformStamped) -> Object:
    """
    Apply a `Transform` or `TransformStamped` to `Object`.

    The pose with covariance is transformed into a different frame,
    while the rest of the Object is kept untouched.

    :param object: The object that should be transformedy
    :param transform: The transform which will applied to the object
    :returns: The transformed object
    """
    object_out = object
    base_time = object.state.header.stamp
    object_out.state = do_transform_state(object.state, transform)

    # Currently commented out as we are not sure how the history will be filled/interpreted
    # for i in range(len(object.state_history)):
    #     object_out.state_history[i] = do_transform_state(object.state_history[i], transform)

    for i in range(len(object.state_predictions)):
        for j in range(len(object.state_predictions[i].states)):
            future_time = object.state_predictions[i].states[j].header.stamp
            state_in = object.state_predictions[i].states[j]
            state_in.header.stamp = base_time
            object_out.state_predictions[i].states[j] = do_transform_state(state_in, transform)
            # Restore the timestamp
            object_out.state_predictions[i].states[j].header.stamp = future_time

    return object_out

tf2_ros.TransformRegistration().add(Object, do_transform_object)

def do_transform_object_list(
        object_list: ObjectList,
        transform: TransformStamped) -> ObjectList:
    """
    Apply a `Transform` or `TransformStamped` to `ObjectList`.

    The pose with covariance is transformed into a different frame,
    while the rest of the ObjectList is kept untouched.

    :param object_list: The object list that should be transformed
    :param transform: The transform which will applied to the object list
    :returns: The transformed object list
    """
    object_list_out = object_list
    object_list_out.header.stamp = transform.header.stamp
    object_list_out.header.frame_id = transform.header.frame_id

    for i in range(len(object_list.objects)):
        object_list_out.objects[i] = do_transform_object(object_list.objects[i], transform)

    return object_list_out

tf2_ros.TransformRegistration().add(ObjectList, do_transform_object_list)

def do_transform_object_with_evidence(
        object_with_evidence: ObjectWithEvidence,
        transform: TransformStamped) -> ObjectWithEvidence:
    """
    Apply a `Transform` or `TransformStamped` to `ObjectWithEvidence`.

    The object is transformed into a different frame,
    while the evidence remains in its sensor frames.

    :param object_with_evidence: The object with evidence that should be transformed
    :param transform: The transform which will be applied to the object
    :returns: The transformed object with evidence
    """
    object_with_evidence_out = object_with_evidence
    object_with_evidence_out.object = do_transform_object(object_with_evidence.object, transform)

    return object_with_evidence_out

tf2_ros.TransformRegistration().add(ObjectWithEvidence, do_transform_object_with_evidence)

def do_transform_object_list_with_evidence(
        object_list: ObjectListWithEvidence,
        transform: TransformStamped) -> ObjectListWithEvidence:
    """
    Apply a `Transform` or `TransformStamped` to `ObjectListWithEvidence`.

    The objects are transformed into a different frame,
    while the evidence remains in its sensor frames.

    :param object_list: The object list that should be transformed
    :param transform: The transform which will be applied to the object list
    :returns: The transformed object list
    """
    object_list_out = object_list
    object_list_out.header.stamp = transform.header.stamp
    object_list_out.header.frame_id = transform.header.frame_id

    for i in range(len(object_list.objects)):
        object_list_out.objects[i] = do_transform_object_with_evidence(object_list.objects[i], transform)

    return object_list_out

tf2_ros.TransformRegistration().add(ObjectListWithEvidence, do_transform_object_list_with_evidence)

def do_transform_ego_data(
        ego_data: EgoData,
        transform: TransformStamped) -> EgoData:
    """
    Apply a `Transform` or `TransformStamped` to `EgoData`.

    The pose with covariance is transformed into a different frame,
    while the rest of the EgoData is kept untouched.

    :param ego_data: The ego data that should be transformed
    :param transform: The transform which will applied to the ego data
    :returns: The transformed ego data
    """
    ego_data_out = ego_data
    ego_data_out.header.stamp = transform.header.stamp
    ego_data_out.header.frame_id = transform.header.frame_id

    ego_data_out.state = do_transform_state(ego_data.state, transform)

    for i in range(len(ego_data.trajectory_planned)):
        ego_data_out.trajectory_planned[i] = do_transform_state(ego_data.trajectory_planned[i], transform)
    for i in range(len(ego_data.trajectory_past)):
        ego_data_out.trajectory_past[i] = do_transform_state(ego_data.trajectory_past[i], transform)
    for i in range(len(ego_data.route_planned)):
        ego_data_out.route_planned[i] = do_transform_state(ego_data.route_planned[i], transform)

    return ego_data_out

tf2_ros.TransformRegistration().add(EgoData, do_transform_ego_data)

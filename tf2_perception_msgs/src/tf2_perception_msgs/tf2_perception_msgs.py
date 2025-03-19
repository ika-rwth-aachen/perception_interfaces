from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped

from perception_msgs.msg import ObjectState, Object, ObjectList, EgoData
from perception_msgs_utils.convenience_state_getters import get_pose_with_covariance
from perception_msgs_utils.convenience_state_setters import set_pose_with_covariance_from_gm_pose_with_covariance
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs


def to_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_to_msg(ObjectState, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(Object, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(ObjectList, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(EgoData, to_msg_msg)


def from_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_from_msg(ObjectState, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(Object, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(ObjectList, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(EgoData, from_msg_msg)

def do_transform_state(
        state: ObjectState,
        transform: TransformStamped) -> ObjectState:
    """
    Apply a `Transform` or `TransformStamped` on a `ObjectState`.

    The pose with covariance is transformed into a different frame,
    while the rest of the ObjectState is kept untouched.

    :param state: The object state that should be transformedy
    :param transform: The transform which will applied to the object state
    :returns: The transformed object state
    """
    state_out = state
    state_out.header.stamp = transform.header.stamp
    state_out.header.frame_id = transform.header.frame_id

    xyz_rpy_cov = PoseWithCovarianceStamped()
    xyz_rpy_cov_tf = PoseWithCovarianceStamped()
    xyz_rpy_cov.pose = get_pose_with_covariance(state)
    xyz_rpy_cov_tf = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(xyz_rpy_cov, transform)
    set_pose_with_covariance_from_gm_pose_with_covariance(state_out, xyz_rpy_cov_tf.pose)
    return state_out

tf2_ros.TransformRegistration().add(ObjectState, do_transform_state)

def do_transform_object(
        object: Object,
        transform: TransformStamped) -> Object:
    """
    Apply a `Transform` or `TransformStamped` on a `Object`.

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
    Apply a `Transform` or `TransformStamped` on a `ObjectList`.

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

def do_transform_ego_data(
        ego_data: EgoData,
        transform: TransformStamped) -> EgoData:
    """
    Apply a `Transform` or `TransformStamped` on a `EgoData`.

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
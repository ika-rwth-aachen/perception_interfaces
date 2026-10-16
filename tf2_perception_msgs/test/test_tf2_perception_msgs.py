# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

import copy
import math
import pytest
from perception_msgs.msg import EgoData, Object, ObjectState, ObjectStatePrediction, ISCACTR, HEXAMOTION, EGO, EGORWS
from perception_msgs_utils.init import initialize_state
from perception_msgs_utils.utils import get_continuous_state_size
from perception_msgs_utils.convenience_state_setters import set_position_from_list, set_velocity_from_list, set_acceleration_from_list, set_continuous_state_covariance_diagonal
from perception_msgs_utils.constants import CONTINUOUS_STATE_COVARIANCE_INVALID, CONTINUOUS_STATE_COVARIANCE_UNKNOWN
from perception_msgs_utils.covariance_transform import transform_pose_with_covariance
from perception_msgs_utils.convenience_state_getters import get_pose_covariance, get_velocity_xyz_with_covariance, get_acceleration_xyz_with_covariance
from perception_msgs_utils.convenience_state_setters import set_continuous_state_covariance_at
from perception_msgs_utils.convenience_state_setters import set_velocity_xyz_yaw_with_covariance_from_gm_vector3, set_acceleration_xyz_yaw_with_covariance_from_gm_vector3
from perception_msgs_utils.state_index import index_x
from perception_msgs_utils.convenience_state_getters import get_continuous_state_covariance_diagonal, get_vel_x, get_vel_y, get_acc_x, get_acc_y
from perception_msgs_utils.state_setters import set_yaw, set_yaw_rate, set_width, set_length, set_height, set_roll, set_pitch, set_yaw, set_roll_rate, set_pitch_rate, set_yaw_rate, set_width, set_length, set_height, set_steering_angle_ack, set_steering_angle_rate_ack, set_steering_angle_front, set_steering_angle_rear
from perception_msgs_utils.state_getters import get_x, get_y, get_z, get_yaw, get_vel_lon, get_vel_lat, get_acc_lon, get_acc_lat, get_yaw_rate, get_roll, get_pitch, get_roll_rate, get_pitch_rate, get_yaw_rate, get_width, get_length, get_height, get_steering_angle_ack, get_steering_angle_rate_ack, get_steering_angle_front, get_steering_angle_rear
from tf2_perception_msgs import do_transform_ego_data, do_transform_object, do_transform_state
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Vector3
from tf_transformations import quaternion_from_euler

EPS = 1e-12

def test_doTransform_Object_ISCACTR():
    obj = Object()
    obj_tf = Object()
    initialize_state(obj, ISCACTR.MODEL_ID)
    n = get_continuous_state_size(obj.state.model_id)
    set_position_from_list(obj, [1.0, 2.0, 3.0])
    set_velocity_from_list(obj, [1.0, 2.0])
    set_acceleration_from_list(obj, [3.0, 4.0])
    set_yaw(obj, math.pi)
    set_yaw_rate(obj, -1.0)
    set_width(obj, -1.0)
    set_length(obj, -1.0)
    set_height(obj, -1.0)
    covariance_diagonal = [float(i) for i in range(n)]
    set_continuous_state_covariance_diagonal(obj, covariance_diagonal)

    tf = TransformStamped()
    tf.transform.translation.x = 10.0
    tf.transform.translation.y = 20.0
    tf.transform.translation.z = 30.0
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 1.0
    tf.transform.rotation.w = 0.0

    obj_tf = do_transform_object(obj, tf)

    # transformed state
    assert math.isclose(get_x(obj_tf), 9.0, abs_tol=EPS)
    assert math.isclose(get_y(obj_tf), 18.0, abs_tol=EPS)
    assert math.isclose(get_z(obj_tf), 33.0, abs_tol=EPS)
    assert math.isclose(get_yaw(obj_tf), 0.0, abs_tol=EPS)
    assert math.isclose(get_vel_x(obj_tf), 1.0, abs_tol=EPS)
    assert math.isclose(get_vel_y(obj_tf), 2.0, abs_tol=EPS)
    assert math.isclose(get_acc_x(obj_tf), 3.0, abs_tol=EPS)
    assert math.isclose(get_acc_y(obj_tf), 4.0, abs_tol=EPS)

    # transform-invariant state
    assert math.isclose(get_vel_lon(obj_tf), 1.0, abs_tol=EPS)
    assert math.isclose(get_vel_lat(obj_tf), 2.0, abs_tol=EPS)
    assert math.isclose(get_acc_lon(obj_tf), 3.0, abs_tol=EPS)
    assert math.isclose(get_acc_lat(obj_tf), 4.0, abs_tol=EPS)
    assert math.isclose(get_yaw_rate(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_width(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_length(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_height(obj_tf), -1.0, abs_tol=EPS)

    # transformed covariance
    covariance_diagonal_tf = get_continuous_state_covariance_diagonal(obj)
    for i in range(n):
        assert math.isclose(covariance_diagonal[i], covariance_diagonal_tf[i], abs_tol=EPS), f"i={i}"

def test_doTransform_Object_HEXAMOTION():
    obj = Object()
    obj_tf = Object()
    initialize_state(obj, HEXAMOTION.MODEL_ID)
    n = get_continuous_state_size(obj.state.model_id)
    set_position_from_list(obj, [1.0, 2.0, 3.0])
    set_velocity_from_list(obj, [1.0, 2.0])
    set_acceleration_from_list(obj, [3.0, 4.0])
    set_roll(obj, math.pi)
    set_pitch(obj, 0.0)
    set_yaw(obj, 0.0)
    set_roll_rate(obj, -1.0)
    set_pitch_rate(obj, -1.0)
    set_yaw_rate(obj, -1.0)
    set_width(obj, -1.0)
    set_length(obj, -1.0)
    set_height(obj, -1.0)
    covariance_diagonal = [float(i) for i in range(n)]
    set_continuous_state_covariance_diagonal(obj, covariance_diagonal)

    tf = TransformStamped()
    tf.transform.translation.x = 10.0
    tf.transform.translation.y = 20.0
    tf.transform.translation.z = 30.0
    tf.transform.rotation.x = 1.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 0.0

    obj_tf = do_transform_object(obj, tf)

    # transformed state
    assert math.isclose(get_x(obj_tf), 11.0, abs_tol=EPS)
    assert math.isclose(get_y(obj_tf), 18.0, abs_tol=EPS)
    assert math.isclose(get_z(obj_tf), 27.0, abs_tol=EPS)
    assert math.isclose(get_roll(obj_tf), 0.0, abs_tol=EPS)
    assert math.isclose(get_pitch(obj_tf), 0.0, abs_tol=EPS)
    assert math.isclose(get_yaw(obj_tf), 0.0, abs_tol=EPS)
    assert math.isclose(get_vel_x(obj_tf), 1.0, abs_tol=EPS)
    assert math.isclose(get_vel_y(obj_tf), 2.0, abs_tol=EPS)
    assert math.isclose(get_acc_x(obj_tf), 3.0, abs_tol=EPS)
    assert math.isclose(get_acc_y(obj_tf), 4.0, abs_tol=EPS)

    # transform-invariant state
    assert math.isclose(get_vel_lon(obj_tf), 1.0, abs_tol=EPS)
    assert math.isclose(get_vel_lat(obj_tf), 2.0, abs_tol=EPS)
    assert math.isclose(get_acc_lon(obj_tf), 3.0, abs_tol=EPS)
    assert math.isclose(get_acc_lat(obj_tf), 4.0, abs_tol=EPS)
    assert math.isclose(get_roll_rate(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_pitch_rate(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_yaw_rate(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_width(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_length(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_height(obj_tf), -1.0, abs_tol=EPS)

    # transformed covariance
    covariance_diagonal_tf = get_continuous_state_covariance_diagonal(obj)
    for i in range(n):
        assert math.isclose(covariance_diagonal[i], covariance_diagonal_tf[i], abs_tol=EPS), f"i={i}"

def test_doTransform_Object_EGO():
    obj = Object()
    obj_tf = Object()
    initialize_state(obj, EGO.MODEL_ID)
    n = get_continuous_state_size(obj.state.model_id)
    set_position_from_list(obj, [1.0, 2.0, 3.0])
    set_velocity_from_list(obj, [1.0, 2.0])
    set_acceleration_from_list(obj, [3.0, 4.0])
    set_yaw_rate(obj, -1.0)
    set_steering_angle_ack(obj, -1.0)
    set_steering_angle_rate_ack(obj, -1.0)
    covariance_diagonal = [float(i) for i in range(n)]
    set_continuous_state_covariance_diagonal(obj, covariance_diagonal)

    tf = TransformStamped()
    tf.transform.translation.x = 10.0
    tf.transform.translation.y = 20.0
    tf.transform.translation.z = 30.0
    tf.transform.rotation.x = 1.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 0.0

    obj_tf = do_transform_object(obj, tf)

    # transformed state
    assert math.isclose(get_x(obj_tf), 11.0, abs_tol=EPS)
    assert math.isclose(get_y(obj_tf), 18.0, abs_tol=EPS)
    assert math.isclose(get_z(obj_tf), 27.0, abs_tol=EPS)
    assert math.isclose(get_yaw(obj_tf), 0.0, abs_tol=EPS)
    assert math.isclose(get_vel_x(obj_tf), 1.0, abs_tol=EPS)
    assert math.isclose(get_vel_y(obj_tf), 2.0, abs_tol=EPS)
    assert math.isclose(get_acc_x(obj_tf), 3.0, abs_tol=EPS)
    assert math.isclose(get_acc_y(obj_tf), 4.0, abs_tol=EPS)

    # transform-invariant state
    assert math.isclose(get_vel_lon(obj_tf), 1.0, abs_tol=EPS)
    assert math.isclose(get_vel_lat(obj_tf), 2.0, abs_tol=EPS)
    assert math.isclose(get_acc_lon(obj_tf), 3.0, abs_tol=EPS)
    assert math.isclose(get_acc_lat(obj_tf), 4.0, abs_tol=EPS)
    assert math.isclose(get_yaw_rate(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_steering_angle_ack(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_steering_angle_rate_ack(obj_tf), -1.0, abs_tol=EPS)

    # transformed covariance
    covariance_diagonal_tf = get_continuous_state_covariance_diagonal(obj)
    for i in range(n):
        assert math.isclose(covariance_diagonal[i], covariance_diagonal_tf[i], abs_tol=EPS), f"i={i}"

def test_doTransform_Object_EGORWS():
    obj = Object()
    obj_tf = Object()
    initialize_state(obj, EGORWS.MODEL_ID)
    n = get_continuous_state_size(obj.state.model_id)
    set_position_from_list(obj, [1.0, 2.0, 3.0])
    set_roll(obj, math.pi / 4)
    set_pitch(obj, math.pi / 4)
    set_yaw(obj, math.pi / 2)
    set_velocity_from_list(obj, [1.0, 2.0])
    set_acceleration_from_list(obj, [3.0, 4.0])
    set_yaw_rate(obj, -1.0)
    set_steering_angle_front(obj, -1.0)
    set_steering_angle_rear(obj, -1.0)
    covariance_diagonal = [float(i) for i in range(n)]
    set_continuous_state_covariance_diagonal(obj, covariance_diagonal)

    tf = TransformStamped()
    tf.transform.translation.x = 10.0
    tf.transform.translation.y = 20.0
    tf.transform.translation.z = 30.0
    tf.transform.rotation.x = 1.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 0.0

    obj_tf = do_transform_object(obj, tf)

    # transformed state
    assert math.isclose(get_x(obj_tf), 11.0, abs_tol=EPS)
    assert math.isclose(get_y(obj_tf), 18.0, abs_tol=EPS)
    assert math.isclose(get_z(obj_tf), 27.0, abs_tol=EPS)
    assert math.isclose(get_roll(obj_tf), -3 * math.pi / 4, abs_tol=EPS)
    assert math.isclose(get_pitch(obj_tf), -math.pi / 4, abs_tol=EPS)
    assert math.isclose(get_yaw(obj_tf), -math.pi / 2, abs_tol=EPS)

    # transform-invariant state
    assert math.isclose(get_vel_lon(obj_tf), 1.0, abs_tol=EPS)
    assert math.isclose(get_vel_lat(obj_tf), 2.0, abs_tol=EPS)
    assert math.isclose(get_acc_lon(obj_tf), 3.0, abs_tol=EPS)
    assert math.isclose(get_acc_lat(obj_tf), 4.0, abs_tol=EPS)
    assert math.isclose(get_yaw_rate(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_steering_angle_front(obj_tf), -1.0, abs_tol=EPS)
    assert math.isclose(get_steering_angle_rear(obj_tf), -1.0, abs_tol=EPS)

    # transformed covariance
    covariance_diagonal_tf = get_continuous_state_covariance_diagonal(obj)
    for i in range(n):
        assert math.isclose(covariance_diagonal[i], covariance_diagonal_tf[i], abs_tol=EPS), f"i={i}"



def test_transform_prediction_without_covariance():
    obj = Object()
    initialize_state(obj, ISCACTR.MODEL_ID)
    set_continuous_state_covariance_diagonal(obj, [0.1] * get_continuous_state_size(obj))
    set_position_from_list(obj, [1.0, 2.0, 3.0])
    prediction = ObjectStatePrediction()
    prediction.states.append(copy.deepcopy(obj.state))
    prediction.states[0].continuous_state_covariance = []
    obj.state_predictions.append(prediction)

    transform = TransformStamped()
    transform.transform.translation.x = 10.0
    transform.transform.rotation.w = 1.0
    transformed = do_transform_object(obj, transform)

    assert math.isclose(get_x(transformed.state), 11.0, abs_tol=EPS)
    assert transformed.state.continuous_state_covariance
    assert math.isclose(get_x(transformed.state_predictions[0].states[0]), 11.0, abs_tol=EPS)
    assert len(transformed.state_predictions[0].states[0].continuous_state_covariance) == 0


@pytest.mark.parametrize("model", [EGO.MODEL_ID, EGORWS.MODEL_ID])
def test_transform_ego_trajectories_with_optional_covariance(model):
    ego = EgoData()
    initialize_state(ego, model)
    set_position_from_list(ego, [1.0, 2.0, 3.0])
    set_velocity_from_list(ego, [4.0, 0.0])
    set_continuous_state_covariance_diagonal(ego, [0.1] * get_continuous_state_size(ego.state))
    ego.trajectory_planned = [copy.deepcopy(ego.state), copy.deepcopy(ego.state)]
    ego.trajectory_planned[0].continuous_state_covariance = []
    ego.trajectory_past = copy.deepcopy(ego.trajectory_planned)

    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.transform.translation.x = 10.0
    transform.transform.translation.y = 20.0
    transform.transform.rotation.z = 1.0
    transform.transform.rotation.w = 0.0
    transformed = do_transform_ego_data(copy.deepcopy(ego), transform)

    assert transformed.header.frame_id == "map"
    assert get_x(transformed.state) == pytest.approx(9.0)
    assert get_y(transformed.state) == pytest.approx(18.0)
    assert get_continuous_state_covariance_diagonal(transformed.state)[0] == pytest.approx(0.1)
    for trajectory in [transformed.trajectory_planned, transformed.trajectory_past]:
        assert len(trajectory) == 2
        assert len(trajectory[0].continuous_state_covariance) == 0
        assert len(trajectory[1].continuous_state_covariance) == len(ego.state.continuous_state_covariance)
        for state in trajectory:
            assert state.header.frame_id == "map"
            assert get_x(state) == pytest.approx(9.0)
            assert get_y(state) == pytest.approx(18.0)
            assert abs(get_yaw(state)) == pytest.approx(math.pi)
            assert get_vel_lon(state) == pytest.approx(4.0)
        assert get_continuous_state_covariance_diagonal(trajectory[0])[0] == CONTINUOUS_STATE_COVARIANCE_INVALID
        assert get_continuous_state_covariance_diagonal(trajectory[1])[0] == pytest.approx(0.1)
    assert get_x(ego.state) == pytest.approx(1.0)
    assert ego.state.continuous_state_covariance


@pytest.mark.parametrize("model", [EGO.MODEL_ID, EGORWS.MODEL_ID, ISCACTR.MODEL_ID, HEXAMOTION.MODEL_ID])
@pytest.mark.parametrize("marker", [CONTINUOUS_STATE_COVARIANCE_INVALID, CONTINUOUS_STATE_COVARIANCE_UNKNOWN])
@pytest.mark.parametrize("angle", [0.0, math.pi / 2, math.pi / 4])
def test_rotation_preserves_covariance_markers(model, marker, angle):
    state = ObjectState()
    initialize_state(state, model)
    set_position_from_list(state, [1.0, 2.0, 3.0])
    set_continuous_state_covariance_diagonal(state, [4.0] * get_continuous_state_size(state))
    set_continuous_state_covariance_at(state, index_x(model), index_x(model), marker)
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.transform.translation.x = 10.0
    transform.transform.rotation.z = math.sin(angle / 2)
    transform.transform.rotation.w = math.cos(angle / 2)
    transformed = do_transform_state(state, transform)
    covariance = get_pose_covariance(transformed)
    assert covariance[0] == (4.0 if angle == math.pi / 2 else marker)
    assert covariance[7] == (4.0 if angle == 0.0 else marker)
    assert covariance[14] == 4.0
    if angle == math.pi / 4:
        assert covariance[1] == CONTINUOUS_STATE_COVARIANCE_UNKNOWN
    else:
        assert covariance[1] == pytest.approx(0.0, abs=EPS)
    assert covariance[6] == covariance[1]
    assert all(math.isfinite(value) for value in transformed.continuous_state_covariance)
    assert get_x(transformed) == pytest.approx(10.0 + math.cos(angle) - 2.0 * math.sin(angle))
    assert get_y(transformed) == pytest.approx(math.sin(angle) + 2.0 * math.cos(angle))
    assert transformed.header.frame_id == "map"


def test_negative_cross_covariance_rotates_numerically():
    state = ObjectState()
    initialize_state(state, EGO.MODEL_ID)
    set_continuous_state_covariance_diagonal(state, [4.0] * get_continuous_state_size(state))
    set_continuous_state_covariance_at(state, EGO.X, EGO.Y, -1.0)
    set_continuous_state_covariance_at(state, EGO.Y, EGO.X, -1.0)
    transform = TransformStamped()
    transform.transform.rotation.z = math.sin(math.pi / 8)
    transform.transform.rotation.w = math.cos(math.pi / 8)
    covariance = get_pose_covariance(do_transform_state(state, transform))
    assert covariance[0] == pytest.approx(5.0)
    assert covariance[7] == pytest.approx(3.0)
    assert covariance[1] == pytest.approx(0.0, abs=EPS)


def test_unknown_cross_terms_and_invalid_diagonals_follow_rotation():
    pose = PoseWithCovarianceStamped()
    pose.pose.pose.orientation.w = 1.0
    pose.pose.covariance[0] = CONTINUOUS_STATE_COVARIANCE_INVALID
    pose.pose.covariance[7] = CONTINUOUS_STATE_COVARIANCE_UNKNOWN
    pose.pose.covariance[2] = pose.pose.covariance[12] = CONTINUOUS_STATE_COVARIANCE_UNKNOWN
    transform = TransformStamped()
    transform.transform.rotation.z = math.sin(math.pi / 8)
    transform.transform.rotation.w = math.cos(math.pi / 8)
    covariance = transform_pose_with_covariance(pose, transform).pose.covariance
    assert covariance[0] == CONTINUOUS_STATE_COVARIANCE_INVALID
    assert covariance[7] == CONTINUOUS_STATE_COVARIANCE_INVALID
    assert all(covariance[i] == CONTINUOUS_STATE_COVARIANCE_UNKNOWN for i in [1, 6, 2, 12, 8, 13])
    assert covariance[14] == 0.0


@pytest.mark.parametrize("marker", [CONTINUOUS_STATE_COVARIANCE_INVALID, CONTINUOUS_STATE_COVARIANCE_UNKNOWN])
def test_velocity_and_acceleration_covariance_getters_preserve_markers(marker):
    state = ObjectState()
    initialize_state(state, EGO.MODEL_ID)
    set_yaw(state, math.pi / 4)
    set_continuous_state_covariance_diagonal(state, [4.0] * get_continuous_state_size(state))
    set_continuous_state_covariance_at(state, EGO.VEL_LON, EGO.VEL_LON, marker)
    set_continuous_state_covariance_at(state, EGO.ACC_LON, EGO.ACC_LON, marker)
    for pose in [get_velocity_xyz_with_covariance(state), get_acceleration_xyz_with_covariance(state)]:
        assert pose.covariance[0] == marker
        assert pose.covariance[7] == marker
        assert pose.covariance[1] == CONTINUOUS_STATE_COVARIANCE_UNKNOWN
        assert pose.covariance[14] == 0.0


@pytest.mark.parametrize("marker", [CONTINUOUS_STATE_COVARIANCE_INVALID, CONTINUOUS_STATE_COVARIANCE_UNKNOWN])
def test_velocity_and_acceleration_covariance_setters_preserve_markers(marker):
    state = ObjectState()
    initialize_state(state, EGO.MODEL_ID)
    vector = Vector3(x=1.0)
    set_velocity_xyz_yaw_with_covariance_from_gm_vector3(state, vector, math.pi / 4, marker, 4.0, 0.0)
    set_acceleration_xyz_yaw_with_covariance_from_gm_vector3(state, vector, math.pi / 4, marker, 4.0, 0.0)
    n = get_continuous_state_size(state)
    for i, j in [(EGO.VEL_LON, EGO.VEL_LAT), (EGO.ACC_LON, EGO.ACC_LAT)]:
        assert state.continuous_state_covariance[n * i + i] == marker
        assert state.continuous_state_covariance[n * j + j] == marker
        assert state.continuous_state_covariance[n * i + j] == CONTINUOUS_STATE_COVARIANCE_UNKNOWN


def test_nested_states_preserve_markers_under_three_axis_rotation():
    obj = Object()
    initialize_state(obj, EGO.MODEL_ID)
    set_continuous_state_covariance_diagonal(obj, [CONTINUOUS_STATE_COVARIANCE_UNKNOWN] * get_continuous_state_size(obj))
    prediction = ObjectStatePrediction(states=[copy.deepcopy(obj.state)])
    obj.state_predictions = [prediction]
    ego = EgoData(state=copy.deepcopy(obj.state), trajectory_planned=[copy.deepcopy(obj.state)],
                  trajectory_past=[copy.deepcopy(obj.state)])
    transform = TransformStamped()
    q = transform.transform.rotation
    q.x, q.y, q.z, q.w = quaternion_from_euler(0.3, 0.4, 0.5)
    transformed = do_transform_object(obj, transform)
    transformed_ego = do_transform_ego_data(ego, transform)
    states = [transformed.state, transformed.state_predictions[0].states[0], transformed_ego.state,
              transformed_ego.trajectory_planned[0], transformed_ego.trajectory_past[0]]
    for state in states:
        covariance = get_pose_covariance(state)
        assert all(covariance[7 * i] == CONTINUOUS_STATE_COVARIANCE_UNKNOWN for i in range(6))
        assert all(math.isfinite(value) for value in covariance)


if __name__ == "__main__":
    pytest.main()

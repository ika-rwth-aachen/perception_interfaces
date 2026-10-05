# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

import copy
import math
import pytest
from perception_msgs.msg import EgoData, Object, ObjectStatePrediction, ISCACTR, HEXAMOTION, EGO, EGORWS
from perception_msgs_utils.init import initialize_state
from perception_msgs_utils.utils import get_continuous_state_size
from perception_msgs_utils.convenience_state_setters import set_position_from_list, set_velocity_from_list, set_acceleration_from_list, set_continuous_state_covariance_diagonal
from perception_msgs_utils.constants import CONTINUOUS_STATE_COVARIANCE_INVALID
from perception_msgs_utils.convenience_state_getters import get_continuous_state_covariance_diagonal, get_vel_x, get_vel_y, get_acc_x, get_acc_y
from perception_msgs_utils.state_setters import set_yaw, set_yaw_rate, set_width, set_length, set_height, set_roll, set_pitch, set_yaw, set_roll_rate, set_pitch_rate, set_yaw_rate, set_width, set_length, set_height, set_steering_angle_ack, set_steering_angle_rate_ack, set_steering_angle_front, set_steering_angle_rear
from perception_msgs_utils.state_getters import get_x, get_y, get_z, get_yaw, get_vel_lon, get_vel_lat, get_acc_lon, get_acc_lat, get_yaw_rate, get_roll, get_pitch, get_roll_rate, get_pitch_rate, get_yaw_rate, get_width, get_length, get_height, get_steering_angle_ack, get_steering_angle_rate_ack, get_steering_angle_front, get_steering_angle_rear
from tf2_perception_msgs import do_transform_ego_data, do_transform_object
from geometry_msgs.msg import TransformStamped

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


if __name__ == "__main__":
    pytest.main()

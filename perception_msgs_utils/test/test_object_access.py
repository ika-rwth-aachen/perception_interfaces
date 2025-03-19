import pytest
import random
import numpy as np

from perception_msgs.msg import EGO, EGORWS, Object, ISCACTR, HEXAMOTION, TRAFFICLIGHT, ObjectClassification
from geometry_msgs.msg import PoseWithCovariance, Vector3
from perception_msgs_utils.init import initialize_state
from perception_msgs_utils.utils import get_continuous_state_size, get_discrete_state_size, get_continuous_state_covariance_size
from perception_msgs_utils.convenience_state_getters import get_continuous_state, get_discrete_state, get_continuous_state_covariance, get_class_with_highest_probability, get_pose_with_covariance, get_velocity, get_acceleration, get_roll_in_deg, get_pitch_in_deg, get_yaw_in_deg, get_velocity_xyz, get_velocity_magnitude, get_acceleration_magnitude, get_velocity_xyz_with_covariance, get_acceleration_xyz_with_covariance, get_acceleration_xyz
from perception_msgs_utils.convenience_state_setters import set_continuous_state, set_discrete_state, set_continuous_state_covariance, set_continuous_state_covariance_at, set_pose_with_covariance_from_gm_pose_with_covariance, set_velocity_from_gm_vector3, set_acceleration_from_gm_vector3, set_roll_in_deg, set_pitch_in_deg, set_yaw_in_deg, set_velocity_xyz_yaw_from_gm_vector3, set_velocity_xyz_yaw_with_covariance_from_gm_vector3, set_acceleration_xyz_yaw_from_gm_vector3, set_acceleration_xyz_yaw_with_covariance_from_gm_vector3
from perception_msgs_utils.state_setters import set_x, set_y, set_z, set_vel_lon, set_vel_lat, set_acc_lon, set_acc_lat, set_roll, set_pitch, set_yaw, set_yaw_rate, set_steering_angle_ack, set_steering_angle_rate_ack, set_standstill, set_steering_angle_front, set_steering_angle_rear, set_width, set_length, set_height, set_roll_rate, set_pitch_rate, set_state, set_type
from perception_msgs_utils.state_getters import get_x, get_y, get_z, get_vel_lon, get_vel_lat, get_acc_lon, get_acc_lat, get_roll, get_pitch, get_yaw, get_yaw_rate, get_steering_angle_ack, get_steering_angle_rate_ack, get_standstill, get_steering_angle_front, get_steering_angle_rear, get_width, get_length, get_height, get_roll_rate, get_pitch_rate, get_state, get_type
from perception_msgs_utils.state_index import index_vel_lon, index_vel_lat, index_acc_lon, index_acc_lat

def random_value():
    return random.uniform(-1, 1)

def test_init():
    obj = Object()
    initialize_state(obj, EGO.MODEL_ID)

    assert get_continuous_state_size(obj) == get_continuous_state_size(EGO.MODEL_ID)
    assert get_discrete_state_size(obj) == get_discrete_state_size(EGO.MODEL_ID)
    assert get_continuous_state_covariance_size(obj) == get_continuous_state_covariance_size(EGO.MODEL_ID)

    n = get_continuous_state_size(obj)
    m = get_discrete_state_size(obj)

    continuous_state = get_continuous_state(obj)
    for i in range(n):
        assert continuous_state[i] == pytest.approx(0.0), f"i={i}"

    discrete_state = get_discrete_state(obj)
    for i in range(m):
        assert discrete_state[i] == 0, f"i={i}"

    continuous_state_covariance = get_continuous_state_covariance(obj)
    for i in range(n):
        for j in range(n):
            if i == j:
                assert continuous_state_covariance[n * i + j] == pytest.approx(-1.0), f"i={i}, j={j}"
            else:
                assert continuous_state_covariance[n * i + j] == pytest.approx(0.0), f"i={i}, j={j}"

def test_set_get_EGO():
    obj = Object()
    initialize_state(obj, EGO.MODEL_ID)

    val = random_value()
    set_x(obj, val)
    assert get_x(obj) == pytest.approx(val)

    val = random_value()
    set_y(obj, val)
    assert get_y(obj) == pytest.approx(val)

    val = random_value()
    set_z(obj, val)
    assert get_z(obj) == pytest.approx(val)

    val = random_value()
    set_vel_lon(obj, val)
    assert get_vel_lon(obj) == pytest.approx(val)

    val = random_value()
    set_vel_lat(obj, val)
    assert get_vel_lat(obj) == pytest.approx(val)

    val = random_value()
    set_acc_lon(obj, val)
    assert get_acc_lon(obj) == pytest.approx(val)

    val = random_value()
    set_acc_lat(obj, val)
    assert get_acc_lat(obj) == pytest.approx(val)

    val = random_value()
    set_roll(obj, val)
    assert get_roll(obj) == pytest.approx(val)

    val = random_value()
    set_pitch(obj, val)
    assert get_pitch(obj) == pytest.approx(val)

    val = random_value()
    set_yaw(obj, val)
    assert get_yaw(obj) == pytest.approx(val)

    val = random_value()
    set_yaw_rate(obj, val)
    assert get_yaw_rate(obj) == pytest.approx(val)

    val = random_value()
    set_steering_angle_ack(obj, val)
    assert get_steering_angle_ack(obj) == pytest.approx(val)

    val = random_value()
    set_steering_angle_rate_ack(obj, val)
    assert get_steering_angle_rate_ack(obj) == pytest.approx(val)

    set_standstill(obj, True)
    assert get_standstill(obj) == True

    continuous_state_covariance = get_continuous_state_covariance(obj)
    n = get_continuous_state_size(obj)
    for i in range(n):
        for j in range(n):
            if i == j:
                assert continuous_state_covariance[n * i + j] == pytest.approx(np.finfo(float).max), f"i={i}, j={j}"
            else:
                assert continuous_state_covariance[n * i + j] == pytest.approx(0.0), f"i={i}, j={j}"

def test_set_get_EGORWS():
    obj = Object()
    initialize_state(obj, EGORWS.MODEL_ID)

    val = random_value()
    set_x(obj, val)
    assert get_x(obj) == pytest.approx(val)

    val = random_value()
    set_y(obj, val)
    assert get_y(obj) == pytest.approx(val)

    val = random_value()
    set_z(obj, val)
    assert get_z(obj) == pytest.approx(val)

    val = random_value()
    set_vel_lon(obj, val)
    assert get_vel_lon(obj) == pytest.approx(val)

    val = random_value()
    set_vel_lat(obj, val)
    assert get_vel_lat(obj) == pytest.approx(val)

    val = random_value()
    set_acc_lon(obj, val)
    assert get_acc_lon(obj) == pytest.approx(val)

    val = random_value()
    set_acc_lat(obj, val)
    assert get_acc_lat(obj) == pytest.approx(val)

    val = random_value()
    set_roll(obj, val)
    assert get_roll(obj) == pytest.approx(val)

    val = random_value()
    set_pitch(obj, val)
    assert get_pitch(obj) == pytest.approx(val)

    val = random_value()
    set_yaw(obj, val)
    assert get_yaw(obj) == pytest.approx(val)

    val = random_value()
    set_yaw_rate(obj, val)
    assert get_yaw_rate(obj) == pytest.approx(val)

    val = random_value()
    set_steering_angle_front(obj, val)
    assert get_steering_angle_front(obj) == pytest.approx(val)

    val = random_value()
    set_steering_angle_rear(obj, val)
    assert get_steering_angle_rear(obj) == pytest.approx(val)

    set_standstill(obj, True)
    assert get_standstill(obj) == True

    continuous_state_covariance = get_continuous_state_covariance(obj)
    n = get_continuous_state_size(obj)
    for i in range(n):
        for j in range(n):
            if i == j:
                assert continuous_state_covariance[n * i + j] == pytest.approx(np.finfo(float).max), f"i={i}, j={j}"
            else:
                assert continuous_state_covariance[n * i + j] == pytest.approx(0.0), f"i={i}, j={j}"

def test_set_get_ISCACTR():
    obj = Object()
    initialize_state(obj, ISCACTR.MODEL_ID)

    val = random_value()
    set_x(obj, val)
    assert get_x(obj) == pytest.approx(val)

    val = random_value()
    set_y(obj, val)
    assert get_y(obj) == pytest.approx(val)

    val = random_value()
    set_z(obj, val)
    assert get_z(obj) == pytest.approx(val)

    val = random_value()
    set_vel_lon(obj, val)
    assert get_vel_lon(obj) == pytest.approx(val)

    val = random_value()
    set_vel_lat(obj, val)
    assert get_vel_lat(obj) == pytest.approx(val)

    val = random_value()
    set_acc_lon(obj, val)
    assert get_acc_lon(obj) == pytest.approx(val)

    val = random_value()
    set_acc_lat(obj, val)
    assert get_acc_lat(obj) == pytest.approx(val)

    val = random_value()
    set_yaw(obj, val)
    assert get_yaw(obj) == pytest.approx(val)

    val = random_value()
    set_yaw_rate(obj, val)
    assert get_yaw_rate(obj) == pytest.approx(val)

    val = random_value()
    set_width(obj, val)
    assert get_width(obj) == pytest.approx(val)

    val = random_value()
    set_length(obj, val)
    assert get_length(obj) == pytest.approx(val)

    val = random_value()
    set_height(obj, val)
    assert get_height(obj) == pytest.approx(val)

def test_set_get_HEXAMOTION():
    obj = Object()
    initialize_state(obj, HEXAMOTION.MODEL_ID)

    val = random_value()
    set_x(obj, val)
    assert get_x(obj) == pytest.approx(val)

    val = random_value()
    set_y(obj, val)
    assert get_y(obj) == pytest.approx(val)

    val = random_value()
    set_z(obj, val)
    assert get_z(obj) == pytest.approx(val)

    val = random_value()
    set_vel_lon(obj, val)
    assert get_vel_lon(obj) == pytest.approx(val)

    val = random_value()
    set_vel_lat(obj, val)
    assert get_vel_lat(obj) == pytest.approx(val)

    val = random_value()
    set_acc_lon(obj, val)
    assert get_acc_lon(obj) == pytest.approx(val)

    val = random_value()
    set_acc_lat(obj, val)
    assert get_acc_lat(obj) == pytest.approx(val)

    val = random_value()
    set_roll(obj, val)
    assert get_roll(obj) == pytest.approx(val)

    val = random_value()
    set_roll_rate(obj, val)
    assert get_roll_rate(obj) == pytest.approx(val)

    val = random_value()
    set_pitch(obj, val)
    assert get_pitch(obj) == pytest.approx(val)

    val = random_value()
    set_pitch_rate(obj, val)
    assert get_pitch_rate(obj) == pytest.approx(val)

    val = random_value()
    set_yaw(obj, val)
    assert get_yaw(obj) == pytest.approx(val)

    val = random_value()
    set_yaw_rate(obj, val)
    assert get_yaw_rate(obj) == pytest.approx(val)

    val = random_value()
    set_length(obj, val)
    assert get_length(obj) == pytest.approx(val)

    val = random_value()
    set_width(obj, val)
    assert get_width(obj) == pytest.approx(val)

    val = random_value()
    set_height(obj, val)
    assert get_height(obj) == pytest.approx(val)

def test_set_get_TRAFFICLIGHT():
    obj = Object()
    initialize_state(obj, TRAFFICLIGHT.MODEL_ID)

    val = random_value()
    set_x(obj, val)
    assert get_x(obj) == pytest.approx(val)

    val = random_value()
    set_y(obj, val)
    assert get_y(obj) == pytest.approx(val)

    val = random_value()
    set_z(obj, val)
    assert get_z(obj) == pytest.approx(val)

    set_state(obj, TRAFFICLIGHT.STATE_RED)
    assert get_state(obj) == pytest.approx(TRAFFICLIGHT.STATE_RED)

    set_type(obj, TRAFFICLIGHT.TYPE_STRAIGHT)
    assert get_type(obj) == pytest.approx(TRAFFICLIGHT.TYPE_STRAIGHT)

def test_get_class_with_highest_probability():
    obj = Object()
    dummy_class = ObjectClassification()
    max_class = ObjectClassification()

    dummy_class.type = ObjectClassification.MOTORBIKE
    dummy_class.probability = 0.2
    obj.state.classifications.append(dummy_class)
    dummy_class.type = ObjectClassification.VAN
    dummy_class.probability = 0.3
    obj.state.classifications.append(dummy_class)
    dummy_class.type = ObjectClassification.CAR
    dummy_class.probability = 0.4
    max_class = dummy_class
    obj.state.classifications.append(dummy_class)
    dummy_class.type = ObjectClassification.PEDESTRIAN
    dummy_class.probability = 0.1
    obj.state.classifications.append(dummy_class)

    output_class = get_class_with_highest_probability(obj)

    assert output_class.probability == pytest.approx(max_class.probability)
    assert output_class.type == max_class.type

def test_convenience_set_get():
    obj = Object()

    # set/getContinuousState
    initialize_state(obj, EGO.MODEL_ID)
    state1 = [float(i) for i in range(get_continuous_state_size(EGO.MODEL_ID))]
    set_continuous_state(obj, state1)
    state2 = get_continuous_state(obj)
    for i in range(get_continuous_state_size(EGO.MODEL_ID)):
        assert state1[i] == pytest.approx(state2[i]), f"i={i}"

    # set/getDiscreteState
    initialize_state(obj, EGO.MODEL_ID)
    discrete_state1 = [i for i in range(get_discrete_state_size(EGO.MODEL_ID))]
    set_discrete_state(obj, discrete_state1)
    discrete_state2 = get_discrete_state(obj)
    for i in range(get_discrete_state_size(EGO.MODEL_ID)):
        assert discrete_state1[i] == discrete_state2[i], f"i={i}"

    # set/getCovariance
    initialize_state(obj, EGO.MODEL_ID)
    state_cov1 = [float(i) for i in range(get_continuous_state_covariance_size(EGO.MODEL_ID))]
    set_continuous_state_covariance(obj, state_cov1)
    state_cov2 = get_continuous_state_covariance(obj)
    for i in range(get_continuous_state_covariance_size(EGO.MODEL_ID)):
        assert state_cov1[i] == pytest.approx(state_cov2[i]), f"i={i}"

    # set/getPoseWithCovariance
    #   set/getPose
    #     set/getPosition
    #     set/getOrientation
    #   set/getPoseCovariance
    #     set/getContinuousStateCovarianceAt
    initialize_state(obj, EGO.MODEL_ID)
    p1 = PoseWithCovariance()
    p1.pose.position.x = 1.0
    p1.pose.position.y = 1.0
    p1.pose.position.z = 1.0
    p1.pose.orientation.x = 0.5
    p1.pose.orientation.y = 0.5
    p1.pose.orientation.z = 0.5
    p1.pose.orientation.w = 0.5
    for i in range(6):
        for j in range(6):
            p1.covariance[6 * i + j] = 6 * i + j
    set_pose_with_covariance_from_gm_pose_with_covariance(obj, p1)
    p2 = get_pose_with_covariance(obj)
    assert p1.pose.position.x == pytest.approx(p2.pose.position.x)
    assert p1.pose.position.y == pytest.approx(p2.pose.position.y)
    assert p1.pose.position.z == pytest.approx(p2.pose.position.z)
    assert p1.pose.orientation.x == pytest.approx(p2.pose.orientation.x)
    assert p1.pose.orientation.y == pytest.approx(p2.pose.orientation.y)
    assert p1.pose.orientation.z == pytest.approx(p2.pose.orientation.z)
    assert p1.pose.orientation.w == pytest.approx(p2.pose.orientation.w)
    for i in range(6):
        for j in range(6):
            assert p1.covariance[6 * i + j] == pytest.approx(p2.covariance[6 * i + j]), f"i={i}, j={j}"

    # set/getVelocity
    initialize_state(obj, EGO.MODEL_ID)
    vel1 = Vector3()
    vel1.x = 3.0
    vel1.y = 4.0
    vel1.z = 5.0
    set_velocity_from_gm_vector3(obj, vel1)
    vel2 = get_velocity(obj)
    assert vel1.x == pytest.approx(vel2.x)
    assert vel1.y == pytest.approx(vel2.y)
    assert vel2.z == pytest.approx(0.0)
    assert get_velocity_magnitude(obj) == pytest.approx(np.sqrt(vel1.x**2 + vel1.y**2))

    # set/getAcceleration
    initialize_state(obj, EGO.MODEL_ID)
    acc1 = Vector3()
    acc1.x = 3.0
    acc1.y = 4.0
    acc1.z = 5.0
    set_acceleration_from_gm_vector3(obj, acc1)
    acc2 = get_acceleration(obj)
    assert acc1.x == pytest.approx(acc2.x)
    assert acc1.y == pytest.approx(acc2.y)
    assert acc2.z == pytest.approx(0.0)
    assert get_acceleration_magnitude(obj) == pytest.approx(np.sqrt(acc1.x**2 + acc1.y**2))

    # set/getRoll/Pitch/YawInDeg
    initialize_state(obj, EGO.MODEL_ID)
    val = random_value()
    set_roll_in_deg(obj, val)
    assert get_roll_in_deg(obj) == pytest.approx(val)
    val = random_value()
    set_pitch_in_deg(obj, val)
    assert get_pitch_in_deg(obj) == pytest.approx(val)
    val = random_value()
    set_yaw_in_deg(obj, val)
    assert get_yaw_in_deg(obj) == pytest.approx(val)

    # set/getVelocityXYZ
    initialize_state(obj, EGO.MODEL_ID)
    vel1_lon_lat = Vector3()
    vel1_lon_lat.x = 3.0
    vel1_lon_lat.y = 4.0
    vel1_lon_lat.z = 5.0
    set_velocity_from_gm_vector3(obj, vel1_lon_lat)
    set_yaw(obj, np.pi / 2)
    vel2_xyz = get_velocity_xyz(obj)
    assert vel2_xyz.x == pytest.approx(-vel1_lon_lat.y)
    assert vel2_xyz.y == pytest.approx(vel1_lon_lat.x)
    assert vel2_xyz.z == pytest.approx(0.0)
    set_velocity_xyz_yaw_from_gm_vector3(obj, vel2_xyz, np.pi / 2)
    vel3_xyz = get_velocity_xyz(obj)
    assert vel3_xyz.x == pytest.approx(vel2_xyz.x)
    assert vel3_xyz.y == pytest.approx(vel2_xyz.y)
    assert vel3_xyz.z == pytest.approx(vel2_xyz.z)

    # set/getVeocityXYZWithCovariance
    initialize_state(obj, ISCACTR.MODEL_ID)
    vel1_lon_lat = Vector3()
    vel1_lon_lat.x = 3.0
    vel1_lon_lat.y = 4.0
    vel1_lon_lat.z = 5.0
    set_velocity_from_gm_vector3(obj, vel1_lon_lat)
    var_lon = abs(random_value()) + 0.5
    var_lat = abs(random_value()) + 0.1
    cov_lat_lon = random_value() * max(var_lon, var_lat)
    set_continuous_state_covariance_at(obj, index_vel_lon(obj.state.model_id), index_vel_lon(obj.state.model_id), var_lon)
    set_continuous_state_covariance_at(obj, index_vel_lon(obj.state.model_id), index_vel_lat(obj.state.model_id), cov_lat_lon)
    set_continuous_state_covariance_at(obj, index_vel_lat(obj.state.model_id), index_vel_lon(obj.state.model_id), cov_lat_lon)
    set_continuous_state_covariance_at(obj, index_vel_lat(obj.state.model_id), index_vel_lat(obj.state.model_id), var_lat)
    set_yaw(obj, np.pi / 2)
    vel2_xyz_with_cov = get_velocity_xyz_with_covariance(obj)
    assert vel2_xyz_with_cov.pose.position.y == pytest.approx(vel1_lon_lat.x)
    assert vel2_xyz_with_cov.pose.position.x == pytest.approx(-vel1_lon_lat.y)
    assert vel2_xyz_with_cov.pose.position.z == pytest.approx(0.0)
    assert vel2_xyz_with_cov.covariance[0] == pytest.approx(var_lat)
    assert vel2_xyz_with_cov.covariance[1] == pytest.approx(-cov_lat_lon)
    assert vel2_xyz_with_cov.covariance[6] == pytest.approx(-cov_lat_lon)
    assert vel2_xyz_with_cov.covariance[7] == pytest.approx(var_lon)
    assert vel2_xyz_with_cov.covariance[2] == pytest.approx(0.0)
    assert vel2_xyz_with_cov.covariance[8] == pytest.approx(0.0)
    assert vel2_xyz_with_cov.covariance[12] == pytest.approx(0.0)
    assert vel2_xyz_with_cov.covariance[13] == pytest.approx(0.0)
    assert vel2_xyz_with_cov.covariance[14] == pytest.approx(0.0)
    vel_xyz = Vector3()
    vel_xyz.x = vel2_xyz_with_cov.pose.position.x
    vel_xyz.y = vel2_xyz_with_cov.pose.position.y
    vel_xyz.z = vel2_xyz_with_cov.pose.position.z
    set_velocity_xyz_yaw_with_covariance_from_gm_vector3(obj, vel_xyz, np.pi / 2, vel2_xyz_with_cov.covariance[0], vel2_xyz_with_cov.covariance[7], vel2_xyz_with_cov.covariance[1])
    vel3_xyz_with_cov = get_velocity_xyz_with_covariance(obj)
    assert vel3_xyz_with_cov.pose.position.x == pytest.approx(vel2_xyz_with_cov.pose.position.x)
    assert vel3_xyz_with_cov.pose.position.y == pytest.approx(vel2_xyz_with_cov.pose.position.y)
    assert vel3_xyz_with_cov.pose.position.z == pytest.approx(vel2_xyz_with_cov.pose.position.z)
    assert vel3_xyz_with_cov.covariance[0] == pytest.approx(vel2_xyz_with_cov.covariance[0])
    assert vel3_xyz_with_cov.covariance[1] == pytest.approx(vel2_xyz_with_cov.covariance[1])
    assert vel3_xyz_with_cov.covariance[2] == pytest.approx(vel2_xyz_with_cov.covariance[2])
    assert vel3_xyz_with_cov.covariance[6] == pytest.approx(vel2_xyz_with_cov.covariance[6])
    assert vel3_xyz_with_cov.covariance[7] == pytest.approx(vel2_xyz_with_cov.covariance[7])
    assert vel3_xyz_with_cov.covariance[8] == pytest.approx(vel2_xyz_with_cov.covariance[8])
    assert vel3_xyz_with_cov.covariance[12] == pytest.approx(vel2_xyz_with_cov.covariance[12])
    assert vel3_xyz_with_cov.covariance[13] == pytest.approx(vel2_xyz_with_cov.covariance[13])
    assert vel3_xyz_with_cov.covariance[14] == pytest.approx(vel2_xyz_with_cov.covariance[14])

    # set/getAccelerationXYZ
    initialize_state(obj, EGO.MODEL_ID)
    acc1_lon_lat = Vector3()
    acc2_xyz = Vector3()
    acc3_xyz = Vector3()
    acc1_lon_lat.x = 3.0
    acc1_lon_lat.y = 4.0
    acc1_lon_lat.z = 5.0
    set_acceleration_from_gm_vector3(obj, acc1_lon_lat)
    set_yaw(obj, np.pi / 2)
    acc2_xyz = get_acceleration_xyz(obj)
    assert acc2_xyz.x == pytest.approx(-acc1_lon_lat.y)
    assert acc2_xyz.y == pytest.approx(acc1_lon_lat.x)
    assert acc2_xyz.z == pytest.approx(0.0)
    set_acceleration_xyz_yaw_from_gm_vector3(obj, acc2_xyz, np.pi / 2)
    acc3_xyz = get_acceleration_xyz(obj)
    assert acc3_xyz.x == pytest.approx(acc2_xyz.x)
    assert acc3_xyz.y == pytest.approx(acc2_xyz.y)
    assert acc3_xyz.z == pytest.approx(acc2_xyz.z)

    # set/getAccelerationXYZWithCovariance
    initialize_state(obj, ISCACTR.MODEL_ID)
    acc1_lon_lat = Vector3()
    acc1_lon_lat.x = 3.0
    acc1_lon_lat.y = 4.0
    acc1_lon_lat.z = 5.0
    set_acceleration_from_gm_vector3(obj, acc1_lon_lat)
    var_lon = abs(random_value()) + 0.5
    var_lat = abs(random_value()) + 0.1
    cov_lat_lon = random_value() * max(var_lon, var_lat)
    set_continuous_state_covariance_at(obj, index_acc_lon(obj.state.model_id), index_acc_lon(obj.state.model_id), var_lon)
    set_continuous_state_covariance_at(obj, index_acc_lon(obj.state.model_id), index_acc_lat(obj.state.model_id), cov_lat_lon)
    set_continuous_state_covariance_at(obj, index_acc_lat(obj.state.model_id), index_acc_lon(obj.state.model_id), cov_lat_lon)
    set_continuous_state_covariance_at(obj, index_acc_lat(obj.state.model_id), index_acc_lat(obj.state.model_id), var_lat)
    set_yaw(obj, np.pi / 2)
    acc2_xyz_with_cov = get_acceleration_xyz_with_covariance(obj)
    assert acc2_xyz_with_cov.pose.position.y == pytest.approx(acc1_lon_lat.x)
    assert acc2_xyz_with_cov.pose.position.x == pytest.approx(-acc1_lon_lat.y)
    assert acc2_xyz_with_cov.pose.position.z == pytest.approx(0.0)
    assert acc2_xyz_with_cov.covariance[0] == pytest.approx(var_lat)
    assert acc2_xyz_with_cov.covariance[1] == pytest.approx(-cov_lat_lon)
    assert acc2_xyz_with_cov.covariance[6] == pytest.approx(-cov_lat_lon)
    assert acc2_xyz_with_cov.covariance[7] == pytest.approx(var_lon)
    assert acc2_xyz_with_cov.covariance[2] == pytest.approx(0.0)
    assert acc2_xyz_with_cov.covariance[8] == pytest.approx(0.0)
    assert acc2_xyz_with_cov.covariance[12] == pytest.approx(0.0)
    assert acc2_xyz_with_cov.covariance[13] == pytest.approx(0.0)
    assert acc2_xyz_with_cov.covariance[14] == pytest.approx(0.0)
    acc_xyz = Vector3()
    acc_xyz.x = acc2_xyz_with_cov.pose.position.x
    acc_xyz.y = acc2_xyz_with_cov.pose.position.y
    acc_xyz.z = acc2_xyz_with_cov.pose.position.z
    set_acceleration_xyz_yaw_with_covariance_from_gm_vector3(obj, acc_xyz, np.pi / 2, acc2_xyz_with_cov.covariance[0], acc2_xyz_with_cov.covariance[7], acc2_xyz_with_cov.covariance[1])
    acc3_xyz_with_cov = get_acceleration_xyz_with_covariance(obj)
    assert acc3_xyz_with_cov.pose.position.x == pytest.approx(acc2_xyz_with_cov.pose.position.x)
    assert acc3_xyz_with_cov.pose.position.y == pytest.approx(acc2_xyz_with_cov.pose.position.y)
    assert acc3_xyz_with_cov.pose.position.z == pytest.approx(acc2_xyz_with_cov.pose.position.z)
    assert acc3_xyz_with_cov.covariance[0] == pytest.approx(acc2_xyz_with_cov.covariance[0])
    assert acc3_xyz_with_cov.covariance[1] == pytest.approx(acc2_xyz_with_cov.covariance[1])
    assert acc3_xyz_with_cov.covariance[2] == pytest.approx(acc2_xyz_with_cov.covariance[2])
    assert acc3_xyz_with_cov.covariance[6] == pytest.approx(acc2_xyz_with_cov.covariance[6])
    assert acc3_xyz_with_cov.covariance[7] == pytest.approx(acc2_xyz_with_cov.covariance[7])
    assert acc3_xyz_with_cov.covariance[8] == pytest.approx(acc2_xyz_with_cov.covariance[8])
    assert acc3_xyz_with_cov.covariance[12] == pytest.approx(acc2_xyz_with_cov.covariance[12])
    assert acc3_xyz_with_cov.covariance[13] == pytest.approx(acc2_xyz_with_cov.covariance[13])
    assert acc3_xyz_with_cov.covariance[14] == pytest.approx(acc2_xyz_with_cov.covariance[14])
# ============================================================================
# MIT License
# 
# Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ============================================================================

"""
Convenience setter functions for objects state members.

"""
from typing import TypeVar, Union, List
import math
import tf2_geometry_msgs
from perception_msgs.msg import Object, ObjectState, EgoData
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Vector3, Vector3Stamped, TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from .checks import sanity_check_continuous_state, sanity_check_discrete_state, sanity_check_continuous_state_covariance
from .utils import get_continuous_state_size
from .state_setters import set_x, set_y, set_z, set_roll, set_pitch, set_yaw, set_vel_lon, set_vel_lat, set_acc_lon, set_acc_lat
from .state_index import has_x, has_y, has_z, has_roll, has_pitch, has_yaw, has_vel_lon, has_vel_lat, has_acc_lon, has_acc_lat
from .state_index import index_x, index_y, index_z, index_roll, index_pitch, index_yaw, index_vel_lon, index_vel_lat, index_acc_lon, index_acc_lat

T = TypeVar('T', bound=Union[Object, ObjectState, EgoData])

# --- full state / covariance ------------------------------------------------- 
def set_continuous_state(obj: T, val: List[float]) -> None:
    """Set the continuous state for a given object state.

    Args:
        state: ObjectState instance
        val: List of values to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    state.continuous_state = val
    sanity_check_continuous_state(state)

def set_discrete_state(obj: T, val: List[int]) -> None:
    """Set the discrete state for a given object state.

    Args:
        state: ObjectState instance
        val: List of values to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    state.discrete_state = val
    sanity_check_discrete_state(state)

def set_continuous_state_covariance(obj: T, val: List[float]) -> None:
    """Set the continuous state covariance for a given object state.

    Args:
        state: ObjectState instance
        val: List of values to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    state.continuous_state_covariance = val
    sanity_check_continuous_state_covariance(state)

def set_continuous_state_covariance_at(obj: T, i: int, j: int, val: float) -> None:
    """Set the continuous state covariance at a given index.

    Args:
        state: ObjectState instance
        i: Row index
        j: Column index
        val: Value to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    n = get_continuous_state_size(state)
    state.continuous_state_covariance[n * i + j] = val
    sanity_check_continuous_state_covariance(state)

def set_continuous_state_covariance_diagonal(obj: T, val: List[float]) -> None:
    """Set the diagonal of the continuous state covariance matrix.

    Args:
        state: ObjectState instance
        val: List of values to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    n = get_continuous_state_size(state)
    for i in range(n):
        set_continuous_state_covariance_at(state, i, i, val[i])

# --- vector quantities -------------------------------------------------------

def set_position_from_gm_point(obj: T, val: Point, reset_covariance: bool = True) -> None:
    """Set the position for a given object state.

    Args:
        state: ObjectState instance
        val: Point to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    set_x(state, val.x, reset_covariance)
    set_y(state, val.y, reset_covariance)
    set_z(state, val.z, reset_covariance)

def set_position_from_list(obj: T, val: List[float], reset_covariance: bool = True) -> None:
    """Set the position for a given object state.

    Args:
        state: ObjectState instance
        val: List of values to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    set_x(state, val[0], reset_covariance)
    set_y(state, val[1], reset_covariance)
    set_z(state, val[2], reset_covariance)

def set_orientation_from_gm_quaternion(obj: T, val: Quaternion, reset_covariance: bool = True) -> None:
    """Set the orientation for a given object state.

    Args:
        state: ObjectState instance
        val: Quaternion to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    roll, pitch, yaw = euler_from_quaternion([val.x, val.y, val.z, val.w])
    if has_roll(state.model_id): set_roll(state, roll, reset_covariance)
    if has_pitch(state.model_id): set_pitch(state, pitch, reset_covariance)
    if has_yaw(state.model_id): set_yaw(state, yaw, reset_covariance)

def set_orientation_from_list(obj: T, val: List[float], reset_covariance: bool = True) -> None:
    """Set the orientation for a given object state.

    Args:
        state: ObjectState instance
        val: List of values to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    if has_roll(state.model_id): set_roll(state, val[0], reset_covariance)
    if has_pitch(state.model_id): set_pitch(state, val[1], reset_covariance)
    if has_yaw(state.model_id): set_yaw(state, val[2], reset_covariance)

def set_pose_from_gm_pose(obj: T, val: Pose, reset_covariance: bool = True) -> None:
    """Set the pose for a given object state.

    Args:
        state: ObjectState instance
        val: Pose to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    set_position_from_gm_point(state, val.position, reset_covariance)
    set_orientation_from_gm_quaternion(state, val.orientation, reset_covariance)

def set_pose_from_lists(obj: T, pos: List[float], ori: List[float], reset_covariance: bool = True) -> None:
    """Set the pose for a given object state.

    Args:
        state: ObjectState instance
        pos: List of position values
        ori: List of orientation values
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    set_position_from_list(state, pos, reset_covariance)
    set_orientation_from_list(state, ori, reset_covariance)

def set_pose_covariance_from_list(obj: T, val: List[float]) -> None:
    """Set the pose covariance for a given object state.

    Args:
        state: ObjectState instance
        val: List of values to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    n = 6
    model_id = state.model_id
    for i in range(n):
        for j in range(n):
            ix = 0
            jx = 0
            if i == 0 and has_x(model_id):
                ix = index_x(model_id)
            elif i == 1 and has_y(model_id):
                ix = index_y(model_id)
            elif i == 2 and has_z(model_id):
                ix = index_z(model_id)
            elif i == 3 and has_roll(model_id):
                ix = index_roll(model_id)
            elif i == 4 and has_pitch(model_id):
                ix = index_pitch(model_id)
            elif i == 5 and has_yaw(model_id):
                ix = index_yaw(model_id)
            if j == 0 and has_x(model_id):
                jx = index_x(model_id)
            elif j == 1 and has_y(model_id):
                jx = index_y(model_id)
            elif j == 2 and has_z(model_id):
                jx = index_z(model_id)
            elif j == 3 and has_roll(model_id):
                jx = index_roll(model_id)
            elif j == 4 and has_pitch(model_id):
                jx = index_pitch(model_id)
            elif j == 5 and has_yaw(model_id):
                jx = index_yaw(model_id)
            set_continuous_state_covariance_at(state, ix, jx, val[n * i + j])

def set_pose_with_covariance_from_gm_pose_with_covariance(obj: T, val: PoseWithCovariance, reset_covariance: bool = True) -> None:
    """Set the pose with covariance for a given object state.

    Args:
        state: ObjectState instance
        val: PoseWithCovariance to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    set_pose_from_gm_pose(state, val.pose, reset_covariance)
    set_pose_covariance_from_list(state, val.covariance)

def set_velocity_from_gm_vector3(obj: T, val: Vector3, reset_covariance: bool = True) -> None:
    """Set the velocity for a given object state.

    Args:
        state: ObjectState instance
        val: Vector3 to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    if has_vel_lon(state.model_id): set_vel_lon(state, val.x, reset_covariance)
    if has_vel_lat(state.model_id): set_vel_lat(state, val.y, reset_covariance)

def set_velocity_from_list(obj: T, val: List[float], reset_covariance: bool = True) -> None:
    """Set the velocity for a given object state.

    Args:
        state: ObjectState instance
        val: List of values to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    if has_vel_lon(state.model_id): set_vel_lon(state, val[0], reset_covariance)
    if has_vel_lat(state.model_id): set_vel_lat(state, val[1], reset_covariance)

def set_acceleration_from_gm_vector3(obj: T, val: Vector3, reset_covariance: bool = True) -> None:
    """Set the acceleration for a given object state.

    Args:
        state: ObjectState instance
        val: Vector3 to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    if has_acc_lon(state.model_id): set_acc_lon(state, val.x, reset_covariance)
    if has_acc_lat(state.model_id): set_acc_lat(state, val.y, reset_covariance)

def set_acceleration_from_list(obj: T, val: List[float], reset_covariance: bool = True) -> None:
    """Set the acceleration for a given object state.

    Args:
        state: ObjectState instance
        val: List of values to set
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    if has_acc_lon(state.model_id): set_acc_lon(state, val[0], reset_covariance)
    if has_acc_lat(state.model_id): set_acc_lat(state, val[1], reset_covariance)

# --- alternative state entries ------------------------------------------------

def set_roll_in_deg(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the roll angle for a given object state in degrees.

    Args:
        state: ObjectState instance
        val: Roll angle in degrees
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    set_roll(state, math.radians(val), reset_covariance)

def set_pitch_in_deg(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the pitch angle for a given object state in degrees.

    Args:
        state: ObjectState instance
        val: Pitch angle in degrees
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    set_pitch(state, math.radians(val), reset_covariance)

def set_yaw_in_deg(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the yaw angle for a given object state in degrees.

    Args:
        state: ObjectState instance
        val: Yaw angle in degrees
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    set_yaw(state, math.radians(val), reset_covariance)

def set_velocity_xyz_yaw_with_covariance_from_gm_vector3(obj: T, vel_xyz_in: Vector3, yaw: float, var_vel_x: float, var_vel_y: float, cov_vel_xy: float) -> None:
    """Set the velocity and yaw angle for a given object state.

    Args:
        state: ObjectState instance
        vel_xyz_in: Vector3 to set
        yaw: Yaw angle in radians
        var_vel_x: Variance of the x velocity
        var_vel_y: Variance of the y velocity
        cov_vel_xy: Covariance of the x and y velocity
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    vel_lon_lat = PoseWithCovariance()
    vel_xyz = PoseWithCovariance()
    vel_xyz.pose.position.x = vel_xyz_in.x
    vel_xyz.pose.position.y = vel_xyz_in.y
    vel_xyz.pose.position.z = 0.0

    vel_xyz.covariance[0] = var_vel_x
    vel_xyz.covariance[1] = cov_vel_xy
    vel_xyz.covariance[6] = cov_vel_xy
    vel_xyz.covariance[7] = var_vel_y

    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(0.0, 0.0, -yaw)
    tf = TransformStamped()
    tf.transform.rotation = q
    tf.transform.translation.x = 0.0
    tf.transform.translation.y = 0.0
    tf.transform.translation.z = 0.0
    vel_xyz_stamped = PoseWithCovarianceStamped()
    vel_xyz_stamped.pose = vel_xyz
    vel_lon_lat_stamped = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(vel_xyz_stamped, tf)
    vel_lon_lat = vel_lon_lat_stamped.pose
    set_velocity_from_list(state, [vel_lon_lat.pose.position.x, vel_lon_lat.pose.position.y], False)
    set_yaw(state, yaw, False)
    ix = 0
    jx = 0
    n = 2
    model_id = state.model_id
    for i in range(n):
        for j in range(n):
            if i == 0 and has_vel_lon(model_id):
                ix = index_vel_lon(model_id)
            elif i == 1 and has_vel_lat(model_id):
                ix = index_vel_lat(model_id)
            else:
                continue
            if j == 0 and has_vel_lon(model_id):
                jx = index_vel_lon(model_id)
            elif j == 1 and has_vel_lat(model_id):
                jx = index_vel_lat(model_id)
            else:
                continue

            set_continuous_state_covariance_at(state, ix, jx, vel_lon_lat.covariance[6 * i + j])

def set_velocity_xyz_yaw_from_gm_vector3(obj: T, vel_xyz: Vector3, yaw: float, reset_covariance: bool = True) -> None:
    """Set the velocity and yaw angle for a given object state.

    Args:
        state: ObjectState instance
        vel_xyz: Vector3 to set
        yaw: Yaw angle in radians
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    vel_lon_lat = Vector3()
    vel_xyz_in = vel_xyz
    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(0.0, 0.0, -yaw)
    tf = TransformStamped()
    tf.transform.rotation = q
    tf.transform.translation.x = 0.0
    tf.transform.translation.y = 0.0
    tf.transform.translation.z = 0.0
    vel_xyz_in_stamped = Vector3Stamped()
    vel_xyz_in_stamped.vector = vel_xyz_in
    vel_lon_lat_stamped = tf2_geometry_msgs.do_transform_vector3(vel_xyz_in_stamped, tf)
    vel_lon_lat = vel_lon_lat_stamped.vector
    set_velocity_from_gm_vector3(state, vel_lon_lat, reset_covariance)
    set_yaw(state, yaw, reset_covariance)

def set_acceleration_xyz_yaw_with_covariance_from_gm_vector3(obj: T, acc_xyz_in: Vector3, yaw: float, var_acc_x: float, var_acc_y: float, cov_acc_xy: float) -> None:
    """Set the acceleration and yaw angle for a given object state.

    Args:
        state: ObjectState instance
        acc_xyz_in: Vector3 to set
        yaw: Yaw angle in radians
        var_acc_x: Variance of the x acceleration
        var_acc_y: Variance of the y acceleration
        cov_acc_xy: Covariance of the x and y acceleration
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    acc_lon_lat = PoseWithCovariance()
    acc_xyz = PoseWithCovariance()
    acc_xyz.pose.position.x = acc_xyz_in.x
    acc_xyz.pose.position.y = acc_xyz_in.y
    acc_xyz.pose.position.z = 0.0

    acc_xyz.covariance[0] = var_acc_x
    acc_xyz.covariance[1] = cov_acc_xy
    acc_xyz.covariance[6] = cov_acc_xy
    acc_xyz.covariance[7] = var_acc_y

    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(0.0, 0.0, -yaw)
    tf = TransformStamped()
    tf.transform.rotation = q
    tf.transform.translation.x = 0.0
    tf.transform.translation.y = 0.0
    tf.transform.translation.z = 0.0
    acc_xyz_stamped = PoseWithCovarianceStamped()
    acc_xyz_stamped.pose = acc_xyz
    acc_lon_lat_stamped = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(acc_xyz_stamped, tf)
    acc_lon_lat = acc_lon_lat_stamped.pose
    set_acceleration_from_list(state, [acc_lon_lat.pose.position.x, acc_lon_lat.pose.position.y], False)
    set_yaw(state, yaw, False)

    ix = 0
    jx = 0
    n = 2
    model_id = state.model_id
    for i in range(n):
        for j in range(n):
            if i == 0 and has_acc_lon(model_id):
                ix = index_acc_lon(model_id)
            elif i == 1 and has_acc_lat(model_id):
                ix = index_acc_lat(model_id)
            else:
                continue
            if j == 0 and has_acc_lon(model_id):
                jx = index_acc_lon(model_id)
            elif j == 1 and has_acc_lat(model_id):
                jx = index_acc_lat(model_id)
            else:
                continue

            set_continuous_state_covariance_at(state, ix, jx, acc_lon_lat.covariance[6 * i + j])

def set_acceleration_xyz_yaw_from_gm_vector3(obj: T, acc_xyz: Vector3, yaw: float, reset_covariance: bool = True) -> None:
    """Set the acceleration and yaw angle for a given object state.

    Args:
        state: ObjectState instance
        acc_xyz: Vector3 to set
        yaw: Yaw angle in radians
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    acc_lon_lat = Vector3()
    acc_xyz_in = acc_xyz
    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(0.0, 0.0, -yaw)
    tf = TransformStamped()
    tf.transform.rotation = q
    tf.transform.translation.x = 0.0
    tf.transform.translation.y = 0.0
    tf.transform.translation.z = 0.0
    acc_xyz_in_stamped = Vector3Stamped()
    acc_xyz_in_stamped.vector = acc_xyz_in
    acc_lon_lat_stamped = tf2_geometry_msgs.do_transform_vector3(acc_xyz_in_stamped, tf)
    acc_lon_lat = acc_lon_lat_stamped.vector
    set_acceleration_from_gm_vector3(state, acc_lon_lat, reset_covariance)
    set_yaw(state, yaw, reset_covariance)
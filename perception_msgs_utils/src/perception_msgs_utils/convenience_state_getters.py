"""
Convenience getter functions for objects state members.

"""
from typing import TypeVar, Union, List
import math
import tf2_geometry_msgs
from perception_msgs.msg import ObjectState, Object, EgoData, ObjectClassification
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Vector3, Vector3Stamped, TransformStamped
from tf_transformations import quaternion_from_euler


from .checks import sanity_check_continuous_state, sanity_check_discrete_state, sanity_check_continuous_state_covariance
from .utils import get_continuous_state_size
from .state_getters import get_x, get_y, get_z, get_roll, get_pitch, get_yaw, get_vel_lon, get_vel_lat, get_acc_lon, get_acc_lat
from .state_index import has_x, has_y, has_z, has_roll, has_pitch, has_yaw, has_vel_lon, has_vel_lat, has_acc_lon, has_acc_lat
from .state_index import index_x, index_y, index_z, index_roll, index_pitch, index_yaw, index_vel_lon, index_vel_lat, index_acc_lon, index_acc_lat

T = TypeVar('T', bound=Union[Object, ObjectState, EgoData])

# --- full state/covariance -------------------------------------------------

def get_continuous_state(obj: T) -> List[float]:
    """
    Get the continuous state of an object.

    Args:
        obj: The object to get the state from.

    Returns:
        The continuous state of the object.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state

def get_discrete_state(obj: T) -> List[int]:
    """
    Get the discrete state of an object.

    Args:
        obj: The object to get the state from.

    Returns:
        The discrete state of the object.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_discrete_state(state)
    return state.discrete_state

def get_continuous_state_covariance(obj: T) -> List[float]:
    """
    Get the continuous state covariance of an object.

    Args:
        obj: The object to get the state from.

    Returns:
        The continuous state covariance of the object.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state_covariance(state)
    return state.continuous_state_covariance

def get_continuous_state_covariance_at(obj: T, i: int, j: int) -> float:
    """
    Get the continuous state covariance of an object at a specific index.

    Args:
        obj: The object to get the state from.
        i: The row index of the covariance matrix.
        j: The column index of the covariance matrix.

    Returns:
        The continuous state covariance of the object at the specified index.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    n = get_continuous_state_size(state)
    covariance = get_continuous_state_covariance(state)
    return covariance[i * n + j]

def get_continuous_state_covariance_diagonal(obj: T) -> List[float]:
    """
    Get the diagonal of the continuous state covariance of an object.

    Args:
        obj: The object to get the state from.

    Returns:
        The diagonal of the continuous state covariance of the object.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    n = get_continuous_state_size(state)
    return [get_continuous_state_covariance_at(state, i, i) for i in range(n)]

# --- vector quantities -----------------------------------------------------

def get_position(obj: T) -> Point:
    """
    Get the position of an object.

    Args:
        obj: The object to get the position from.

    Returns:
        The position of the object.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    position = Point()
    position.x = get_x(state)
    position.y = get_y(state)
    position.z = get_z(state)
    return position

def get_orientation(obj: T) -> Quaternion:
    """
    Get the orientation of an object.

    Args:
        obj: The object to get the orientation"
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    roll, pitch, yaw = 0, 0, 0
    if has_roll(state.model_id): roll = get_roll(state)
    if has_pitch(state.model_id): pitch = get_pitch(state)
    if has_yaw(state.model_id): yaw = get_yaw(state)
    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(roll, pitch, yaw)
    return q

def get_pose(obj: T) -> Pose:
    """
    Get the pose of an object.

    Args:
        obj: The object to get the pose"
    """
    pose = Pose()
    pose.position = get_position(obj)
    pose.orientation = get_orientation(obj)
    return pose

def get_pose_covariance(obj: T) -> List[float]:
    """
    Get the pose covariance of an object.

    Args:
        obj: The object to get the pose covariance from.

    Returns:
        The pose covariance of the object.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    n = 6
    model_id = state.model_id
    pose_covariance = [0.0] * n * n
    for i in range(n):
        for j in range(n):
            ix = 0
            jx = 0
            if i == 0 and has_x(model_id): ix = index_x(model_id)
            elif i == 1 and has_y(model_id): ix = index_y(model_id)
            elif i == 2 and has_z(model_id): ix = index_z(model_id)
            elif i == 3 and has_roll(model_id): ix = index_roll(model_id)
            elif i == 4 and has_pitch(model_id): ix = index_pitch(model_id)
            elif i == 5 and has_yaw(model_id): ix = index_yaw(model_id)
            else: continue
            if j == 0 and has_x(model_id): jx = index_x(model_id)
            elif j == 1 and has_y(model_id): jx = index_y(model_id)
            elif j == 2 and has_z(model_id): jx = index_z(model_id)
            elif j == 3 and has_roll(model_id): jx = index_roll(model_id)
            elif j == 4 and has_pitch(model_id): jx = index_pitch(model_id)
            elif j == 5 and has_yaw(model_id): jx = index_yaw(model_id)
            else: continue
            pose_covariance[i * n + j] = get_continuous_state_covariance_at(state, ix, jx)

    return pose_covariance

def get_pose_with_covariance(obj: T) -> PoseWithCovariance:
    """
    Get the pose with covariance of an object.

    Args:
        obj: The object to get the pose with covariance from.

    Returns:
        The pose with covariance of the object.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    pose_with_covariance = PoseWithCovariance()
    pose_with_covariance.pose = get_pose(state)
    pose_with_covariance.covariance = get_pose_covariance(state)
    return pose_with_covariance

def get_velocity(obj: T) -> Vector3:
    """
    Get the velocity of an object.

    Args:
        obj: The object to get the velocity from.

    Returns:
        The velocity of the object.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    velocity = Vector3()
    velocity.x = get_vel_lon(state)
    velocity.y = get_vel_lat(state)
    return velocity

def get_velocity_magnitude(obj: T) -> float:
    """
    Get the magnitude of the velocity of an object.

    Args:
        obj: The object to get the velocity magnitude from.

    Returns:
        The magnitude of the velocity of the object.

    """
    velocity = get_velocity(obj)
    return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

def get_acceleration(obj: T) -> Vector3:
    """
    Get the acceleration of an object.

    Args:
        obj: The object to get the acceleration"
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    acceleration = Vector3()
    acceleration.x = get_acc_lon(state)
    acceleration.y = get_acc_lat(state)
    acceleration.z = 0.0
    return acceleration

def get_acceleration_magnitude(obj: T) -> float:
    """
    Get the magnitude of the acceleration of an object.

    Args:
        obj: The object to get the acceleration magnitude"
    """
    acceleration = get_acceleration(obj)
    return math.sqrt(acceleration.x ** 2 + acceleration.y ** 2 + acceleration.z ** 2)

# --- alternative state entries -----------------------------------------------------

def get_roll_in_deg(obj: T) -> float:
    """
    Get the roll of an object in degrees.

    Args:
        obj: The object to get the roll from.
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    return math.degrees(get_roll(state))

def get_pitch_in_deg(obj: T) -> float:
    """
    Get the pitch of an object in degrees.

    Args:
        obj: The object to get the pitch from.
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    return math.degrees(get_pitch(state))

def get_yaw_in_deg(obj: T) -> float:
    """
    Get the yaw of an object in degrees.

    Args:
        obj: The object to get the yaw from.
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    return math.degrees(get_yaw(state))

def get_velocity_xyz_with_covariance(obj: T) -> PoseWithCovariance:
    """
    Get the velocity of an object in the x, y, and z directions with covariance.

    Args:
        obj: The object to get the velocity from.
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    vel_lon_lat = PoseWithCovariance()
    vel_xyz = PoseWithCovariance()
    vel_lon_lat.pose.position.x = get_vel_lon(state)
    vel_lon_lat.pose.position.y = get_vel_lat(state)
    vel_lon_lat.pose.position.z = 0.0

    ix = 0
    jx = 0
    n = 2
    model_id = state.model_id
    for i in range(n):
        for j in range(n):
            if i == 0 and has_vel_lon(model_id): ix = index_vel_lon(model_id)
            elif i == 1 and has_vel_lat(model_id): ix = index_vel_lat(model_id)
            else: continue
            if j == 0 and has_vel_lon(model_id): jx = index_vel_lon(model_id)
            elif j == 1 and has_vel_lat(model_id): jx = index_vel_lat(model_id)
            else: continue
            vel_lon_lat.covariance[i * 6 + j] = get_continuous_state_covariance_at(state, ix, jx)

    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(0, 0, get_yaw(state))
    tf = TransformStamped()
    tf.transform.rotation = q
    vel_lon_lat_stamped = PoseWithCovarianceStamped()
    vel_lon_lat_stamped.pose = vel_lon_lat
    vel_xyz = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(vel_lon_lat_stamped, tf)
    return vel_xyz.pose

def get_velocity_xyz(obj: T) -> Vector3:
    """
    Get the velocity of an object in the x, y, and z directions.

    Args:
        obj: The object to get the velocity from.
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    vel_lon_lat = Vector3()
    vel_xyz = Vector3()
    vel_lon_lat = get_velocity(obj)

    tf = TransformStamped()
    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(0, 0, get_yaw(state))
    tf.transform.rotation = q
    vel_lon_lat_stamped = Vector3Stamped()
    vel_lon_lat_stamped.vector = vel_lon_lat
    vel_xyz = tf2_geometry_msgs.do_transform_vector3(vel_lon_lat_stamped, tf)

    return vel_xyz.vector

def get_vel_x(obj: T) -> float:
    """
    Get the x-velocity of an object.

    Args:
        obj: The object to get the x-velocity from.
    """
    return get_velocity_xyz(obj).x

def get_vel_y(obj: T) -> float:
    """
    Get the y-velocity of an object.

    Args:
        obj: The object to get the y-velocity from.
    """
    return get_velocity_xyz(obj).y

def get_acceleration_xyz_with_covariance(obj: T) -> PoseWithCovariance:
    """
    Get the acceleration of an object in the x, y, and z directions with covariance.

    Args:
        obj: The object to get the acceleration from.
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    acc_lon_lat = PoseWithCovariance()
    acc_xyz = PoseWithCovariance()
    acc_lon_lat.pose.position.x = get_acc_lon(state)
    acc_lon_lat.pose.position.y = get_acc_lat(state)
    acc_lon_lat.pose.position.z = 0.0

    ix = 0
    jx = 0
    n = 2
    model_id = state.model_id
    for i in range(n):
        for j in range(n):
            if i == 0 and has_acc_lon(model_id): ix = index_acc_lon(model_id)
            elif i == 1 and has_acc_lat(model_id): ix = index_acc_lat(model_id)
            else: continue
            if j == 0 and has_acc_lon(model_id): jx = index_acc_lon(model_id)
            elif j == 1 and has_acc_lat(model_id): jx = index_acc_lat(model_id)
            else: continue
            acc_lon_lat.covariance[i * 6 + j] = get_continuous_state_covariance_at(state, ix, jx)

    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(0, 0, get_yaw(state))
    tf = TransformStamped()
    tf.transform.rotation = q
    acc_lon_lat_stamped = PoseWithCovarianceStamped()
    acc_lon_lat_stamped.pose = acc_lon_lat
    acc_xyz = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(acc_lon_lat_stamped, tf)
    return acc_xyz.pose

def get_acceleration_xyz(obj: T) -> Vector3:
    """
    Get the acceleration of an object in the x, y, and z directions.

    Args:
        obj: The object to get the acceleration from.
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    acc_lon_lat = Vector3()
    acc_xyz = Vector3()
    acc_lon_lat = get_acceleration(obj)

    tf = TransformStamped()
    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(0, 0, get_yaw(state))
    tf.transform.rotation = q
    
    acc_lon_lat_stamped = Vector3Stamped()
    acc_lon_lat_stamped.vector = acc_lon_lat
    acc_xyz = tf2_geometry_msgs.do_transform_vector3(acc_lon_lat_stamped, tf)

    return acc_xyz.vector

def get_acc_x(obj: T) -> float:
    """
    Get the x-acceleration of an object.

    Args:
        obj: The object to get the x-acceleration from.
    """
    return get_acceleration_xyz(obj).x

def get_acc_y(obj: T) -> float:
    """
    Get the y-acceleration of an object.

    Args:
        obj: The object to get the y-acceleration from.
    """
    return get_acceleration_xyz(obj).y

# --- misc --------------------------------------------------------------------

def get_class_with_highest_probability(obj: T) -> ObjectClassification:
    """
    Get the class with the highest probability of an object.

    Args:
        obj: The object to get the class with the highest probability from.

    Returns:
        The class with the highest probability of the object.

    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    class_with_highest_probability = ObjectClassification()
    class_with_highest_probability.probability = 0.0
    for classification in state.classifications:
        if classification.probability > class_with_highest_probability.probability:
            class_with_highest_probability = classification
    return class_with_highest_probability

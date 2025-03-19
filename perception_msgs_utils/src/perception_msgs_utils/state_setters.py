"""
State setter functions for perception message objects.

This module provides functions to modify state information in perception message objects.
"""

from typing import TypeVar, Union
from perception_msgs.msg import ObjectState, Object, EgoData
from .state_index import (
    index_x, index_y, index_z,
    index_vel_lon, index_vel_lat,
    index_acc_lon, index_acc_lat,
    index_roll, index_roll_rate,
    index_pitch, index_pitch_rate,
    index_yaw, index_yaw_rate,
    index_steering_angle_ack, index_steering_angle_rate_ack,
    index_steering_angle_front, index_steering_angle_rear,
    index_width, index_length, index_height, index_standstill,
    index_state, index_type
)
from .checks import sanity_check_continuous_state, sanity_check_discrete_state
from .utils import set_continuous_state_covariance_to_unknown_at

T = TypeVar('T', bound=Union[Object, ObjectState, EgoData])

def set_x(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the x-position for a given object or object state.
    
    Args:
        obj: Object or ObjectState instance
        val: Value to set
        reset_covariance: Whether to reset the covariance matrix
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_x(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_y(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the y-position for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_y(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_z(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the z-position for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_z(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_vel_lon(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the longitudinal velocity for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_vel_lon(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_vel_lat(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the lateral velocity for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_vel_lat(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_acc_lon(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the longitudinal acceleration for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_acc_lon(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_acc_lat(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the lateral acceleration for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_acc_lat(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_roll(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the roll angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_roll(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_roll_rate(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the roll rate for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_roll_rate(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_pitch(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the pitch angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_pitch(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_pitch_rate(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the pitch rate for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_pitch_rate(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_yaw(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the yaw angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_yaw(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_yaw_rate(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the yaw rate for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_yaw_rate(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_steering_angle_ack(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the Ackermann steering angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_steering_angle_ack(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_steering_angle_rate_ack(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the Ackermann steering angle rate for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_steering_angle_rate_ack(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_steering_angle_front(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the front steering angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_steering_angle_front(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_steering_angle_rear(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the rear steering angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_steering_angle_rear(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_width(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the width for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_width(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_length(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the length for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_length(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_height(obj: T, val: float, reset_covariance: bool = True) -> None:
    """Set the height for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    idx = index_height(state.model_id)
    state.continuous_state[idx] = val
    if reset_covariance:
        set_continuous_state_covariance_to_unknown_at(state, idx, idx)

def set_standstill(obj: T, val: bool) -> None:
    """Set the standstill flag for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_discrete_state(state)
    idx = index_standstill(state.model_id)
    state.discrete_state[idx] = val

def set_state(obj: T, val: int) -> None:
    """Set the state for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_discrete_state(state)
    idx = index_state(state.model_id)
    state.discrete_state[idx] = val

def set_type(obj: T, val: int) -> None:
    """Set the type for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_discrete_state(state)
    idx = index_type(state.model_id)
    state.discrete_state[idx] = val
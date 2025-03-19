"""
State getter functions for perception message objects.

This module provides functions to access state information from perception message objects.
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

T = TypeVar('T', bound=Union[Object, ObjectState, EgoData])

def get_x(obj: T) -> float:
    """Get the x-position for a given object or object state.
    
    Args:
        obj: Object or ObjectState instance
        
    Returns:
        float: x-position value
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_x(state.model_id)]

def get_y(obj: T) -> float:
    """Get the y-position for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_y(state.model_id)]

def get_z(obj: T) -> float:
    """Get the z-position for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_z(state.model_id)]

def get_vel_lon(obj: T) -> float:
    """Get the longitudinal velocity for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_vel_lon(state.model_id)]

def get_vel_lat(obj: T) -> float:
    """Get the lateral velocity for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_vel_lat(state.model_id)]

def get_acc_lon(obj: T) -> float:
    """Get the longitudinal acceleration for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_acc_lon(state.model_id)]

def get_acc_lat(obj: T) -> float:
    """Get the lateral acceleration for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_acc_lat(state.model_id)]

def get_roll(obj: T) -> float:
    """Get the roll angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_roll(state.model_id)]

def get_roll_rate(obj: T) -> float:
    """Get the roll rate for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_roll_rate(state.model_id)]

def get_pitch(obj: T) -> float:
    """Get the pitch angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_pitch(state.model_id)]

def get_pitch_rate(obj: T) -> float:
    """Get the pitch rate for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_pitch_rate(state.model_id)]

def get_yaw(obj: T) -> float:
    """Get the yaw angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_yaw(state.model_id)]

def get_yaw_rate(obj: T) -> float:
    """Get the yaw rate for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_yaw_rate(state.model_id)]

def get_steering_angle_ack(obj: T) -> float:
    """Get the Ackermann steering angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_steering_angle_ack(state.model_id)]

def get_steering_angle_rate_ack(obj: T) -> float:
    """Get the Ackermann steering angle rate for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_steering_angle_rate_ack(state.model_id)]

def get_steering_angle_front(obj: T) -> float:
    """Get the front steering angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_steering_angle_front(state.model_id)]

def get_steering_angle_rear(obj: T) -> float:
    """Get the rear steering angle for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_steering_angle_rear(state.model_id)]

def get_width(obj: T) -> float:
    """Get the width for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_width(state.model_id)]

def get_length(obj: T) -> float:
    """Get the length for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_length(state.model_id)]

def get_height(obj: T) -> float:
    """Get the height for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return state.continuous_state[index_height(state.model_id)]

def get_standstill(obj: T) -> bool:
    """Get the standstill state for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_discrete_state(state)
    return state.discrete_state[index_standstill(state.model_id)]

def get_state(obj: T) -> int:
    """Get the traffic light state for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return int(state.discrete_state[index_state(state.model_id)])

def get_type(obj: T) -> int:
    """Get the traffic light type for a given object or object state."""
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    return int(state.discrete_state[index_type(state.model_id)])
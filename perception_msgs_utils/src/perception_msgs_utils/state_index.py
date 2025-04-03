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
Object state vector indices based on state model.

This module provides functions to get indices into state vectors based on the model type.
"""

from typing import Dict, Set
from .constants import (
    EGO_MODEL_ID, EGORWS_MODEL_ID, ISCACTR_MODEL_ID,
    HEXAMOTION_MODEL_ID, TRAFFICLIGHT_MODEL_ID
)
from perception_msgs.msg import EGO, EGORWS, ISCACTR, HEXAMOTION, TRAFFICLIGHT

class UnknownStateEntryError(Exception):
    """Exception raised when a state entry is not supported by a model."""
    pass

# State vector indices for each model
_EGO_INDICES = {
    'x': EGO.X,
    'y': EGO.Y,
    'z': EGO.Z,
    'vel_lon': EGO.VEL_LON,
    'vel_lat': EGO.VEL_LAT,
    'acc_lon': EGO.ACC_LON,
    'acc_lat': EGO.ACC_LAT,
    'roll': EGO.ROLL,
    'pitch': EGO.PITCH,
    'yaw': EGO.YAW,
    'yaw_rate': EGO.YAW_RATE,
    'steering_angle_ack': EGO.STEERING_ANGLE_ACK,
    'steering_angle_rate_ack': EGO.STEERING_ANGLE_RATE_ACK
}

_EGORWS_INDICES = {
    'x': EGORWS.X,
    'y': EGORWS.Y,
    'z': EGORWS.Z,
    'vel_lon': EGORWS.VEL_LON,
    'vel_lat': EGORWS.VEL_LAT,
    'acc_lon': EGORWS.ACC_LON,
    'acc_lat': EGORWS.ACC_LAT,
    'roll': EGORWS.ROLL,
    'pitch': EGORWS.PITCH,
    'yaw': EGORWS.YAW,
    'yaw_rate': EGORWS.YAW_RATE,
    'steering_angle_front': EGORWS.STEERING_ANGLE_FRONT,
    'steering_angle_rear': EGORWS.STEERING_ANGLE_REAR
}

_EGO_DISCRETE_INDICES = {
    'standstill': EGO.STANDSTILL
}

_EGORWS_DISCRETE_INDICES = {
    'standstill': EGORWS.STANDSTILL
}

_ISCACTR_INDICES = {
    'x': ISCACTR.X,
    'y': ISCACTR.Y,
    'z': ISCACTR.Z,
    'vel_lon': ISCACTR.VEL_LON,
    'vel_lat': ISCACTR.VEL_LAT,
    'acc_lon': ISCACTR.ACC_LON,
    'acc_lat': ISCACTR.ACC_LAT,
    'yaw': ISCACTR.YAW,
    'yaw_rate': ISCACTR.YAW_RATE,
    'width': ISCACTR.WIDTH,
    'length': ISCACTR.LENGTH,
    'height': ISCACTR.HEIGHT
}

_HEXAMOTION_INDICES = {
    'x': HEXAMOTION.X,
    'y': HEXAMOTION.Y,
    'z': HEXAMOTION.Z,
    'vel_lon': HEXAMOTION.VEL_LON,
    'vel_lat': HEXAMOTION.VEL_LAT,
    'acc_lon': HEXAMOTION.ACC_LON,
    'acc_lat': HEXAMOTION.ACC_LAT,
    'roll': HEXAMOTION.ROLL,
    'pitch': HEXAMOTION.PITCH,
    'yaw': HEXAMOTION.YAW,
    'roll_rate': HEXAMOTION.ROLL_RATE,
    'pitch_rate': HEXAMOTION.PITCH_RATE,
    'yaw_rate': HEXAMOTION.YAW_RATE,
    'length': HEXAMOTION.LENGTH,
    'width': HEXAMOTION.WIDTH,
    'height': HEXAMOTION.HEIGHT
}

_TRAFFICLIGHT_INDICES = {
    'x': TRAFFICLIGHT.X,
    'y': TRAFFICLIGHT.Y,
    'z': TRAFFICLIGHT.Z
}

_TRAFFICLIGHT_DISCRETE_INDICES = {
    'state': TRAFFICLIGHT.STATE,
    'type': TRAFFICLIGHT.TYPE
}

# Model capabilities (which states are supported by each model)
_MODEL_CAPABILITIES: Dict[int, Set[str]] = {
    EGO_MODEL_ID: set(_EGO_INDICES.keys()),
    EGORWS_MODEL_ID: set(_EGORWS_INDICES.keys()),
    ISCACTR_MODEL_ID: set(_ISCACTR_INDICES.keys()),
    HEXAMOTION_MODEL_ID: set(_HEXAMOTION_INDICES.keys()),
    TRAFFICLIGHT_MODEL_ID: set(_TRAFFICLIGHT_INDICES.keys())
}

# Discrete Model capabilities (which states are supported by each model)
_DISCRETE_MODEL_CAPABILITIES: Dict[int, Set[str]] = {
    EGO_MODEL_ID: set(_EGO_DISCRETE_INDICES.keys()),
    EGORWS_MODEL_ID: set(_EGORWS_DISCRETE_INDICES.keys()),
    TRAFFICLIGHT_MODEL_ID: set(_TRAFFICLIGHT_DISCRETE_INDICES.keys())
}

def _get_index(model_id: int, entry: str) -> int:
    """Get the vector index for a state entry in a given model.
    
    Args:
        model_id: Model identifier
        entry: Name of the state entry
        
    Returns:
        int: Index into the state vector
        
    Raises:
        UnknownStateEntryError: If the entry is not supported by the model
    """
    if model_id == EGO_MODEL_ID:
        indices = _EGO_INDICES
    elif model_id == EGORWS_MODEL_ID:
        indices = _EGORWS_INDICES
    elif model_id == ISCACTR_MODEL_ID:
        indices = _ISCACTR_INDICES
    elif model_id == HEXAMOTION_MODEL_ID:
        indices = _HEXAMOTION_INDICES
    elif model_id == TRAFFICLIGHT_MODEL_ID:
        indices = _TRAFFICLIGHT_INDICES
    else:
        raise UnknownStateEntryError(f"Unknown model ID: {model_id}")
        
    if entry not in indices:
        raise UnknownStateEntryError(f"Model with ID {model_id} does not support entry: {entry}")
        
    return indices[entry]

def _get_discrete_index(model_id: int, entry: str) -> int:
    """Get the vector index for a discrete state entry in a given model.
    
    Args:
        model_id: Model identifier
        entry: Name of the state entry
        
    Returns:
        int: Index into the state vector
        
    Raises:
        UnknownStateEntryError: If the entry is not supported by the model
    """
    if model_id == EGO_MODEL_ID:
        indices = _EGO_DISCRETE_INDICES
    elif model_id == EGORWS_MODEL_ID:
        indices = _EGORWS_DISCRETE_INDICES
    elif model_id == TRAFFICLIGHT_MODEL_ID:
        indices = _TRAFFICLIGHT_DISCRETE_INDICES
    else:
        raise UnknownStateEntryError(f"Unknown model ID: {model_id}")
        
    if entry not in indices:
        raise UnknownStateEntryError(f"Model with ID {model_id} does not support entry: {entry}")
        
    return indices[entry]

def index_x(model_id: int) -> int:
    """Get the vector-index that stores the x-position."""
    return _get_index(model_id, 'x')

def index_y(model_id: int) -> int:
    """Get the vector-index that stores the y-position."""
    return _get_index(model_id, 'y')

def index_z(model_id: int) -> int:
    """Get the vector-index that stores the z-position."""
    return _get_index(model_id, 'z')

def index_vel_lon(model_id: int) -> int:
    """Get the vector-index that stores the longitudinal velocity."""
    return _get_index(model_id, 'vel_lon')

def index_vel_lat(model_id: int) -> int:
    """Get the vector-index that stores the lateral velocity."""
    return _get_index(model_id, 'vel_lat')

def index_acc_lon(model_id: int) -> int:
    """Get the vector-index that stores the longitudinal acceleration."""
    return _get_index(model_id, 'acc_lon')

def index_acc_lat(model_id: int) -> int:
    """Get the vector-index that stores the lateral acceleration."""
    return _get_index(model_id, 'acc_lat')

def index_roll(model_id: int) -> int:
    """Get the vector-index that stores the roll angle."""
    return _get_index(model_id, 'roll')

def index_roll_rate(model_id: int) -> int:
    """Get the vector-index that stores the roll rate."""
    return _get_index(model_id, 'roll_rate')

def index_pitch(model_id: int) -> int:
    """Get the vector-index that stores the pitch angle."""
    return _get_index(model_id, 'pitch')

def index_pitch_rate(model_id: int) -> int:
    """Get the vector-index that stores the pitch rate."""
    return _get_index(model_id, 'pitch_rate')

def index_yaw(model_id: int) -> int:
    """Get the vector-index that stores the yaw angle."""
    return _get_index(model_id, 'yaw')

def index_yaw_rate(model_id: int) -> int:
    """Get the vector-index that stores the yaw rate."""
    return _get_index(model_id, 'yaw_rate')

def index_steering_angle_ack(model_id: int) -> int:
    """Get the vector-index that stores the Ackermann steering angle."""
    return _get_index(model_id, 'steering_angle_ack')

def index_steering_angle_rate_ack(model_id: int) -> int:
    """Get the vector-index that stores the Ackermann steering angle rate."""
    return _get_index(model_id, 'steering_angle_rate_ack')

def index_steering_angle_front(model_id: int) -> int:
    """Get the vector-index that stores the front steering angle."""
    return _get_index(model_id, 'steering_angle_front')

def index_steering_angle_rear(model_id: int) -> int:
    """Get the vector-index that stores the rear steering angle."""
    return _get_index(model_id, 'steering_angle_rear')

def index_width(model_id: int) -> int:
    """Get the vector-index that stores the width."""
    return _get_index(model_id, 'width')

def index_length(model_id: int) -> int:
    """Get the vector-index that stores the length."""
    return _get_index(model_id, 'length')

def index_height(model_id: int) -> int:
    """Get the vector-index that stores the height."""
    return _get_index(model_id, 'height')

def index_standstill(model_id: int) -> int:
    """Get the vector-index that stores the standstill flag."""
    return _get_discrete_index(model_id, 'standstill')

def index_state(model_id: int) -> int:
    """Get the vector-index that stores a state entry."""
    return _get_discrete_index(model_id, 'state')

def index_type(model_id: int) -> int:
    """Get the vector-index that stores a type entry."""
    return _get_discrete_index(model_id, 'type')

def has_x(model_id: int) -> bool:
    """Check if the model supports x-position."""
    return 'x' in _MODEL_CAPABILITIES.get(model_id, set())

def has_y(model_id: int) -> bool:
    """Check if the model supports y-position."""
    return 'y' in _MODEL_CAPABILITIES.get(model_id, set())

def has_z(model_id: int) -> bool:
    """Check if the model supports z-position."""
    return 'z' in _MODEL_CAPABILITIES.get(model_id, set())

def has_vel_lon(model_id: int) -> bool:
    """Check if the model supports longitudinal velocity."""
    return 'vel_lon' in _MODEL_CAPABILITIES.get(model_id, set())

def has_vel_lat(model_id: int) -> bool:
    """Check if the model supports lateral velocity."""
    return 'vel_lat' in _MODEL_CAPABILITIES.get(model_id, set())

def has_acc_lon(model_id: int) -> bool:
    """Check if the model supports longitudinal acceleration."""
    return 'acc_lon' in _MODEL_CAPABILITIES.get(model_id, set())

def has_acc_lat(model_id: int) -> bool:
    """Check if the model supports lateral acceleration."""
    return 'acc_lat' in _MODEL_CAPABILITIES.get(model_id, set())

def has_roll(model_id: int) -> bool:
    """Check if the model supports roll angle."""
    return 'roll' in _MODEL_CAPABILITIES.get(model_id, set())

def has_roll_rate(model_id: int) -> bool:
    """Check if the model supports roll rate."""
    return 'roll_rate' in _MODEL_CAPABILITIES.get(model_id, set())

def has_pitch(model_id: int) -> bool:
    """Check if the model supports pitch angle."""
    return 'pitch' in _MODEL_CAPABILITIES.get(model_id, set())

def has_pitch_rate(model_id: int) -> bool:
    """Check if the model supports pitch rate."""
    return 'pitch_rate' in _MODEL_CAPABILITIES.get(model_id, set())

def has_yaw(model_id: int) -> bool:
    """Check if the model supports yaw angle."""
    return 'yaw' in _MODEL_CAPABILITIES.get(model_id, set())

def has_yaw_rate(model_id: int) -> bool:
    """Check if the model supports yaw rate."""
    return 'yaw_rate' in _MODEL_CAPABILITIES.get(model_id, set())

def has_steering_angle_ack(model_id: int) -> bool:
    """Check if the model supports Ackermann steering angle."""
    return 'steering_angle_ack' in _MODEL_CAPABILITIES.get(model_id, set())

def has_steering_angle_rate_ack(model_id: int) -> bool:
    """Check if the model supports Ackermann steering angle rate."""
    return 'steering_angle_rate_ack' in _MODEL_CAPABILITIES.get(model_id, set())

def has_steering_angle_front(model_id: int) -> bool:
    """Check if the model supports front steering angle."""
    return 'steering_angle_front' in _MODEL_CAPABILITIES.get(model_id, set())

def has_steering_angle_rear(model_id: int) -> bool:
    """Check if the model supports rear steering angle."""
    return 'steering_angle_rear' in _MODEL_CAPABILITIES.get(model_id, set())

def has_width(model_id: int) -> bool:
    """Check if the model supports width."""
    return 'width' in _MODEL_CAPABILITIES.get(model_id, set())

def has_length(model_id: int) -> bool:
    """Check if the model supports length."""
    return 'length' in _MODEL_CAPABILITIES.get(model_id, set())

def has_height(model_id: int) -> bool:
    """Check if the model supports height."""
    return 'height' in _MODEL_CAPABILITIES.get(model_id, set())

def has_standstill(model_id: int) -> bool:
    """Check if the model supports standstill."""
    return 'standstill' in _DISCRETE_MODEL_CAPABILITIES.get(model_id, set())

def has_state(model_id: int) -> bool:
    """Check if the model supports state."""
    return 'state' in _DISCRETE_MODEL_CAPABILITIES.get(model_id, set())

def has_type(model_id: int) -> bool:
    """Check if the model supports type."""
    return 'type' in _DISCRETE_MODEL_CAPABILITIES.get(model_id, set())
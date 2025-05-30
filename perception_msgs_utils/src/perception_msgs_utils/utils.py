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
Object state utility functions.

This module provides utility functions for working with object states.
"""

from typing import TypeVar, Union
from perception_msgs.msg import ObjectState, Object, EgoData
from .constants import (
    EGO_MODEL_ID, EGO_CONTINUOUS_STATE_SIZE, EGO_DISCRETE_STATE_SIZE,
    EGORWS_MODEL_ID, EGORWS_CONTINUOUS_STATE_SIZE, EGORWS_DISCRETE_STATE_SIZE,
    ISCACTR_MODEL_ID, ISCACTR_CONTINUOUS_STATE_SIZE, ISCACTR_DISCRETE_STATE_SIZE,
    HEXAMOTION_MODEL_ID, HEXAMOTION_CONTINUOUS_STATE_SIZE, HEXAMOTION_DISCRETE_STATE_SIZE,
    TRAFFICLIGHT_MODEL_ID, TRAFFICLIGHT_CONTINUOUS_STATE_SIZE, TRAFFICLIGHT_DISCRETE_STATE_SIZE,
    CONTINUOUS_STATE_COVARIANCE_UNKNOWN
)

T = TypeVar('T', bound=Union[Object, ObjectState, EgoData])

class UnknownModelError(Exception):
    """Exception raised when an unknown model ID is encountered."""
    pass

def get_continuous_state_size(obj: Union[T, int]) -> int:
    """Get the continuous state size for a given object, object state, or model ID.
    
    Args:
        obj: Object, ObjectState instance, or model ID
        
    Returns:
        int: Size of the continuous state vector
        
    Raises:
        UnknownModelError: If the model ID is not recognized
    """
    if isinstance(obj, (Object, ObjectState)):
        state = obj if isinstance(obj, ObjectState) else obj.state
        return len(state.continuous_state)
    elif isinstance(obj, int):
        model_id = obj
        if model_id == EGO_MODEL_ID:
            return EGO_CONTINUOUS_STATE_SIZE
        elif model_id == EGORWS_MODEL_ID:
            return EGORWS_CONTINUOUS_STATE_SIZE
        elif model_id == ISCACTR_MODEL_ID:
            return ISCACTR_CONTINUOUS_STATE_SIZE
        elif model_id == HEXAMOTION_MODEL_ID:
            return HEXAMOTION_CONTINUOUS_STATE_SIZE
        elif model_id == TRAFFICLIGHT_MODEL_ID:
            return TRAFFICLIGHT_CONTINUOUS_STATE_SIZE
        else:
            raise UnknownModelError(f"Unknown model ID: {model_id}")
    else:
        raise ValueError(f"Invalid type: {type(obj)}")

def get_discrete_state_size(obj: Union[T, int]) -> int:
    """Get the discrete state size for a given object, object state, or model ID.
    
    Args:
        obj: Object, ObjectState instance, or model ID
        
    Returns:
        int: Size of the discrete state vector
        
    Raises:
        UnknownModelError: If the model ID is not recognized
    """
    if isinstance(obj, (Object, ObjectState)):
        state = obj if isinstance(obj, ObjectState) else obj.state
        return len(state.discrete_state)
    elif isinstance(obj, int):
        model_id = obj
        if model_id == EGO_MODEL_ID:
            return EGO_DISCRETE_STATE_SIZE
        elif model_id == EGORWS_MODEL_ID:
            return EGORWS_DISCRETE_STATE_SIZE
        elif model_id == ISCACTR_MODEL_ID:
            return ISCACTR_DISCRETE_STATE_SIZE
        elif model_id == HEXAMOTION_MODEL_ID:
            return HEXAMOTION_DISCRETE_STATE_SIZE
        elif model_id == TRAFFICLIGHT_MODEL_ID:
            return TRAFFICLIGHT_DISCRETE_STATE_SIZE
        else:
            raise UnknownModelError(f"Unknown model ID: {model_id}")
    else:
        raise ValueError(f"Invalid type: {type(obj)}")

def get_continuous_state_covariance_size(obj: Union[T, int]) -> int:
    """Get the continuous state covariance size for a given object, object state, or model ID.
    
    Args:
        obj: Object, ObjectState instance, or model ID
        
    Returns:
        int: Size of the continuous state covariance matrix (flattened)
    """
    if isinstance(obj, (Object, ObjectState)):
        state = obj if isinstance(obj, ObjectState) else obj.state
        return len(state.continuous_state_covariance)
    elif isinstance(obj, int):
        model_id = obj
        n = get_continuous_state_size(model_id)
        return n * n
    else:
        raise ValueError(f"Invalid type: {type(obj)}")

def set_continuous_state_covariance_to_unknown_at(obj: T, i: int, j: int) -> None:
    """Set the continuous state covariance to unknown at position (i,j).
    
    Args:
        obj: Object or ObjectState instance
        i: Row index
        j: Column index
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    n = get_continuous_state_size(state)
    state.continuous_state_covariance[n * i + j] = CONTINUOUS_STATE_COVARIANCE_UNKNOWN 
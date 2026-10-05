# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

"""
Object state initializers.

This module defines functions to initialize object states.
"""
from typing import TypeVar, Union

from perception_msgs.msg import ObjectState, Object, EgoData
from .utils import get_continuous_state_size, get_discrete_state_size
from .convenience_state_setters import set_continuous_state, set_discrete_state, set_continuous_state_covariance
from .constants import CONTINUOUS_STATE_INIT, DISCRETE_STATE_INIT

T = TypeVar('T', bound=Union[Object, ObjectState, EgoData])

def initialize_state(obj: T, model_id: int) -> ObjectState:
    """
    Initialize an object state message with empty covariance storage.

    Covariance getters expose INVALID diagonals and zero cross terms.

    Args:
        obj: The object to initialize the state from.
        model_id: The model ID to assign to the state.

    Returns:
        The initialized object state message.
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    state.model_id = model_id
    set_continuous_state(state, [CONTINUOUS_STATE_INIT] * get_continuous_state_size(model_id))
    set_discrete_state(state, [DISCRETE_STATE_INIT] * get_discrete_state_size(model_id))
    set_continuous_state_covariance(state, [])
    if isinstance(obj, ObjectState):
        return state
    else:
        obj.state = state
        return obj

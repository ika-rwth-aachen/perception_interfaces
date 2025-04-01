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
Object state initializers.

This module defines functions to initialize object states.
"""
from typing import TypeVar, Union

from perception_msgs.msg import ObjectState, Object, EgoData
from .utils import get_continuous_state_size, get_discrete_state_size, get_continuous_state_covariance_size
from .convenience_state_setters import set_continuous_state, set_discrete_state, set_continuous_state_covariance, set_continuous_state_covariance_diagonal
from .constants import CONTINUOUS_STATE_INIT, DISCRETE_STATE_INIT, CONTINUOUS_STATE_COVARIANCE_INIT, CONTINUOUS_STATE_COVARIANCE_INVALID

T = TypeVar('T', bound=Union[Object, ObjectState, EgoData])

def initialize_state(obj: T, model_id: int) -> ObjectState:
    """
    Initialize an object state message from an object.

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
    set_continuous_state_covariance(state, [CONTINUOUS_STATE_COVARIANCE_INIT] * get_continuous_state_covariance_size(model_id))
    set_continuous_state_covariance_diagonal(state, [CONTINUOUS_STATE_COVARIANCE_INVALID] * get_continuous_state_size(model_id))  
    if isinstance(obj, ObjectState):
        return state
    else:
        obj.state = state
        return obj

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
Object state sanity checks.

This module provides functions to validate object states.
"""

from typing import TypeVar, Union
from perception_msgs.msg import ObjectState, Object, EgoData
from .utils import (
    get_continuous_state_size,
    get_discrete_state_size,
    get_continuous_state_covariance_size
)

T = TypeVar('T', bound=Union[Object, ObjectState, EgoData])

class InvalidStateSizeError(Exception):
    """Exception raised when a state vector has an invalid size."""
    pass

class InvalidDiscreteStateSizeError(Exception):
    """Exception raised when a discrete state vector has an invalid size."""
    pass

class InvalidStateCovarianceSizeError(Exception):
    """Exception raised when a state covariance matrix has an invalid size."""
    pass

def sanity_check_continuous_state_size(state: ObjectState) -> None:
    """Check if the continuous state size matches the expected size for the model.
    
    Args:
        state: ObjectState instance to check
        
    Raises:
        InvalidStateSizeError: If the state size doesn't match the expected size
    """
    exp_state_size = get_continuous_state_size(state.model_id)
    state_size = get_continuous_state_size(state)
    if state_size != exp_state_size:
        raise InvalidStateSizeError(
            f"Invalid continuous state size for model with ID: {state.model_id}, "
            f"{state_size} != {exp_state_size}"
        )

def sanity_check_discrete_state_size(state: ObjectState) -> None:
    """Check if the discrete state size matches the expected size for the model.
    
    Args:
        state: ObjectState instance to check
        
    Raises:
        InvalidDiscreteStateSizeError: If the discrete state size doesn't match the expected size
    """
    exp_discrete_state_size = get_discrete_state_size(state.model_id)
    discrete_state_size = get_discrete_state_size(state)
    if discrete_state_size != exp_discrete_state_size:
        raise InvalidDiscreteStateSizeError(
            f"Invalid discrete state size for model with ID: {state.model_id}, "
            f"{discrete_state_size} != {exp_discrete_state_size}"
        )

def sanity_check_continuous_state_covariance_size(state: ObjectState) -> None:
    """Check if the continuous state covariance size matches the expected size for the model.
    
    Args:
        state: ObjectState instance to check
        
    Raises:
        InvalidStateCovarianceSizeError: If the covariance size doesn't match the expected size
    """
    exp_state_cov_size = get_continuous_state_covariance_size(state.model_id)
    state_cov_size = get_continuous_state_covariance_size(state)
    if state_cov_size != exp_state_cov_size:
        raise InvalidStateCovarianceSizeError(
            f"Invalid continuous state covariance size for model with ID: {state.model_id}, "
            f"{state_cov_size} != {exp_state_cov_size}"
        )

def sanity_check_continuous_state(obj: T) -> None:
    """Check if the continuous state is valid.
    
    Args:
        obj: Object or ObjectState instance to check
        
    Raises:
        InvalidStateSizeError: If the state size is invalid
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state_size(state)

def sanity_check_discrete_state(obj: T) -> None:
    """Check if the discrete state is valid.
    
    Args:
        obj: Object or ObjectState instance to check
        
    Raises:
        InvalidDiscreteStateSizeError: If the discrete state size is invalid
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_discrete_state_size(state)

def sanity_check_continuous_state_covariance(obj: T) -> None:
    """Check if the continuous state covariance is valid.
    
    Args:
        obj: Object or ObjectState instance to check
        
    Raises:
        InvalidStateCovarianceSizeError: If the covariance size is invalid
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state_covariance_size(state)

def sanity_check_full_state(obj: T) -> None:
    """Check if all state components are valid.
    
    Args:
        obj: Object or ObjectState instance to check
        
    Raises:
        InvalidStateSizeError: If the state size is invalid
        InvalidDiscreteStateSizeError: If the discrete state size is invalid
        InvalidStateCovarianceSizeError: If the covariance size is invalid
    """
    state = obj if isinstance(obj, ObjectState) else obj.state
    sanity_check_continuous_state(state)
    sanity_check_discrete_state(state)
    sanity_check_continuous_state_covariance(state) 
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
Object state constants.

This module defines constants used in object state handling.
"""

from perception_msgs.msg import EGO, EGORWS, ISCACTR, HEXAMOTION, TRAFFICLIGHT
import sys

# Model IDs
EGO_MODEL_ID = EGO.MODEL_ID
EGORWS_MODEL_ID = EGORWS.MODEL_ID
ISCACTR_MODEL_ID = ISCACTR.MODEL_ID
HEXAMOTION_MODEL_ID = HEXAMOTION.MODEL_ID
TRAFFICLIGHT_MODEL_ID = TRAFFICLIGHT.MODEL_ID

# State sizes for each model
EGO_CONTINUOUS_STATE_SIZE = EGO.CONTINUOUS_STATE_SIZE
EGO_DISCRETE_STATE_SIZE = EGO.DISCRETE_STATE_SIZE
EGORWS_CONTINUOUS_STATE_SIZE = EGORWS.CONTINUOUS_STATE_SIZE
EGORWS_DISCRETE_STATE_SIZE = EGORWS.DISCRETE_STATE_SIZE
ISCACTR_CONTINUOUS_STATE_SIZE = ISCACTR.CONTINUOUS_STATE_SIZE
ISCACTR_DISCRETE_STATE_SIZE = ISCACTR.DISCRETE_STATE_SIZE
HEXAMOTION_CONTINUOUS_STATE_SIZE = HEXAMOTION.CONTINUOUS_STATE_SIZE
HEXAMOTION_DISCRETE_STATE_SIZE = HEXAMOTION.DISCRETE_STATE_SIZE
TRAFFICLIGHT_CONTINUOUS_STATE_SIZE = TRAFFICLIGHT.CONTINUOUS_STATE_SIZE
TRAFFICLIGHT_DISCRETE_STATE_SIZE = TRAFFICLIGHT.DISCRETE_STATE_SIZE

# State initialization values
CONTINUOUS_STATE_INIT = 0.0
DISCRETE_STATE_INIT = 0
CONTINUOUS_STATE_COVARIANCE_INIT = 0.0
CONTINUOUS_STATE_COVARIANCE_INVALID = -1.0
CONTINUOUS_STATE_COVARIANCE_UNKNOWN = sys.float_info.max 
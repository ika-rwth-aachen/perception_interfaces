/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

/**
 * @file constants.h
 * @brief Object state constants
 */

#pragma once

#include <cmath>
#include <limits>


namespace perception_msgs {

namespace object_access {

    const double CONTINUOUS_STATE_INIT = 0;

    const long int DISCRETE_STATE_INIT = 0;

    const double CONTINUOUS_STATE_COVARIANCE_INIT = 0;
    const double CONTINUOUS_STATE_COVARIANCE_INVALID = -1;
    const double CONTINUOUS_STATE_COVARIANCE_UNKNOWN = std::numeric_limits<double>::max();

} // namespace object_access

} // namespace perception_msgs

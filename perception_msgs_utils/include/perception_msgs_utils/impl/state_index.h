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
 * @file state_index.h
 * @brief Object state vector indices based on state model
 */

#pragma once


namespace perception_msgs {

namespace object_access {

  const std::string kExceptionUnknownStateEntry = "Model with the following ID does not support requested entry: ";

  /**
   * @brief Get the vector-index that stores the x-position for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexX(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::X;
      case EGORWS::MODEL_ID:
        return EGORWS::X;
      case ISCACTR::MODEL_ID:
        return ISCACTR::X;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::X;
      case TRAFFICLIGHT::MODEL_ID:
        return TRAFFICLIGHT::X;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "x");
    }
  }

  /**
   * @brief Get the vector-index that stores the y-position for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexY(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::Y;
      case EGORWS::MODEL_ID:
        return EGORWS::Y;
      case ISCACTR::MODEL_ID:
        return ISCACTR::Y;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::Y;
      case TRAFFICLIGHT::MODEL_ID:
        return TRAFFICLIGHT::Y;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "y");
    }
  }

  /**
   * @brief Get the vector-index that stores the z-position for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexZ(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::Z;
      case EGORWS::MODEL_ID:
        return EGORWS::Z;
      case ISCACTR::MODEL_ID:
        return ISCACTR::Z;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::Z;
      case TRAFFICLIGHT::MODEL_ID:
        return TRAFFICLIGHT::Z;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "z");
    }
  }

  /**
   * @brief Get the vector-index that stores the longitudinal velocity for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexVelLon(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::VEL_LON;
      case EGORWS::MODEL_ID:
        return EGORWS::VEL_LON;
      case ISCACTR::MODEL_ID:
        return ISCACTR::VEL_LON;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::VEL_LON;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "vel_lon");
    }
  }

  /**
   * @brief Get the vector-index that stores the lateral velocity for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexVelLat(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::VEL_LAT;
      case EGORWS::MODEL_ID:
        return EGORWS::VEL_LAT;
      case ISCACTR::MODEL_ID:
        return ISCACTR::VEL_LAT;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::VEL_LAT;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "vel_lat");
    }
  }

  /**
   * @brief Get the vector-index that stores the longitudinal acceleration for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexAccLon(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::ACC_LON;
      case EGORWS::MODEL_ID:
        return EGORWS::ACC_LON;
      case ISCACTR::MODEL_ID:
        return ISCACTR::ACC_LON;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::ACC_LON;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "acc_lon");
    }
  }

  /**
   * @brief Get the vector-index that stores the lateral acceleration for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexAccLat(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::ACC_LAT;
      case EGORWS::MODEL_ID:
        return EGORWS::ACC_LAT;
      case ISCACTR::MODEL_ID:
        return ISCACTR::ACC_LAT;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::ACC_LAT;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "acc_lat");
    }
  }

  /**
   * @brief Get the vector-index that stores the roll for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexRoll(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::ROLL;
      case EGORWS::MODEL_ID:
        return EGORWS::ROLL;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::ROLL;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "roll");
    }
  }

  /**
   * @brief Get the vector-index that stores the roll rate for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexRollRate(const unsigned char& model_id) {
    switch(model_id) {
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::ROLL_RATE;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "roll_rate");
    }
  }

  /**
   * @brief Get the vector-index that stores the pitch for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexPitch(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::PITCH;
      case EGORWS::MODEL_ID:
        return EGORWS::PITCH;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::PITCH;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "pitch");
    }
  }

  /**
   * @brief Get the vector-index that stores the pitch rate for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexPitchRate(const unsigned char& model_id) {
    switch(model_id) {
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::PITCH_RATE;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "pitch_rate");
    }
  }

  /**
   * @brief Get the vector-index that stores the yaw for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexYaw(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::YAW;
      case EGORWS::MODEL_ID:
        return EGORWS::YAW;
      case ISCACTR::MODEL_ID:
        return ISCACTR::YAW;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::YAW;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "yaw");
    }
  }

  /**
   * @brief Get the vector-index that stores the yaw-rate for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexYawRate(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::YAW_RATE;
      case EGORWS::MODEL_ID:
        return EGORWS::YAW_RATE;
      case ISCACTR::MODEL_ID:
        return ISCACTR::YAW_RATE;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::YAW_RATE;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "yaw_rate");
    }
  }

  /**
   * @brief Get the vector-index that stores the ackermann steering angle for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexSteeringAngleAck(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::STEERING_ANGLE_ACK;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "steering_angle_ack");
    }
  }

  /**
   * @brief Get the vector-index that stores the steering angle rate for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexSteeringAngleRateAck(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::STEERING_ANGLE_RATE_ACK;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "steering_angle_rate_ack");
    }
  }

  /**
   * @brief Get the vector-index that stores the front wheel angle for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexSteeringAngleFront(const unsigned char& model_id) {
    switch(model_id) {
      case EGORWS::MODEL_ID:
        return EGORWS::STEERING_ANGLE_FRONT;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "steering_angle_front");
    }
  }

  /**
   * @brief Get the vector-index that stores the rear wheel angle for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexSteeringAngleRear(const unsigned char& model_id) {
    switch(model_id) {
      case EGORWS::MODEL_ID:
        return EGORWS::STEERING_ANGLE_REAR;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "steering_angle_rear");
    }
  }

  /**
   * @brief Get the vector-index that stores the width for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexWidth(const unsigned char& model_id) {
    switch(model_id) {
      case ISCACTR::MODEL_ID:
        return ISCACTR::WIDTH;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::WIDTH;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "width");
    }
  }

  /**
   * @brief Get the vector-index that stores the length for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexLength(const unsigned char& model_id) {
    switch(model_id) {
      case ISCACTR::MODEL_ID:
        return ISCACTR::LENGTH;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::LENGTH;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "length");
    }
  }

  /**
   * @brief Get the vector-index that stores the height for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexHeight(const unsigned char& model_id) {
    switch(model_id) {
      case ISCACTR::MODEL_ID:
        return ISCACTR::HEIGHT;
      case HEXAMOTION::MODEL_ID:
        return HEXAMOTION::HEIGHT;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "height");
    }
  }

  /**
   * @brief Get the vector-index that stores the standstill indication for a given model-id.
   *
   * @param model_id
   * @return int
   */
  inline int indexStandstill(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return EGO::STANDSTILL;
      case EGORWS::MODEL_ID:
        return EGORWS::STANDSTILL;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "standstill");
    }
  }

  /**
   * @brief Get the vector-index that stores the traffic light state for a given model-id.
   *
   * @param model_id
   *
   * @return int
  */
  inline int indexTrafficLightState(const unsigned char& model_id) {
    switch(model_id) {
      case TRAFFICLIGHT::MODEL_ID:
        return TRAFFICLIGHT::STATE;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "traffic_light_state");
    }
  }

  /**
   * @brief Get the vector-index that stores the traffic light type for a given model-id.
   *
   * @param model_id
   *
   * @return int
  */
  inline int indexTrafficLightType(const unsigned char& model_id) {
    switch(model_id) {
      case TRAFFICLIGHT::MODEL_ID:
        return TRAFFICLIGHT::TYPE;
      default:
        throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(model_id) + ", " + "traffic_light_type");
    }
  }

  /**
   * @brief Indicates if given model contains an x-position.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasX(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      case TRAFFICLIGHT::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a y-position.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasY(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      case TRAFFICLIGHT::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a z-position.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasZ(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      case TRAFFICLIGHT::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a longitudinal velocity.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasVelLon(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a lateral velocity.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasVelLat(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a longitudinal acceleration.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasAccLon(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a lateral acceleration.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasAccLat(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a roll.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasRoll(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return false;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a roll-rate.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasRollRate(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return false;
      case EGORWS::MODEL_ID:
        return false;
      case ISCACTR::MODEL_ID:
        return false;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a pitch.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasPitch(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return false;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a pitch-rate.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasPitchRate(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return false;
      case EGORWS::MODEL_ID:
        return false;
      case ISCACTR::MODEL_ID:
        return false;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a yaw.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasYaw(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a yaw-rate.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasYawRate(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains an ackermann steering angle.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasSteeringAngleAck(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return false;
      case ISCACTR::MODEL_ID:
        return false;
      case HEXAMOTION::MODEL_ID:
        return false;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains an ackermann steering angle rate.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasSteeringAngleRateAck(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return false;
      case ISCACTR::MODEL_ID:
        return false;
      case HEXAMOTION::MODEL_ID:
        return false;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a front wheel angle.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasSteeringAngleFront(const unsigned char& model_id) {
    switch(model_id) {
      case EGORWS::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a rear wheel angle.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasSteeringAngleRear(const unsigned char& model_id) {
    switch(model_id) {
      case EGORWS::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a width.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasWidth(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return false;
      case EGORWS::MODEL_ID:
        return false;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a length
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasLength(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return false;
      case EGORWS::MODEL_ID:
        return false;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a height.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasHeight(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return false;
      case EGORWS::MODEL_ID:
        return false;
      case ISCACTR::MODEL_ID:
        return true;
      case HEXAMOTION::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a standstill indication.
   *
   * @param model_id
   * @return true
   * @return false
   */
  inline bool hasStandstill(const unsigned char& model_id) {
    switch(model_id) {
      case EGO::MODEL_ID:
        return true;
      case EGORWS::MODEL_ID:
        return true;
      case ISCACTR::MODEL_ID:
        return false;
      case HEXAMOTION::MODEL_ID:
        return false;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a traffic light state.
   *
   * @param model_id
   *
   * @return true
   * @return false
  */
  inline bool hasTrafficLightState(const unsigned char& model_id) {
    switch(model_id) {
      case TRAFFICLIGHT::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

  /**
   * @brief Indicates if given model contains a traffic light type.
   *
   * @param model_id
   *
   * @return true
   * @return false
  */
  inline bool hasTrafficLightType(const unsigned char& model_id) {
    switch(model_id) {
      case TRAFFICLIGHT::MODEL_ID:
        return true;
      default:
        return false;
    }
  }

} // namespace object_access

} // namespace perception_msgs

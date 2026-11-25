#pragma once
#include <cstdint>
#include <perception_msgs/msg/object_classification.hpp>
#include <string>
namespace perception_msgs::object_access {
using perception_msgs::msg::ObjectClassification;
inline std::string taxonomyLabel(uint8_t id) {
  switch (id) {
  case ObjectClassification::TRAFFIC_LIGHT:
    return "traffic_light";
  case ObjectClassification::TRAFFIC_LIGHT_UNKNOWN:
    return "traffic_light_unknown";
  case ObjectClassification::TRAFFIC_LIGHT_OFF:
    return "traffic_light_off";
  case ObjectClassification::TRAFFIC_LIGHT_RED:
    return "traffic_light_red";
  case ObjectClassification::TRAFFIC_LIGHT_YELLOW:
    return "traffic_light_yellow";
  case ObjectClassification::TRAFFIC_LIGHT_GREEN:
    return "traffic_light_green";
  case ObjectClassification::TRAFFIC_LIGHT_RED_YELLOW:
    return "traffic_light_red_yellow";
  case ObjectClassification::TRAFFIC_LIGHT_LEFT_RED:
    return "traffic_light_left_red";
  case ObjectClassification::TRAFFIC_LIGHT_LEFT_RED_STRAIGHT:
    return "traffic_light_left_red_straight";
  case ObjectClassification::TRAFFIC_LIGHT_RED_RIGHT:
    return "traffic_light_red_right";
  case ObjectClassification::TRAFFIC_LIGHT_RED_STRAIGHT:
    return "traffic_light_red_straight";
  case ObjectClassification::TRAFFIC_LIGHT_RED_UP_LEFT:
    return "traffic_light_red_up_left";
  case ObjectClassification::TRAFFIC_LIGHT_RED_RIGHT_STRAIGHT:
    return "traffic_light_red_right_straight";
  case ObjectClassification::TRAFFIC_LIGHT_RED_UP_RIGHT:
    return "traffic_light_red_up_right";
  case ObjectClassification::TRAFFIC_SIGN:
    return "traffic_sign";
  case ObjectClassification::TRAFFIC_SIGN_UNKNOWN:
    return "traffic_sign_unknown";
  case ObjectClassification::TRAFFIC_SIGN_STOP:
    return "traffic_sign_stop";
  case ObjectClassification::TRAFFIC_SIGN_GIVE_WAY:
    return "traffic_sign_give_way";
  case ObjectClassification::TRAFFIC_SIGN_PRIORITY_ROAD:
    return "traffic_sign_priority_road";
  case ObjectClassification::TRAFFIC_SIGN_PRIORITY_AT_NEXT_INTERSECTION:
    return "traffic_sign_priority_at_next_intersection";
  case ObjectClassification::TRAFFIC_SIGN_NO_ENTRY:
    return "traffic_sign_no_entry";
  case ObjectClassification::TRAFFIC_SIGN_NO_VEHICLES:
    return "traffic_sign_no_vehicles";
  case ObjectClassification::TRAFFIC_SIGN_NO_GOODS_VEHICLES_OVER_3_5_T:
    return "traffic_sign_no_goods_vehicles_over_3_5_t";
  case ObjectClassification::TRAFFIC_SIGN_NO_OVERTAKING:
    return "traffic_sign_no_overtaking";
  case ObjectClassification::TRAFFIC_SIGN_NO_OVERTAKING_GOODS_VEHICLES:
    return "traffic_sign_no_overtaking_goods_vehicles";
  case ObjectClassification::TRAFFIC_SIGN_END_NO_OVERTAKING:
    return "traffic_sign_end_no_overtaking";
  case ObjectClassification::TRAFFIC_SIGN_END_NO_OVERTAKING_GOODS_VEHICLES:
    return "traffic_sign_end_no_overtaking_goods_vehicles";
  case ObjectClassification::TRAFFIC_SIGN_END_ALL_SPEED_AND_OVERTAKING_RESTRICTIONS:
    return "traffic_sign_end_all_speed_and_overtaking_restrictions";
  case ObjectClassification::TRAFFIC_SIGN_SPEED_LIMIT_20_KMH:
    return "traffic_sign_speed_limit_20_kmh";
  case ObjectClassification::TRAFFIC_SIGN_SPEED_LIMIT_30_KMH:
    return "traffic_sign_speed_limit_30_kmh";
  case ObjectClassification::TRAFFIC_SIGN_SPEED_LIMIT_50_KMH:
    return "traffic_sign_speed_limit_50_kmh";
  case ObjectClassification::TRAFFIC_SIGN_SPEED_LIMIT_60_KMH:
    return "traffic_sign_speed_limit_60_kmh";
  case ObjectClassification::TRAFFIC_SIGN_SPEED_LIMIT_70_KMH:
    return "traffic_sign_speed_limit_70_kmh";
  case ObjectClassification::TRAFFIC_SIGN_SPEED_LIMIT_80_KMH:
    return "traffic_sign_speed_limit_80_kmh";
  case ObjectClassification::TRAFFIC_SIGN_SPEED_LIMIT_100_KMH:
    return "traffic_sign_speed_limit_100_kmh";
  case ObjectClassification::TRAFFIC_SIGN_SPEED_LIMIT_120_KMH:
    return "traffic_sign_speed_limit_120_kmh";
  case ObjectClassification::TRAFFIC_SIGN_END_SPEED_LIMIT_80_KMH:
    return "traffic_sign_end_speed_limit_80_kmh";
  case ObjectClassification::TRAFFIC_SIGN_DANGER:
    return "traffic_sign_danger";
  case ObjectClassification::TRAFFIC_SIGN_CURVE_LEFT:
    return "traffic_sign_curve_left";
  case ObjectClassification::TRAFFIC_SIGN_CURVE_RIGHT:
    return "traffic_sign_curve_right";
  case ObjectClassification::TRAFFIC_SIGN_DOUBLE_CURVE_FIRST_LEFT:
    return "traffic_sign_double_curve_first_left";
  case ObjectClassification::TRAFFIC_SIGN_UNEVEN_ROAD:
    return "traffic_sign_uneven_road";
  case ObjectClassification::TRAFFIC_SIGN_SLIPPERY_ROAD:
    return "traffic_sign_slippery_road";
  case ObjectClassification::TRAFFIC_SIGN_ROAD_NARROWS_RIGHT:
    return "traffic_sign_road_narrows_right";
  case ObjectClassification::TRAFFIC_SIGN_ROAD_WORKS:
    return "traffic_sign_road_works";
  case ObjectClassification::TRAFFIC_SIGN_TRAFFIC_SIGNALS:
    return "traffic_sign_traffic_signals";
  case ObjectClassification::TRAFFIC_SIGN_PEDESTRIANS:
    return "traffic_sign_pedestrians";
  case ObjectClassification::TRAFFIC_SIGN_CHILDREN:
    return "traffic_sign_children";
  case ObjectClassification::TRAFFIC_SIGN_CYCLISTS:
    return "traffic_sign_cyclists";
  case ObjectClassification::TRAFFIC_SIGN_ICE_OR_SNOW:
    return "traffic_sign_ice_or_snow";
  case ObjectClassification::TRAFFIC_SIGN_WILD_ANIMALS:
    return "traffic_sign_wild_animals";
  case ObjectClassification::TRAFFIC_SIGN_TURN_RIGHT:
    return "traffic_sign_turn_right";
  case ObjectClassification::TRAFFIC_SIGN_TURN_LEFT:
    return "traffic_sign_turn_left";
  case ObjectClassification::TRAFFIC_SIGN_STRAIGHT_AHEAD:
    return "traffic_sign_straight_ahead";
  case ObjectClassification::TRAFFIC_SIGN_STRAIGHT_OR_RIGHT:
    return "traffic_sign_straight_or_right";
  case ObjectClassification::TRAFFIC_SIGN_STRAIGHT_OR_LEFT:
    return "traffic_sign_straight_or_left";
  case ObjectClassification::TRAFFIC_SIGN_KEEP_RIGHT:
    return "traffic_sign_keep_right";
  case ObjectClassification::TRAFFIC_SIGN_KEEP_LEFT:
    return "traffic_sign_keep_left";
  case ObjectClassification::TRAFFIC_SIGN_ROUNDABOUT:
    return "traffic_sign_roundabout";
  default:
    return "";
  }
}
inline uint8_t trafficLightClass(const std::string &label) {
  if (label == "green")
    return ObjectClassification::TRAFFIC_LIGHT_GREEN;
  if (label == "left,red")
    return ObjectClassification::TRAFFIC_LIGHT_LEFT_RED;
  if (label == "left,red,straight")
    return ObjectClassification::TRAFFIC_LIGHT_LEFT_RED_STRAIGHT;
  if (label == "red")
    return ObjectClassification::TRAFFIC_LIGHT_RED;
  if (label == "red,right")
    return ObjectClassification::TRAFFIC_LIGHT_RED_RIGHT;
  if (label == "red,straight")
    return ObjectClassification::TRAFFIC_LIGHT_RED_STRAIGHT;
  if (label == "unknown")
    return ObjectClassification::TRAFFIC_LIGHT_UNKNOWN;
  if (label == "yellow")
    return ObjectClassification::TRAFFIC_LIGHT_YELLOW;
  if (label == "red,up_left")
    return ObjectClassification::TRAFFIC_LIGHT_RED_UP_LEFT;
  if (label == "red,right,straight")
    return ObjectClassification::TRAFFIC_LIGHT_RED_RIGHT_STRAIGHT;
  if (label == "red,up_right")
    return ObjectClassification::TRAFFIC_LIGHT_RED_UP_RIGHT;
  if (label == "off")
    return ObjectClassification::TRAFFIC_LIGHT_OFF;
  if (label == "red,yellow")
    return ObjectClassification::TRAFFIC_LIGHT_RED_YELLOW;
  return ObjectClassification::TRAFFIC_LIGHT_UNKNOWN;
}
inline bool isTrafficLightClass(uint8_t id) { return id >= ObjectClassification::TRAFFIC_LIGHT && id <= ObjectClassification::TRAFFIC_LIGHT_RED_UP_RIGHT; }
inline bool isTrafficSignClass(uint8_t id) { return id >= ObjectClassification::TRAFFIC_SIGN && id <= ObjectClassification::TRAFFIC_SIGN_ROUNDABOUT; }
inline uint8_t trafficLightColor(uint8_t id) {
  if (id == ObjectClassification::TRAFFIC_LIGHT_GREEN) return 4;
  if (id == ObjectClassification::TRAFFIC_LIGHT_YELLOW) return 3;
  if (id == ObjectClassification::TRAFFIC_LIGHT_RED_YELLOW) return 2;
  if (id == ObjectClassification::TRAFFIC_LIGHT_OFF) return 255;
  return id == ObjectClassification::TRAFFIC_LIGHT_RED || (id >= ObjectClassification::TRAFFIC_LIGHT_LEFT_RED && id <= ObjectClassification::TRAFFIC_LIGHT_RED_UP_RIGHT) ? 1 : 0;
}
} // namespace perception_msgs::object_access

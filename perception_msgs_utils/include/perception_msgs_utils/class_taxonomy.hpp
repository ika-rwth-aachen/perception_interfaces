#pragma once
#include <cstdint>
#include <perception_msgs/msg/object_class_taxonomy.hpp>
#include <string>
namespace perception_msgs::object_access {
inline std::string taxonomyLabel(uint8_t id) {
  switch (id) {
  case 101:
    return "traffic_light";
  case 102:
    return "traffic_light_unknown";
  case 103:
    return "traffic_light_off";
  case 104:
    return "traffic_light_red";
  case 105:
    return "traffic_light_yellow";
  case 106:
    return "traffic_light_green";
  case 107:
    return "traffic_light_red_yellow";
  case 108:
    return "traffic_light_left_red";
  case 109:
    return "traffic_light_left_red_straight";
  case 110:
    return "traffic_light_red_right";
  case 111:
    return "traffic_light_red_straight";
  case 112:
    return "traffic_light_red_up_left";
  case 113:
    return "traffic_light_red_right_straight";
  case 114:
    return "traffic_light_red_up_right";
  case 128:
    return "traffic_sign";
  case 129:
    return "traffic_sign_unknown";
  case 130:
    return "traffic_sign_stop";
  case 131:
    return "traffic_sign_give_way";
  case 132:
    return "traffic_sign_priority_road";
  case 133:
    return "traffic_sign_priority_at_next_intersection";
  case 134:
    return "traffic_sign_no_entry";
  case 135:
    return "traffic_sign_no_vehicles";
  case 136:
    return "traffic_sign_no_goods_vehicles_over_3_5_t";
  case 137:
    return "traffic_sign_no_overtaking";
  case 138:
    return "traffic_sign_no_overtaking_goods_vehicles";
  case 139:
    return "traffic_sign_end_no_overtaking";
  case 140:
    return "traffic_sign_end_no_overtaking_goods_vehicles";
  case 141:
    return "traffic_sign_end_all_speed_and_overtaking_restrictions";
  case 142:
    return "traffic_sign_speed_limit_20_kmh";
  case 143:
    return "traffic_sign_speed_limit_30_kmh";
  case 144:
    return "traffic_sign_speed_limit_50_kmh";
  case 145:
    return "traffic_sign_speed_limit_60_kmh";
  case 146:
    return "traffic_sign_speed_limit_70_kmh";
  case 147:
    return "traffic_sign_speed_limit_80_kmh";
  case 148:
    return "traffic_sign_speed_limit_100_kmh";
  case 149:
    return "traffic_sign_speed_limit_120_kmh";
  case 150:
    return "traffic_sign_end_speed_limit_80_kmh";
  case 151:
    return "traffic_sign_danger";
  case 152:
    return "traffic_sign_curve_left";
  case 153:
    return "traffic_sign_curve_right";
  case 154:
    return "traffic_sign_double_curve_first_left";
  case 155:
    return "traffic_sign_uneven_road";
  case 156:
    return "traffic_sign_slippery_road";
  case 157:
    return "traffic_sign_road_narrows_right";
  case 158:
    return "traffic_sign_road_works";
  case 159:
    return "traffic_sign_traffic_signals";
  case 160:
    return "traffic_sign_pedestrians";
  case 161:
    return "traffic_sign_children";
  case 162:
    return "traffic_sign_cyclists";
  case 163:
    return "traffic_sign_ice_or_snow";
  case 164:
    return "traffic_sign_wild_animals";
  case 165:
    return "traffic_sign_turn_right";
  case 166:
    return "traffic_sign_turn_left";
  case 167:
    return "traffic_sign_straight_ahead";
  case 168:
    return "traffic_sign_straight_or_right";
  case 169:
    return "traffic_sign_straight_or_left";
  case 170:
    return "traffic_sign_keep_right";
  case 171:
    return "traffic_sign_keep_left";
  case 172:
    return "traffic_sign_roundabout";
  default:
    return "";
  }
}
inline uint8_t trafficLightClass(const std::string &label) {
  if (label == "green")
    return 106;
  if (label == "left,red")
    return 108;
  if (label == "left,red,straight")
    return 109;
  if (label == "red")
    return 104;
  if (label == "red,right")
    return 110;
  if (label == "red,straight")
    return 111;
  if (label == "unknown")
    return 102;
  if (label == "yellow")
    return 105;
  if (label == "red,up_left")
    return 112;
  if (label == "red,right,straight")
    return 113;
  if (label == "red,up_right")
    return 114;
  if (label == "off")
    return 103;
  if (label == "red,yellow")
    return 107;
  return 102;
}
inline bool isTrafficLightClass(uint8_t id) { return id >= 101 && id <= 114; }
inline bool isTrafficSignClass(uint8_t id) { return id >= 128 && id <= 172; }
inline uint8_t trafficLightColor(uint8_t id) {
  if (id == 106) return 4;
  if (id == 105) return 3;
  if (id == 107) return 2;
  if (id == 103) return 255;
  return id == 104 || (id >= 108 && id <= 114) ? 1 : 0;
}
} // namespace perception_msgs::object_access

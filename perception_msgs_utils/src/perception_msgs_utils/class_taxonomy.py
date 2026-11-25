"""Semantic labels for ObjectClassification.type values."""

from perception_msgs.msg import ObjectClassification

LABELS = {
    ObjectClassification.TRAFFIC_LIGHT: "traffic_light",
    ObjectClassification.TRAFFIC_LIGHT_UNKNOWN: "traffic_light_unknown",
    ObjectClassification.TRAFFIC_LIGHT_OFF: "traffic_light_off",
    ObjectClassification.TRAFFIC_LIGHT_RED: "traffic_light_red",
    ObjectClassification.TRAFFIC_LIGHT_YELLOW: "traffic_light_yellow",
    ObjectClassification.TRAFFIC_LIGHT_GREEN: "traffic_light_green",
    ObjectClassification.TRAFFIC_LIGHT_RED_YELLOW: "traffic_light_red_yellow",
    ObjectClassification.TRAFFIC_LIGHT_LEFT_RED: "traffic_light_left_red",
    ObjectClassification.TRAFFIC_LIGHT_LEFT_RED_STRAIGHT: "traffic_light_left_red_straight",
    ObjectClassification.TRAFFIC_LIGHT_RED_RIGHT: "traffic_light_red_right",
    ObjectClassification.TRAFFIC_LIGHT_RED_STRAIGHT: "traffic_light_red_straight",
    ObjectClassification.TRAFFIC_LIGHT_RED_UP_LEFT: "traffic_light_red_up_left",
    ObjectClassification.TRAFFIC_LIGHT_RED_RIGHT_STRAIGHT: "traffic_light_red_right_straight",
    ObjectClassification.TRAFFIC_LIGHT_RED_UP_RIGHT: "traffic_light_red_up_right",
    ObjectClassification.TRAFFIC_SIGN: "traffic_sign",
    ObjectClassification.TRAFFIC_SIGN_UNKNOWN: "traffic_sign_unknown",
    ObjectClassification.TRAFFIC_SIGN_STOP: "traffic_sign_stop",
    ObjectClassification.TRAFFIC_SIGN_GIVE_WAY: "traffic_sign_give_way",
    ObjectClassification.TRAFFIC_SIGN_PRIORITY_ROAD: "traffic_sign_priority_road",
    ObjectClassification.TRAFFIC_SIGN_PRIORITY_AT_NEXT_INTERSECTION: "traffic_sign_priority_at_next_intersection",
    ObjectClassification.TRAFFIC_SIGN_NO_ENTRY: "traffic_sign_no_entry",
    ObjectClassification.TRAFFIC_SIGN_NO_VEHICLES: "traffic_sign_no_vehicles",
    ObjectClassification.TRAFFIC_SIGN_NO_GOODS_VEHICLES_OVER_3_5_T: "traffic_sign_no_goods_vehicles_over_3_5_t",
    ObjectClassification.TRAFFIC_SIGN_NO_OVERTAKING: "traffic_sign_no_overtaking",
    ObjectClassification.TRAFFIC_SIGN_NO_OVERTAKING_GOODS_VEHICLES: "traffic_sign_no_overtaking_goods_vehicles",
    ObjectClassification.TRAFFIC_SIGN_END_NO_OVERTAKING: "traffic_sign_end_no_overtaking",
    ObjectClassification.TRAFFIC_SIGN_END_NO_OVERTAKING_GOODS_VEHICLES: "traffic_sign_end_no_overtaking_goods_vehicles",
    ObjectClassification.TRAFFIC_SIGN_END_ALL_SPEED_AND_OVERTAKING_RESTRICTIONS: "traffic_sign_end_all_speed_and_overtaking_restrictions",
    ObjectClassification.TRAFFIC_SIGN_SPEED_LIMIT_20_KMH: "traffic_sign_speed_limit_20_kmh",
    ObjectClassification.TRAFFIC_SIGN_SPEED_LIMIT_30_KMH: "traffic_sign_speed_limit_30_kmh",
    ObjectClassification.TRAFFIC_SIGN_SPEED_LIMIT_50_KMH: "traffic_sign_speed_limit_50_kmh",
    ObjectClassification.TRAFFIC_SIGN_SPEED_LIMIT_60_KMH: "traffic_sign_speed_limit_60_kmh",
    ObjectClassification.TRAFFIC_SIGN_SPEED_LIMIT_70_KMH: "traffic_sign_speed_limit_70_kmh",
    ObjectClassification.TRAFFIC_SIGN_SPEED_LIMIT_80_KMH: "traffic_sign_speed_limit_80_kmh",
    ObjectClassification.TRAFFIC_SIGN_SPEED_LIMIT_100_KMH: "traffic_sign_speed_limit_100_kmh",
    ObjectClassification.TRAFFIC_SIGN_SPEED_LIMIT_120_KMH: "traffic_sign_speed_limit_120_kmh",
    ObjectClassification.TRAFFIC_SIGN_END_SPEED_LIMIT_80_KMH: "traffic_sign_end_speed_limit_80_kmh",
    ObjectClassification.TRAFFIC_SIGN_DANGER: "traffic_sign_danger",
    ObjectClassification.TRAFFIC_SIGN_CURVE_LEFT: "traffic_sign_curve_left",
    ObjectClassification.TRAFFIC_SIGN_CURVE_RIGHT: "traffic_sign_curve_right",
    ObjectClassification.TRAFFIC_SIGN_DOUBLE_CURVE_FIRST_LEFT: "traffic_sign_double_curve_first_left",
    ObjectClassification.TRAFFIC_SIGN_UNEVEN_ROAD: "traffic_sign_uneven_road",
    ObjectClassification.TRAFFIC_SIGN_SLIPPERY_ROAD: "traffic_sign_slippery_road",
    ObjectClassification.TRAFFIC_SIGN_ROAD_NARROWS_RIGHT: "traffic_sign_road_narrows_right",
    ObjectClassification.TRAFFIC_SIGN_ROAD_WORKS: "traffic_sign_road_works",
    ObjectClassification.TRAFFIC_SIGN_TRAFFIC_SIGNALS: "traffic_sign_traffic_signals",
    ObjectClassification.TRAFFIC_SIGN_PEDESTRIANS: "traffic_sign_pedestrians",
    ObjectClassification.TRAFFIC_SIGN_CHILDREN: "traffic_sign_children",
    ObjectClassification.TRAFFIC_SIGN_CYCLISTS: "traffic_sign_cyclists",
    ObjectClassification.TRAFFIC_SIGN_ICE_OR_SNOW: "traffic_sign_ice_or_snow",
    ObjectClassification.TRAFFIC_SIGN_WILD_ANIMALS: "traffic_sign_wild_animals",
    ObjectClassification.TRAFFIC_SIGN_TURN_RIGHT: "traffic_sign_turn_right",
    ObjectClassification.TRAFFIC_SIGN_TURN_LEFT: "traffic_sign_turn_left",
    ObjectClassification.TRAFFIC_SIGN_STRAIGHT_AHEAD: "traffic_sign_straight_ahead",
    ObjectClassification.TRAFFIC_SIGN_STRAIGHT_OR_RIGHT: "traffic_sign_straight_or_right",
    ObjectClassification.TRAFFIC_SIGN_STRAIGHT_OR_LEFT: "traffic_sign_straight_or_left",
    ObjectClassification.TRAFFIC_SIGN_KEEP_RIGHT: "traffic_sign_keep_right",
    ObjectClassification.TRAFFIC_SIGN_KEEP_LEFT: "traffic_sign_keep_left",
    ObjectClassification.TRAFFIC_SIGN_ROUNDABOUT: "traffic_sign_roundabout",
}


def taxonomy_label(class_id):
    return LABELS.get(class_id, "")


def traffic_light_class(label):
    return LIGHT_LABELS.get(label, ObjectClassification.TRAFFIC_LIGHT_UNKNOWN)


LIGHT_LABELS = {
    "green": ObjectClassification.TRAFFIC_LIGHT_GREEN,
    "left,red": ObjectClassification.TRAFFIC_LIGHT_LEFT_RED,
    "left,red,straight": ObjectClassification.TRAFFIC_LIGHT_LEFT_RED_STRAIGHT,
    "red": ObjectClassification.TRAFFIC_LIGHT_RED,
    "red,right": ObjectClassification.TRAFFIC_LIGHT_RED_RIGHT,
    "red,straight": ObjectClassification.TRAFFIC_LIGHT_RED_STRAIGHT,
    "unknown": ObjectClassification.TRAFFIC_LIGHT_UNKNOWN,
    "yellow": ObjectClassification.TRAFFIC_LIGHT_YELLOW,
    "red,up_left": ObjectClassification.TRAFFIC_LIGHT_RED_UP_LEFT,
    "red,right,straight": ObjectClassification.TRAFFIC_LIGHT_RED_RIGHT_STRAIGHT,
    "red,up_right": ObjectClassification.TRAFFIC_LIGHT_RED_UP_RIGHT,
    "off": ObjectClassification.TRAFFIC_LIGHT_OFF,
    "red,yellow": ObjectClassification.TRAFFIC_LIGHT_RED_YELLOW,
}


def traffic_light_color(class_id):
    """Color group used by geometry and map association."""
    if class_id == ObjectClassification.TRAFFIC_LIGHT_GREEN:
        return 4
    if class_id == ObjectClassification.TRAFFIC_LIGHT_YELLOW:
        return 3
    if class_id == ObjectClassification.TRAFFIC_LIGHT_RED_YELLOW:
        return 2
    if class_id == ObjectClassification.TRAFFIC_LIGHT_OFF:
        return 255
    if (
        class_id == ObjectClassification.TRAFFIC_LIGHT_RED
        or ObjectClassification.TRAFFIC_LIGHT_LEFT_RED <= class_id <= ObjectClassification.TRAFFIC_LIGHT_RED_UP_RIGHT
    ):
        return 1
    return 0

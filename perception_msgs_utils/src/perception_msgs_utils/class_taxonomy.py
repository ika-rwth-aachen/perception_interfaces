"""Semantic labels for ObjectClassification.type values."""

LABELS = {
    101: "traffic_light",
    102: "traffic_light_unknown",
    103: "traffic_light_off",
    104: "traffic_light_red",
    105: "traffic_light_yellow",
    106: "traffic_light_green",
    107: "traffic_light_red_yellow",
    108: "traffic_light_left_red",
    109: "traffic_light_left_red_straight",
    110: "traffic_light_red_right",
    111: "traffic_light_red_straight",
    112: "traffic_light_red_up_left",
    113: "traffic_light_red_right_straight",
    114: "traffic_light_red_up_right",
    128: "traffic_sign",
    129: "traffic_sign_unknown",
    130: "traffic_sign_stop",
    131: "traffic_sign_give_way",
    132: "traffic_sign_priority_road",
    133: "traffic_sign_priority_at_next_intersection",
    134: "traffic_sign_no_entry",
    135: "traffic_sign_no_vehicles",
    136: "traffic_sign_no_goods_vehicles_over_3_5_t",
    137: "traffic_sign_no_overtaking",
    138: "traffic_sign_no_overtaking_goods_vehicles",
    139: "traffic_sign_end_no_overtaking",
    140: "traffic_sign_end_no_overtaking_goods_vehicles",
    141: "traffic_sign_end_all_speed_and_overtaking_restrictions",
    142: "traffic_sign_speed_limit_20_kmh",
    143: "traffic_sign_speed_limit_30_kmh",
    144: "traffic_sign_speed_limit_50_kmh",
    145: "traffic_sign_speed_limit_60_kmh",
    146: "traffic_sign_speed_limit_70_kmh",
    147: "traffic_sign_speed_limit_80_kmh",
    148: "traffic_sign_speed_limit_100_kmh",
    149: "traffic_sign_speed_limit_120_kmh",
    150: "traffic_sign_end_speed_limit_80_kmh",
    151: "traffic_sign_danger",
    152: "traffic_sign_curve_left",
    153: "traffic_sign_curve_right",
    154: "traffic_sign_double_curve_first_left",
    155: "traffic_sign_uneven_road",
    156: "traffic_sign_slippery_road",
    157: "traffic_sign_road_narrows_right",
    158: "traffic_sign_road_works",
    159: "traffic_sign_traffic_signals",
    160: "traffic_sign_pedestrians",
    161: "traffic_sign_children",
    162: "traffic_sign_cyclists",
    163: "traffic_sign_ice_or_snow",
    164: "traffic_sign_wild_animals",
    165: "traffic_sign_turn_right",
    166: "traffic_sign_turn_left",
    167: "traffic_sign_straight_ahead",
    168: "traffic_sign_straight_or_right",
    169: "traffic_sign_straight_or_left",
    170: "traffic_sign_keep_right",
    171: "traffic_sign_keep_left",
    172: "traffic_sign_roundabout",
}


def taxonomy_label(class_id):
    return LABELS.get(class_id, "")


def traffic_light_class(label):
    return LIGHT_LABELS.get(label, 102)


LIGHT_LABELS = {
    "green": 106,
    "left,red": 108,
    "left,red,straight": 109,
    "red": 104,
    "red,right": 110,
    "red,straight": 111,
    "unknown": 102,
    "yellow": 105,
    "red,up_left": 112,
    "red,right,straight": 113,
    "red,up_right": 114,
    "off": 103,
    "red,yellow": 107,
}


def traffic_light_color(class_id):
    """Color group used by geometry and map association."""
    if class_id == 106:
        return 4
    if class_id == 105:
        return 3
    if class_id == 107:
        return 2
    if class_id == 103:
        return 255
    if class_id == 104 or 108 <= class_id <= 114:
        return 1
    return 0

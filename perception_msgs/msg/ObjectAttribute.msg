# A named categorical property, separate from an object's primary classification.
# Examples: "traffic_sign.type", "vehicle.indicator.left", "vehicle.indicator.right".
# Nonempty names and value vocabularies are agreed by producer and consumer.
string name

# Alternative values for this one property, using class_id as the value identifier.
# Each value identifier appears at most once. Scores lie in [0, 1], highest first.
# An empty list means unknown.
# Alternatives are not required to form a normalized or exhaustive distribution.
# Omitted values must not be interpreted as negative observations.
vision_msgs/ObjectHypothesis[] hypotheses

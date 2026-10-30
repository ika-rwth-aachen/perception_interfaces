# Categorical properties observed for a detection in the source image.
# Preserve the primary object classification in detection.results.
# detection.id identifies the tracked object; empty means untracked.
vision_msgs/Detection2D detection

# At most one entry per attribute name. Missing attributes mean unobserved.
# Values describe the observation at detection.header.stamp, not future state.
ObjectAttribute[] attributes

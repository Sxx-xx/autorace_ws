# Normalized traffic sign detection, aggregated from the per-sign detectors.

std_msgs/Header header

# One of: intersection, left, right, construction, parking, stop, tunnel
string name
# Number of consecutive frames the sign has been seen.
uint16 streak

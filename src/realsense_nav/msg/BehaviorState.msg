std_msgs/String state
# Possible states: "GO_TO_CONE", "TURN_LEFT", "TURN_RIGHT", "WAIT", "STOP"

# Parameters for the state
float64 target_distance  # For GO_TO_CONE: stop distance
float64 turn_angle       # For TURN_LEFT/TURN_RIGHT: degrees to turn
float64 wait_duration    # For WAIT: seconds to wait

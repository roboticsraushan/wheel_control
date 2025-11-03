# Junction detection result
string junction_id           # Detected junction ID
float32 confidence           # Recognition confidence [0.0, 1.0]
geometry_msgs/Point centroid # Image centroid of matched features
int32 num_matches            # Number of feature matches
float64 timestamp            # Detection timestamp
bool is_expected             # True if this matches the expected junction

# Junction information message
string id                    # Unique junction ID (e.g., "J001")
geometry_msgs/Pose pose      # Junction pose in world frame
string image_path            # Path to junction image
string[] visible_junctions   # IDs of visible junctions from this junction
string[] associated_nodes    # Node IDs visible from this junction
float64 timestamp            # When junction was recorded

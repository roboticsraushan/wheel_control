# Sequence of junctions to navigate through
string task_id               # Associated navigation task ID
string[] junction_ids        # Ordered list of junction IDs to visit
geometry_msgs/Pose[] junction_poses  # Corresponding poses
float64 total_distance       # Estimated total path distance (meters)
int32 num_junctions          # Number of junctions in path

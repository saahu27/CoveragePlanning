# One executable coverage segment (centerline polyline).
uint64 id
geometry_msgs/Pose[] centerline
uint32 lane_index
uint8 outcome   # 0=pending, 1=completed, 2=skipped
bool blocked

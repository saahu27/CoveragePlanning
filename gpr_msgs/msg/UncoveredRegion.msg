# Spatially grouped uncovered swath on one lane.
uint8 reason    # 0=pending, 1=blocked, 2=skipped
uint32 lane_index
uint64[] segment_ids
float64 swath_length_m
float64 swath_area_m2
float64 bbox_x_min
float64 bbox_y_min
float64 bbox_x_max
float64 bbox_y_max

#ifndef GPR_METRICS__UNCOVERED_REGION_HPP_
#define GPR_METRICS__UNCOVERED_REGION_HPP_

#include <cstdint>
#include <vector>

#include "gpr_common/coverage_types.hpp"

namespace gpr_metrics
{

/// @brief Why a group of segments remains uncovered.
enum class UncoveredReason : uint8_t
{
  Pending = 0,
  Blocked = 1,
  Skipped = 2,
};

/// @brief Spatially grouped uncovered swath on one lane.
struct UncoveredRegion
{
  UncoveredReason reason{UncoveredReason::Pending};
  uint32_t lane_index{0};
  std::vector<gpr_common::SegmentId> segment_ids;
  double swath_length_m{0.0};
  double swath_area_m2{0.0};
  double bbox_x_min{0.0};
  double bbox_y_min{0.0};
  double bbox_x_max{0.0};
  double bbox_y_max{0.0};
};

}  // namespace gpr_metrics

#endif  // GPR_METRICS__UNCOVERED_REGION_HPP_

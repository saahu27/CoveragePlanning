#ifndef GPR_METRICS__COVERAGE_BASELINE_HPP_
#define GPR_METRICS__COVERAGE_BASELINE_HPP_

#include <cstdint>
#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/types.hpp"

namespace gpr_metrics
{

/// @brief Immutable snapshot of the initial boustrophedon decomposition.
struct CoverageBaseline
{
  uint64_t mission_id{0};
  gpr_common::ScanArea scan_area{};
  double coverage_inset{0.0};
  double swath_width_m{0.5};
  std::vector<gpr_common::CoverageSegment> segments;
  double planned_swath_length_m{0.0};
  double planned_swath_area_m2{0.0};
  uint32_t segment_count{0};

  /// @brief Build a baseline from the master segment set at mission start.
  [[nodiscard]] static CoverageBaseline from_segments(
    std::vector<gpr_common::CoverageSegment> master,
    gpr_common::ScanArea scan_area,
    double coverage_inset,
    double swath_width_m,
    uint64_t mission_id = 1);
};

}  // namespace gpr_metrics

#endif  // GPR_METRICS__COVERAGE_BASELINE_HPP_

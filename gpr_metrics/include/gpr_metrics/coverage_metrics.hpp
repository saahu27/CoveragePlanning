#ifndef GPR_METRICS__COVERAGE_METRICS_HPP_
#define GPR_METRICS__COVERAGE_METRICS_HPP_

#include <cstdint>

namespace gpr_metrics
{

/// @brief Aggregated length- and count-based coverage KPIs.
struct CoverageMetrics
{
  double baseline_swath_length_m{0.0};
  double completed_swath_length_m{0.0};
  double blocked_swath_length_m{0.0};
  double skipped_swath_length_m{0.0};
  double pending_swath_length_m{0.0};

  double baseline_swath_area_m2{0.0};
  double completed_swath_area_m2{0.0};

  double partial_swath_length_m{0.0};
  double partial_pct{0.0};
  double swath_coverage_pct{0.0};

  double coverage_pct{0.0};
  double blocked_pct{0.0};
  double skipped_pct{0.0};
  double remaining_pct{0.0};
  double plan_retention_pct{0.0};

  double mean_lateral_error_m{0.0};
  double max_lateral_error_m{0.0};

  uint32_t segments_total{0};
  uint32_t segments_completed{0};
  uint32_t segments_partial{0};
  uint32_t segments_blocked{0};
  uint32_t segments_skipped{0};
  uint32_t segments_pending{0};
};

}  // namespace gpr_metrics

#endif  // GPR_METRICS__COVERAGE_METRICS_HPP_

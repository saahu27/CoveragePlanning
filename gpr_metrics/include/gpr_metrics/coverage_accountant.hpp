#ifndef GPR_METRICS__COVERAGE_ACCOUNTANT_HPP_
#define GPR_METRICS__COVERAGE_ACCOUNTANT_HPP_

#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_metrics/coverage_baseline.hpp"
#include "gpr_metrics/coverage_metrics.hpp"
#include "gpr_metrics/metrics_config.hpp"
#include "gpr_metrics/uncovered_region.hpp"

namespace gpr_metrics
{

/// @brief Pure functions that compute coverage KPIs from baseline + live catalog.
class CoverageAccountant
{
public:
  [[nodiscard]] static CoverageMetrics compute(
    const CoverageBaseline & baseline,
    const std::vector<gpr_common::CoverageSegment> & current,
    const MetricsConfig & metrics_config = {});

  [[nodiscard]] static std::vector<UncoveredRegion> group_uncovered(
    const std::vector<gpr_common::CoverageSegment> & current,
    double swath_width_m);

private:
  [[nodiscard]] static UncoveredReason reason_for(
    const gpr_common::CoverageSegment & seg) noexcept;
};

}  // namespace gpr_metrics

#endif  // GPR_METRICS__COVERAGE_ACCOUNTANT_HPP_

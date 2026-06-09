#ifndef GPR_METRICS__COVERAGE_REPORTER_HPP_
#define GPR_METRICS__COVERAGE_REPORTER_HPP_

#include <optional>
#include <string>
#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/types.hpp"
#include "gpr_metrics/coverage_baseline.hpp"
#include "gpr_metrics/coverage_metrics.hpp"
#include "gpr_metrics/metrics_config.hpp"
#include "gpr_metrics/uncovered_region.hpp"

namespace gpr_metrics
{

/// @brief Holds the frozen baseline and computes live coverage reports.
class CoverageReporter
{
public:
  void configure(
    gpr_common::ScanArea scan_area, double coverage_inset, double swath_width_m,
    MetricsConfig metrics_config = {});

  /// @brief Freeze the master segment set as the mission baseline.
  void record_baseline(std::vector<gpr_common::CoverageSegment> master);

  [[nodiscard]] bool has_baseline() const noexcept {return baseline_.has_value();}
  [[nodiscard]] const CoverageBaseline & baseline() const {return *baseline_;}

  [[nodiscard]] CoverageMetrics compute_metrics(
    const std::vector<gpr_common::CoverageSegment> & current) const;

  [[nodiscard]] std::vector<UncoveredRegion> compute_uncovered(
    const std::vector<gpr_common::CoverageSegment> & current) const;

  /// @brief Human-readable multi-line summary for logs and file export.
  [[nodiscard]] std::string format_text_report(
    const CoverageMetrics & metrics,
    const std::vector<UncoveredRegion> & regions) const;

  /// @brief Write a text report to @p path; returns false on I/O error.
  [[nodiscard]] bool export_text_report(
    const std::string & path,
    const CoverageMetrics & metrics,
    const std::vector<UncoveredRegion> & regions) const;

  /// @brief Build `export_dir/coverage_report_<mission_id>_<stamp>.txt`, creating @p export_dir.
  [[nodiscard]] static std::string make_export_path(
    const std::string & export_dir, uint64_t mission_id, int64_t sec, uint32_t nanosec);

private:
  std::optional<CoverageBaseline> baseline_;
  gpr_common::ScanArea scan_area_{};
  double coverage_inset_{0.0};
  double swath_width_m_{0.5};
  MetricsConfig metrics_config_{};
  uint64_t next_mission_id_{1};
};

}  // namespace gpr_metrics

#endif  // GPR_METRICS__COVERAGE_REPORTER_HPP_

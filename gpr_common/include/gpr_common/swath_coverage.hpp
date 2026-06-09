#ifndef GPR_COMMON__SWATH_COVERAGE_HPP_
#define GPR_COMMON__SWATH_COVERAGE_HPP_

#include <utility>
#include <vector>

#include "gpr_common/types.hpp"

namespace gpr_common
{

/// @brief Which phase of job execution is appending to the driven trace.
enum class ExecutionTracePhase : uint8_t
{
  Transit = 0,
  Coverage = 1,
};

/// @brief Criteria for arc-length swath traversal coverage.
struct SwathCoverageConfig
{
  double lateral_max_m{0.22};
  double heading_max_rad{0.61};
  double sample_half_width_m{0.025};
  double min_complete_fraction{0.85};
  double min_partial_fraction{0.35};
  bool partial_enabled{true};
};

/// @brief Result of projecting a point onto a polyline by arc length.
struct PolylineProjection
{
  double arc_s_m{0.0};
  double lateral_m{0.0};
  double heading_err_rad{0.0};
  bool valid{false};
};

/// @brief Arc-length coverage along a swath centerline.
struct SwathCoverageResult
{
  double total_length_m{0.0};
  double covered_length_m{0.0};
  double covered_fraction{0.0};
  double mean_lateral_error_m{0.0};
  double max_lateral_error_m{0.0};
  std::vector<std::pair<double, double>> covered_intervals_m;
  std::size_t on_swath_sample_count{0U};
};

[[nodiscard]] double polyline_arc_length(const Polyline & line) noexcept;

[[nodiscard]] PolylineProjection project_point_onto_polyline(
  const Pose2D & point, const Polyline & line) noexcept;

[[nodiscard]] std::vector<std::pair<double, double>> merge_intervals(
  std::vector<std::pair<double, double>> intervals) noexcept;

[[nodiscard]] double intervals_union_length(
  const std::vector<std::pair<double, double>> & intervals) noexcept;

[[nodiscard]] std::vector<std::pair<double, double>> complement_intervals(
  const std::vector<std::pair<double, double>> & covered,
  double total_length_m) noexcept;

[[nodiscard]] Polyline sub_polyline_by_arc_length(
  const Polyline & line, double s0_m, double s1_m) noexcept;

[[nodiscard]] Pose2D point_at_arc_on_polyline(
  const Polyline & line, double arc_s_m) noexcept;

/// @brief Project covered arc intervals from @p src_line onto @p dst_line.
[[nodiscard]] std::vector<std::pair<double, double>> map_covered_intervals_between_polylines(
  const std::vector<std::pair<double, double>> & src_intervals_m,
  const Polyline & src_line,
  const Polyline & dst_line) noexcept;

[[nodiscard]] SwathCoverageResult compute_swath_coverage(
  const Polyline & centerline,
  const Polyline & driven,
  const SwathCoverageConfig & config) noexcept;

[[nodiscard]] double interval_overlap_length(
  const std::vector<std::pair<double, double>> & a,
  const std::vector<std::pair<double, double>> & b) noexcept;

/// @brief Map pass-level covered intervals onto a constituent segment centerline.
[[nodiscard]] SwathCoverageResult coverage_for_segment_from_pass(
  const Polyline & segment_centerline,
  const Polyline & pass_centerline,
  const SwathCoverageResult & pass_measured) noexcept;

}  // namespace gpr_common

#endif  // GPR_COMMON__SWATH_COVERAGE_HPP_

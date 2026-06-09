#ifndef GPR_COMMON__COVERAGE_TYPES_HPP_
#define GPR_COMMON__COVERAGE_TYPES_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "gpr_common/swath_coverage.hpp"
#include "gpr_common/types.hpp"

namespace gpr_common
{

/// @brief Stable identifier for a coverage segment.
using SegmentId = uint64_t;

/// @brief Persistent execution outcome for a coverage segment.
enum class SegmentOutcome : uint8_t
{
  Pending = 0,             ///< Not yet attempted.
  Completed = 1,           ///< Successfully driven; counts toward coverage.
  Skipped = 2,             ///< Abandoned by recovery; does NOT count as covered.
  PartiallyCompleted = 3,  ///< Some arc length covered; tail may remain schedulable.
};

/// @brief Provenance of a catalog segment (baseline master, split child, or OARP rank).
enum class SegmentSource : uint8_t
{
  Baseline = 0,  ///< Original boustrophedon master segment.
  Split = 1,     ///< Child piece after partial invalidation of a baseline segment.
  OarpRank = 2,  ///< Ephemeral rank replanned on the live free-space mask.
};

/// @brief One coverable lane piece plus its persistent coverage state.
struct CoverageSegment
{
  SegmentId id{0};            ///< Stable identity (see make_segment_id).
  Polyline centerline;        ///< Path the robot drives to cover this segment.
  std::optional<uint32_t> lane_index;  ///< Owning boustrophedon lane, if known.
  SegmentOutcome outcome{SegmentOutcome::Pending};
  bool blocked{false};        ///< Obstacle-blocked; only meaningful when Pending.
  bool attempted{false};      ///< True once transit or coverage has started for this piece.
  SegmentSource source{SegmentSource::Baseline};
  /// @brief Master baseline id for split/OARP rollup (unset for baseline masters).
  std::optional<SegmentId> baseline_id;
  /// @brief Covered arc-length intervals [s0,s1] in meters along @p centerline (forward).
  std::vector<std::pair<double, double>> covered_intervals_m;
  /// @brief Mean lateral error (m) from last coverage evaluation on this segment.
  double last_mean_lateral_error_m{0.0};
  /// @brief Max lateral error (m) from last coverage evaluation on this segment.
  double last_max_lateral_error_m{0.0};
};

/// @brief Baseline ledger id used for coverage metrics rollup.
[[nodiscard]] inline SegmentId effective_baseline_id(const CoverageSegment & seg) noexcept
{
  return seg.baseline_id.value_or(seg.id);
}

/// @brief True when the segment is open coverage work (pending and not blocked).
[[nodiscard]] inline bool is_segment_open(const CoverageSegment & seg) noexcept
{
  if (seg.blocked || seg.centerline.size() < 2U) {
    return false;
  }
  return seg.outcome == SegmentOutcome::Pending ||
         seg.outcome == SegmentOutcome::PartiallyCompleted;
}

/// @brief True when a blocked-but-unattempted segment may be probed via transit.
[[nodiscard]] inline bool is_segment_probe(const CoverageSegment & seg) noexcept
{
  return seg.outcome == SegmentOutcome::Pending && seg.blocked && !seg.attempted &&
         seg.centerline.size() >= 2U;
}

/// @brief True when the segment has recorded coverage progress but is not finished.
[[nodiscard]] inline bool has_coverage_progress(const CoverageSegment & seg) noexcept
{
  return !seg.covered_intervals_m.empty();
}

/// @brief True when the segment may enter the coverage schedule queue.
[[nodiscard]] inline bool is_segment_schedulable(
  const CoverageSegment & seg, bool schedule_probes) noexcept
{
  if (seg.centerline.size() < 2U) {
    return false;
  }
  // PartiallyCompleted tails are tracked for viz/metrics but not re-queued yet.
  if (seg.outcome == SegmentOutcome::PartiallyCompleted) {
    return false;
  }
  // Never re-queue segments that already have coverage intervals stored.
  if (seg.outcome == SegmentOutcome::Pending && has_coverage_progress(seg)) {
    return false;
  }
  if (seg.outcome == SegmentOutcome::Pending && !seg.blocked) {
    return true;
  }
  return schedule_probes && is_segment_probe(seg);
}

/// @brief True when the segment counts toward coverage metrics.
[[nodiscard]] inline bool is_segment_covered(const CoverageSegment & seg) noexcept
{
  return seg.outcome == SegmentOutcome::Completed ||
         seg.outcome == SegmentOutcome::PartiallyCompleted;
}

/// @brief Remaining uncovered arc length (m) along @p seg centerline.
[[nodiscard]] double uncovered_arc_length_m(const CoverageSegment & seg) noexcept;

/// @brief Centerline polyline for scheduling/driving (uncovered tail when partial).
[[nodiscard]] Polyline schedulable_centerline(
  const CoverageSegment & segment, DriveDirection direction);

/// @brief A segment to drive in a chosen direction (one unit of work).
struct CoverageJob
{
  SegmentId segment_id{0};
  DriveDirection direction{DriveDirection::Forward};
  /// @brief All catalog segments covered when this pass completes (merged lane ranks).
  std::vector<SegmentId> covers{};

  [[nodiscard]] bool operator==(const CoverageJob & other) const noexcept
  {
    return segment_id == other.segment_id && direction == other.direction &&
           covers == other.covers;
  }
};

/// @brief Centerline of a segment ordered for the given drive direction.
[[nodiscard]] Polyline job_polyline(
  const CoverageSegment & segment, DriveDirection direction);

/// @brief Pose where the robot enters the segment for the given direction.
[[nodiscard]] Pose2D job_entry_pose(
  const CoverageSegment & segment, DriveDirection direction);

/// @brief Pose where the robot exits the segment for the given direction.
[[nodiscard]] Pose2D job_exit_pose(
  const CoverageSegment & segment, DriveDirection direction);

/// @brief Sub-polyline from the robot forward along @p line (no backward retracing).
[[nodiscard]] Polyline trim_polyline_ahead(
  const Polyline & line, const Pose2D & robot, double prune_distance_m = 0.15);

/// @brief Fraction of @p segment samples within @p tol_m of @p driven (0..1).
[[nodiscard]] double polyline_covered_fraction(
  const Polyline & segment, const Polyline & driven, double tol_m = 0.25);

/// @brief Geometry-derived id (legacy/path-invalidator use); not churn-free.
[[nodiscard]] SegmentId compute_segment_id(
  const Polyline & centerline, std::optional<uint32_t> lane_index,
  double quantize_m = 0.05);

// Deterministic segment identity based purely on the lane index and the
// sub-segment ordinal within that lane. Unlike compute_segment_id, this is
// independent of geometry and the live grid, so an ID never changes as the
// occupancy map evolves. This is what makes persistent coverage tracking work.
/// @brief Stable id from (lane, ordinal); used by the master segment set.
[[nodiscard]] SegmentId make_segment_id(uint32_t lane_index, uint32_t sub_ordinal);

/// @brief Id for a child piece split from a baseline segment.
[[nodiscard]] SegmentId make_split_segment_id(
  SegmentId baseline_id, uint32_t split_ordinal);

/// @brief Id for an OARP-lite replanned rank (generation avoids id reuse).
[[nodiscard]] SegmentId make_oarp_rank_id(
  uint32_t replan_generation, uint32_t rank_index);

/// @brief Id for a merged lane pass (contiguous schedulable pieces on one lane).
[[nodiscard]] SegmentId make_pass_job_id(uint32_t lane_index, uint32_t pass_ordinal);

/// @brief Build forward+reverse jobs for every schedulable segment.
[[nodiscard]] std::vector<CoverageJob> schedulable_jobs_from_segments(
  const std::vector<CoverageSegment> & segments, bool schedule_probes);

/// @brief Build forward+reverse jobs for every open (uncovered) segment.
[[nodiscard]] std::vector<CoverageJob> open_jobs_from_segments(
  const std::vector<CoverageSegment> & segments);

}  // namespace gpr_common

#endif  // GPR_COMMON__COVERAGE_TYPES_HPP_

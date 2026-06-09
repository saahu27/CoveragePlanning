#ifndef GPR_PLANNING__SEGMENT_CATALOG_HPP_
#define GPR_PLANNING__SEGMENT_CATALOG_HPP_

#include <cstdint>
#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/swath_coverage.hpp"
#include "gpr_common/grid_map.hpp"
#include "gpr_planning/astar_grid_planner.hpp"
#include "gpr_planning/oarp_lite_planner.hpp"
#include "gpr_planning/path_invalidator.hpp"

namespace gpr_planning
{

/// @brief Catalog policy for scheduling and splitting.
struct SegmentCatalogConfig
{
  bool schedule_blocked_probes{true};  ///< Schedule blocked-but-unattempted segments.
  double min_split_length_m{0.25};     ///< Drop free fragments shorter than this.
  /// @brief Treat one free fragment as full-length if within this of total (m).
  double split_length_tolerance_m{0.001};
  /// @brief Lateral tolerance (m) for legacy overlap-based completion.
  double mark_completed_overlap_tol_m{0.25};
  /// @brief Minimum covered fraction for legacy overlap-based completion.
  double mark_completed_min_overlap_fraction{0.55};
};

// Persistent catalog of coverage segments. Baseline masters may split into
// children on partial invalidation; OARP-lite ranks are appended when baseline
// work is exhausted on a lane.
class SegmentCatalog
{
public:
  explicit SegmentCatalog(SegmentCatalogConfig config = {});

  /// @brief Monotonic counter bumped whenever the schedulable set changes.
  [[nodiscard]] std::uint64_t revision() const noexcept {return revision_;}

  /// @brief Install the master segment set once (all Pending); resets state.
  void initialize(std::vector<gpr_common::CoverageSegment> master);

  /// @brief Split on partial block, refresh blocked flags, optionally add OARP ranks.
  void update_blocked(
    const gpr_common::GridMap & grid, const PathInvalidatorConfig & config);

  /// @brief Append OARP-lite ranks when baseline/probe work is exhausted.
  void refresh_oarp_ranks(
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & inv_config,
    const OarpLitePlanner & oarp_planner,
    const std::optional<gpr_common::Pose2D> & robot_pose = std::nullopt,
    const AStarGridPlanner * astar = nullptr);

  /// @brief Mark a segment permanently covered (excluded from scheduling).
  void mark_completed(gpr_common::SegmentId id);
  /// @brief Mark job covers and split children along @p driven_path as completed.
  void mark_completed_for_job(
    const std::vector<gpr_common::SegmentId> & cover_ids,
    const gpr_common::Polyline & driven_path,
    double overlap_tol_m = 0.25,
    double min_overlap_fraction = 0.55);

  /// @brief Apply arc-length swath coverage from @p coverage_trace to job covers.
  void apply_swath_coverage_for_job(
    const std::vector<gpr_common::SegmentId> & cover_ids,
    const gpr_common::Polyline & coverage_trace,
    const gpr_common::SwathCoverageConfig & config,
    const gpr_common::Polyline * pass_centerline = nullptr,
    gpr_common::DriveDirection drive_direction = gpr_common::DriveDirection::Forward,
    const std::optional<uint32_t> pass_lane_index = std::nullopt,
    const gpr_common::GridMap * grid = nullptr,
    const PathInvalidatorConfig * inv_config = nullptr);
  /// @brief Mark a job as skipped (excluded from scheduling, not counted covered).
  void mark_job_skipped(gpr_common::SegmentId id);
  /// @brief Mark pending segments matching @p cover_ids (and split children) blocked.
  void mark_blocked_for_job(const std::vector<gpr_common::SegmentId> & cover_ids);
  /// @brief Mark a pending segment blocked (e.g. Nav2 could not reach it).
  void mark_blocked(gpr_common::SegmentId id);
  /// @brief Record that transit or coverage has started for matching cover ids.
  void mark_attempted_for_job(const std::vector<gpr_common::SegmentId> & cover_ids);
  /// @brief Record that transit or coverage has started for @p id.
  void mark_attempted(gpr_common::SegmentId id);

  [[nodiscard]] const std::vector<gpr_common::CoverageSegment> & segments() const noexcept
  {
    return segments_;
  }

  /// @brief Segments still to cover (pending and not blocked).
  [[nodiscard]] std::vector<gpr_common::CoverageSegment> open_segments() const;

  /// @brief Segments eligible for ATSP (open plus optional blocked probes).
  [[nodiscard]] std::vector<gpr_common::CoverageSegment> schedulable_segments() const;

  /// @brief True if any schedulable work or OARP could remain.
  [[nodiscard]] bool has_schedulable_work(
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & inv_config,
    const OarpLitePlanner & oarp_planner) const;

  [[nodiscard]] bool catalog_changed_since(std::uint64_t rev) const noexcept
  {
    return revision_ != rev;
  }

  [[nodiscard]] const gpr_common::CoverageSegment * find(
    gpr_common::SegmentId id) const;

  [[nodiscard]] const SegmentCatalogConfig & config() const noexcept {return config_;}

private:
  [[nodiscard]] bool is_open(const gpr_common::CoverageSegment & seg) const noexcept;

  void purge_pending_oarp_ranks();
  void apply_split_update(
    gpr_common::CoverageSegment seg,
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & config,
    std::vector<gpr_common::CoverageSegment> & updated,
    bool & schedulable_changed);

  void update_partial_segment(
    gpr_common::CoverageSegment & seg,
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & config,
    bool & schedulable_changed);

  SegmentCatalogConfig config_;
  std::vector<gpr_common::CoverageSegment> segments_;
  std::uint64_t revision_{0};
  std::uint32_t split_counter_{0};
  std::uint32_t oarp_generation_{0};
};

/// @brief True when @p job still has at least one pending, unblocked catalog piece.
[[nodiscard]] bool job_still_actionable(
  const gpr_common::CoverageJob & job, const SegmentCatalog & catalog);

}  // namespace gpr_planning

#endif  // GPR_PLANNING__SEGMENT_CATALOG_HPP_

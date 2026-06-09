#ifndef GPR_PLANNING__SEQUENCE_SOLVER_UTILS_HPP_
#define GPR_PLANNING__SEQUENCE_SOLVER_UTILS_HPP_

#include <optional>
#include <unordered_map>
#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/types.hpp"

namespace gpr_planning
{

/// @brief Tunables passed to ATSP / heuristic sequencers.
struct SequenceSolverOptions
{
  /// @brief When set, each id is scheduled in exactly one drive direction.
  const std::unordered_map<gpr_common::SegmentId, gpr_common::DriveDirection> *
  fixed_directions{nullptr};
  /// @brief When set, route ends at this pose (open tour: robot -> jobs -> end).
  std::optional<gpr_common::Pose2D> route_end_pose{};
};

/// @brief A segment paired with a direction and its entry/exit poses.
struct DirectedJob
{
  gpr_common::CoverageJob job;
  gpr_common::SegmentId segment_id;
  gpr_common::Pose2D entry;
  gpr_common::Pose2D exit;
};

/// @brief Expand segments into directed ATSP nodes (one or two per segment).
[[nodiscard]] std::vector<DirectedJob> build_directed_jobs(
  const std::vector<gpr_common::CoverageSegment> & segments,
  const std::unordered_map<gpr_common::SegmentId, gpr_common::DriveDirection> *
  fixed_directions);

}  // namespace gpr_planning

#endif  // GPR_PLANNING__SEQUENCE_SOLVER_UTILS_HPP_

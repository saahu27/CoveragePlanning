#ifndef GPR_PLANNING__BOUSTROPHEDON_SEQUENCER_HPP_
#define GPR_PLANNING__BOUSTROPHEDON_SEQUENCER_HPP_

#include <vector>

#include <optional>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/types.hpp"
#include "gpr_planning/astar_grid_planner.hpp"
#include "gpr_planning/boustrophedon_planner.hpp"

namespace gpr_planning
{

/// @brief Tunables for lane-coherent boustrophedon scheduling.
struct BoustrophedonSequencerConfig
{
  /// @brief Merge adjacent schedulable pieces on a lane when endpoints are closer (m).
  double merge_gap_m{0.2};
  /// @brief Discard merged passes shorter than this (m).
  double merge_endpoint_epsilon_m{0.001};
};

/// @brief One continuous drive along part or all of a scan lane.
struct CoveragePass
{
  gpr_common::CoverageJob job;
  gpr_common::Polyline centerline;
  std::vector<gpr_common::SegmentId> segment_ids;
  uint32_t lane_index{0};
};

/// @brief Orders work lane-by-lane from the outer boundary in boustrophedon order.
class BoustrophedonSequencer
{
public:
  explicit BoustrophedonSequencer(BoustrophedonSequencerConfig config = {});

  /// @brief Build merged lane passes; when @p robot_pose is set, order by nearest
  ///        entry and pick drive direction toward the closest end of each pass.
  [[nodiscard]] std::vector<CoveragePass> plan(
    const std::vector<gpr_common::CoverageSegment> & schedulable,
    const BoustrophedonConfig & boustrophedon_config,
    const std::optional<gpr_common::Pose2D> & robot_pose = std::nullopt,
    const gpr_common::GridMap * grid = nullptr,
    const AStarGridPlanner * astar = nullptr) const;

  [[nodiscard]] const BoustrophedonSequencerConfig & config() const noexcept {return config_;}

private:
  [[nodiscard]] gpr_common::DriveDirection lane_drive_direction(
    uint32_t lane_index, gpr_common::ScanDirection scan_direction) const;

  [[nodiscard]] double segment_lane_key(
    const gpr_common::CoverageSegment & seg,
    gpr_common::ScanDirection scan_direction,
    gpr_common::DriveDirection direction) const;

  [[nodiscard]] gpr_common::Polyline merge_polylines(
    const gpr_common::Polyline & a, const gpr_common::Polyline & b) const;

  [[nodiscard]] gpr_common::Polyline oriented_centerline(
    const gpr_common::CoverageSegment & seg,
    gpr_common::DriveDirection direction) const;

  [[nodiscard]] static gpr_common::DriveDirection prefer_drive_direction(
    const gpr_common::Polyline & centerline, const gpr_common::Pose2D & robot) noexcept;

  BoustrophedonSequencerConfig config_;
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__BOUSTROPHEDON_SEQUENCER_HPP_

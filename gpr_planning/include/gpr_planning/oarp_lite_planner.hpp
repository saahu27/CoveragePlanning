#ifndef GPR_PLANNING__OARP_LITE_PLANNER_HPP_
#define GPR_PLANNING__OARP_LITE_PLANNER_HPP_

#include <vector>
#include <cstdint>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/grid_map.hpp"
#include "gpr_planning/boustrophedon_planner.hpp"
#include "gpr_planning/path_invalidator.hpp"

namespace gpr_planning
{

/// @brief Tunables for OARP-lite rank generation on the live free-space mask.
struct OarpLiteConfig
{
  bool enabled{true};
  double min_rank_length_m{0.35};  ///< Ignore free fragments shorter than this.
  /// @brief Max OARP rank batches per mission (prevents infinite replan at dead ends).
  std::uint32_t max_replan_generations{2U};
  /// @brief Proximity (m) to a completed segment centerline that disqualifies a new rank.
  double overlap_dist_m{0.2};
};

/// @brief Replan coverage ranks on free cells inside the scan area (OARP-lite).
class OarpLitePlanner
{
public:
  OarpLitePlanner(BoustrophedonConfig boustrophedon_config, OarpLiteConfig oarp_config);

  /// @brief True if any new drivable rank could be generated on @p grid.
  [[nodiscard]] bool has_uncovered_ranks(
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & inv_config,
    const std::vector<gpr_common::CoverageSegment> & catalog) const;

  /// @brief Build schedulable rank segments clipped to the current free mask.
  [[nodiscard]] std::vector<gpr_common::CoverageSegment> generate_ranks(
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & inv_config,
    const std::vector<gpr_common::CoverageSegment> & catalog,
    std::uint32_t replan_generation) const;

  [[nodiscard]] const OarpLiteConfig & config() const noexcept {return oarp_config_;}

private:
  [[nodiscard]] bool overlaps_completed(
    const gpr_common::Polyline & candidate,
    const std::vector<gpr_common::CoverageSegment> & catalog,
    std::optional<uint32_t> lane_index) const;

  [[nodiscard]] bool has_open_catalog_on_lane(
    uint32_t lane_index,
    const std::vector<gpr_common::CoverageSegment> & catalog) const;

  BoustrophedonPlanner planner_;
  OarpLiteConfig oarp_config_;
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__OARP_LITE_PLANNER_HPP_

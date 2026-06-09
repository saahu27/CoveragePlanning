#ifndef GPR_PLANNING__ASTAR_GRID_PLANNER_HPP_
#define GPR_PLANNING__ASTAR_GRID_PLANNER_HPP_

#include <limits>
#include <optional>
#include <vector>

#include "gpr_common/grid_map.hpp"
#include "gpr_common/types.hpp"

namespace gpr_planning
{

/// @brief Tunables for the A* transit planner / cost estimator.
struct AStarConfig
{
  int obstacle_cost_threshold{50};   ///< Cost at/above which a cell blocks.
  bool treat_unknown_as_free{true};  ///< Whether unknown cells are passable.
  double footprint_radius{0.12};     ///< Robot radius used for clearance (m).
  /// @brief Grid resolution assumed when @p grid.resolution is zero.
  double fallback_grid_resolution_m{0.05};
  /// @brief Edge sample step uses max(spacing, factor × resolution) on path checks.
  double collision_sample_resolution_factor{0.5};
  // Multiplier applied to the straight-line distance when the direct transit is
  // blocked, so the sequencer prefers lanes reachable in open space.
  double blocked_transit_penalty{4.0};
};

/// @brief A* grid planner used for transit costs and transit path planning.
class AStarGridPlanner
{
public:
  explicit AStarGridPlanner(AStarConfig config);

  /// @brief Cheap geometric transit-cost estimate used for ATSP sequencing.
  [[nodiscard]] double transit_cost(
    const gpr_common::GridMap & grid,
    const gpr_common::Pose2D & from,
    const gpr_common::Pose2D & to) const;

  /// @brief Obstacle-aware path between two poses, or nullopt if unreachable.
  [[nodiscard]] std::optional<gpr_common::Polyline> plan_path(
    const gpr_common::GridMap & grid,
    const gpr_common::Pose2D & from,
    const gpr_common::Pose2D & to) const;

private:
  AStarConfig config_;
  static constexpr double kInf = std::numeric_limits<double>::infinity();

  [[nodiscard]] bool traversable(
    const gpr_common::GridMap & grid, int mx, int my) const;
  [[nodiscard]] bool path_is_traversable(
    const gpr_common::GridMap & grid, const gpr_common::Polyline & path) const;
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__ASTAR_GRID_PLANNER_HPP_

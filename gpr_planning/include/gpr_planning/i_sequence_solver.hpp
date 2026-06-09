#ifndef GPR_PLANNING__I_SEQUENCE_SOLVER_HPP_
#define GPR_PLANNING__I_SEQUENCE_SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/grid_map.hpp"
#include "gpr_common/types.hpp"
#include "gpr_planning/astar_grid_planner.hpp"
#include "gpr_planning/sequence_solver_utils.hpp"

namespace gpr_planning
{

/// @brief OR-Tools ATSP tunables (time limit and routing penalties).
struct AtspSolverConfig
{
  double time_limit_sec{2.0};
  /// @brief Penalty when OR-Tools skips one direction of a bidirectional segment pair.
  double direction_disjunction_penalty{1e8};
  /// @brief Cost matrix entry when A* transit between nodes is infeasible.
  double unreachable_transit_cost{1e9};
  /// @brief Float-to-int multiplier for OR-Tools arc costs.
  double cost_scale{1000.0};
  /// @brief OR-Tools first-solution strategy name (e.g. PATH_CHEAPEST_ARC).
  std::string first_solution_strategy{"PATH_CHEAPEST_ARC"};
};

/// @brief Orders open segments into a visiting schedule (ATSP-style).
class ISequenceSolver
{
public:
  virtual ~ISequenceSolver() = default;

  /// @brief Return an ordered job list covering each open segment once.
  [[nodiscard]] virtual std::vector<gpr_common::CoverageJob> solve(
    const gpr_common::Pose2D & robot_pose,
    const std::vector<gpr_common::CoverageSegment> & open_segments,
    const gpr_common::GridMap & grid,
    const AStarGridPlanner & astar,
    const SequenceSolverOptions & options = {}) const = 0;
};

/// @brief Factory: OR-Tools solver when available, else heuristic fallback.
[[nodiscard]] std::unique_ptr<ISequenceSolver> create_sequence_solver(
  const AtspSolverConfig & config = {});

}  // namespace gpr_planning

#endif  // GPR_PLANNING__I_SEQUENCE_SOLVER_HPP_

#ifndef GPR_PLANNING__HEURISTIC_ATSP_SOLVER_HPP_
#define GPR_PLANNING__HEURISTIC_ATSP_SOLVER_HPP_

#include "gpr_planning/i_sequence_solver.hpp"

namespace gpr_planning
{

/// @brief Greedy nearest-neighbor sequencer used when OR-Tools is unavailable.
class HeuristicAtspSolver : public ISequenceSolver
{
public:
  [[nodiscard]] std::vector<gpr_common::CoverageJob> solve(
    const gpr_common::Pose2D & robot_pose,
    const std::vector<gpr_common::CoverageSegment> & open_segments,
    const gpr_common::GridMap & grid,
    const AStarGridPlanner & astar,
    const SequenceSolverOptions & options = {}) const override;
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__HEURISTIC_ATSP_SOLVER_HPP_

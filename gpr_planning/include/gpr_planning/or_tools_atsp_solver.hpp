#ifndef GPR_PLANNING__OR_TOOLS_ATSP_SOLVER_HPP_
#define GPR_PLANNING__OR_TOOLS_ATSP_SOLVER_HPP_

#include "gpr_planning/i_sequence_solver.hpp"  // AtspSolverConfig

namespace gpr_planning
{

/// @brief ATSP sequencer backed by OR-Tools routing (one direction per merged pass when fixed).
class OrToolsAtspSolver : public ISequenceSolver
{
public:
  explicit OrToolsAtspSolver(const AtspSolverConfig & config = {})
  : config_(config) {}

  [[nodiscard]] std::vector<gpr_common::CoverageJob> solve(
    const gpr_common::Pose2D & robot_pose,
    const std::vector<gpr_common::CoverageSegment> & open_segments,
    const gpr_common::GridMap & grid,
    const AStarGridPlanner & astar,
    const SequenceSolverOptions & options = {}) const override;

private:
  AtspSolverConfig config_;
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__OR_TOOLS_ATSP_SOLVER_HPP_

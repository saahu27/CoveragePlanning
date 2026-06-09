#include "gpr_planning/i_sequence_solver.hpp"

#if GPR_HAS_ORTOOLS
#include "gpr_planning/or_tools_atsp_solver.hpp"
#endif
#include "gpr_planning/heuristic_atsp_solver.hpp"

namespace gpr_planning
{

std::unique_ptr<ISequenceSolver> create_sequence_solver(const AtspSolverConfig & config)
{
#if GPR_HAS_ORTOOLS
  return std::make_unique<OrToolsAtspSolver>(config);
#else
  (void)config;
  return std::make_unique<HeuristicAtspSolver>();
#endif
}

}  // namespace gpr_planning

#include "gpr_planning/heuristic_atsp_solver.hpp"

#include <cmath>
#include <limits>
#include <unordered_set>

#include "gpr_planning/sequence_solver_utils.hpp"

namespace gpr_planning
{

std::vector<gpr_common::CoverageJob> HeuristicAtspSolver::solve(
  const gpr_common::Pose2D & robot_pose,
  const std::vector<gpr_common::CoverageSegment> & open_segments,
  const gpr_common::GridMap & grid,
  const AStarGridPlanner & astar,
  const SequenceSolverOptions & options) const
{
  const auto directed = build_directed_jobs(open_segments, options.fixed_directions);
  if (directed.empty()) {
    return {};
  }

  std::unordered_set<gpr_common::SegmentId> remaining_segments;
  for (const auto & seg : open_segments) {
    remaining_segments.insert(seg.id);
  }

  std::vector<gpr_common::CoverageJob> schedule;
  gpr_common::Pose2D current = robot_pose;

  while (!remaining_segments.empty()) {
    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t best_idx = directed.size();
    for (std::size_t i = 0; i < directed.size(); ++i) {
      if (remaining_segments.count(directed[i].job.segment_id) == 0U) {
        continue;
      }
      const double cost = astar.transit_cost(grid, current, directed[i].entry);
      if (cost < best_cost) {
        best_cost = cost;
        best_idx = i;
      }
    }
    if (best_idx >= directed.size() || !std::isfinite(best_cost)) {
      break;
    }

    const bool last_job = remaining_segments.size() == 1U;
    if (last_job && options.route_end_pose.has_value()) {
      double best_total = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < directed.size(); ++i) {
        if (remaining_segments.count(directed[i].job.segment_id) == 0U) {
          continue;
        }
        const double to_job = astar.transit_cost(grid, current, directed[i].entry);
        const double to_home = astar.transit_cost(
          grid, directed[i].exit, *options.route_end_pose);
        if (!std::isfinite(to_job) || !std::isfinite(to_home)) {
          continue;
        }
        const double total = to_job + to_home;
        if (total < best_total) {
          best_total = total;
          best_idx = i;
          best_cost = to_job;
        }
      }
      if (best_idx >= directed.size() || !std::isfinite(best_total)) {
        break;
      }
    }

    schedule.push_back(directed[best_idx].job);
    remaining_segments.erase(directed[best_idx].job.segment_id);
    current = directed[best_idx].exit;
  }
  return schedule;
}

}  // namespace gpr_planning

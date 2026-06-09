#include "gpr_planning/or_tools_atsp_solver.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "gpr_planning/heuristic_atsp_solver.hpp"
#include "gpr_planning/sequence_solver_utils.hpp"

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include "ortools/constraint_solver/routing_parameters.pb.h"

namespace gpr_planning
{

namespace
{
using NodeIndex = operations_research::RoutingIndexManager::NodeIndex;

operations_research::FirstSolutionStrategy::Value parse_first_solution_strategy(
  const std::string & name)
{
  if (name == "PATH_MOST_CONSTRAINED_ARC") {
    return operations_research::FirstSolutionStrategy::PATH_MOST_CONSTRAINED_ARC;
  }
  if (name == "SAVINGS") {
    return operations_research::FirstSolutionStrategy::SAVINGS;
  }
  if (name == "CHRISTOFIDES") {
    return operations_research::FirstSolutionStrategy::CHRISTOFIDES;
  }
  return operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC;
}
}  // namespace

std::vector<gpr_common::CoverageJob> OrToolsAtspSolver::solve(
  const gpr_common::Pose2D & robot_pose,
  const std::vector<gpr_common::CoverageSegment> & open_segments,
  const gpr_common::GridMap & grid,
  const AStarGridPlanner & astar,
  const SequenceSolverOptions & options) const
{
  const auto jobs = build_directed_jobs(open_segments, options.fixed_directions);
  if (jobs.empty()) {
    return {};
  }

  const bool ends_at_home = options.route_end_pose.has_value();
  const int job_node_count = static_cast<int>(jobs.size());
  const int num_nodes = job_node_count + (ends_at_home ? 2 : 1);
  const int start_node = 0;
  const int end_node = ends_at_home ? num_nodes - 1 : 0;

  const auto node_from_pose = [&](const int node) -> gpr_common::Pose2D {
      if (node == start_node) {
        return robot_pose;
      }
      if (ends_at_home && node == end_node) {
        return *options.route_end_pose;
      }
      return jobs[static_cast<std::size_t>(node - 1)].exit;
    };
  const auto node_to_pose = [&](const int node) -> gpr_common::Pose2D {
      if (node == start_node) {
        return robot_pose;
      }
      if (ends_at_home && node == end_node) {
        return *options.route_end_pose;
      }
      return jobs[static_cast<std::size_t>(node - 1)].entry;
    };

  std::vector<std::vector<int64_t>> cost_matrix(
    static_cast<std::size_t>(num_nodes),
    std::vector<int64_t>(static_cast<std::size_t>(num_nodes), 0));
  for (int from = 0; from < num_nodes; ++from) {
    const gpr_common::Pose2D from_pose = node_from_pose(from);
    for (int to = 0; to < num_nodes; ++to) {
      if (from == to) {
        continue;
      }
      const double cost = astar.transit_cost(grid, from_pose, node_to_pose(to));
      const int64_t unreachable = static_cast<int64_t>(config_.unreachable_transit_cost);
      cost_matrix[static_cast<std::size_t>(from)][static_cast<std::size_t>(to)] =
        std::isfinite(cost) ?
        static_cast<int64_t>(std::lround(cost * config_.cost_scale)) :
        unreachable;
    }
  }

  operations_research::RoutingIndexManager manager(
    num_nodes, 1,
    std::vector<NodeIndex>{NodeIndex{start_node}},
    std::vector<NodeIndex>{NodeIndex{ends_at_home ? end_node : start_node}});
  operations_research::RoutingModel routing(manager);

  const auto cost_fn = [&](const int64_t from_index, const int64_t to_index) -> int64_t {
      const int from = manager.IndexToNode(from_index).value();
      const int to = manager.IndexToNode(to_index).value();
      return cost_matrix[static_cast<std::size_t>(from)][static_cast<std::size_t>(to)];
    };

  routing.SetArcCostEvaluatorOfAllVehicles(
    routing.RegisterTransitCallback(cost_fn));

  bool skip_disjunctions = options.fixed_directions != nullptr;
  if (skip_disjunctions) {
    for (const auto & seg : open_segments) {
      if (options.fixed_directions->find(seg.id) == options.fixed_directions->end()) {
        skip_disjunctions = false;
        break;
      }
    }
  }
  if (!skip_disjunctions) {
    std::unordered_map<gpr_common::SegmentId, std::vector<int64_t>> segment_nodes;
    for (std::size_t i = 0; i < jobs.size(); ++i) {
      segment_nodes[jobs[i].segment_id].push_back(
        manager.NodeToIndex(NodeIndex{static_cast<int>(i + 1)}));
    }
    for (const auto & pair : segment_nodes) {
      if (pair.second.size() > 1U) {
        routing.AddDisjunction(
          pair.second,
          static_cast<int64_t>(config_.direction_disjunction_penalty), 1);
      }
    }
  }

  operations_research::RoutingSearchParameters params =
    operations_research::DefaultRoutingSearchParameters();
  params.set_first_solution_strategy(
    parse_first_solution_strategy(config_.first_solution_strategy));
  const double bounded_limit =
    config_.time_limit_sec > 0.0 ? config_.time_limit_sec : 2.0;
  const auto whole_secs = static_cast<int64_t>(bounded_limit);
  params.mutable_time_limit()->set_seconds(whole_secs);
  params.mutable_time_limit()->set_nanos(
    static_cast<int32_t>((bounded_limit - static_cast<double>(whole_secs)) * 1e9));

  const operations_research::Assignment * solution =
    routing.SolveWithParameters(params);
  if (solution == nullptr) {
    HeuristicAtspSolver fallback;
    return fallback.solve(robot_pose, open_segments, grid, astar, options);
  }

  std::vector<gpr_common::CoverageJob> schedule;
  int64_t index = routing.Start(0);
  while (!routing.IsEnd(index)) {
    const int node = manager.IndexToNode(index).value();
    if (node > 0 && (!ends_at_home || node < end_node)) {
      schedule.push_back(jobs[static_cast<std::size_t>(node - 1)].job);
    }
    index = solution->Value(routing.NextVar(index));
  }
  return schedule;
}

}  // namespace gpr_planning

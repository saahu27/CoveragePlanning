#include "gpr_planning/astar_grid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace gpr_planning
{

namespace
{
/// @brief Grid cell coordinate used as a map/visited key.
struct NodeKey
{
  int mx;
  int my;
  bool operator==(const NodeKey & o) const {return mx == o.mx && my == o.my;}
};

/// @brief Hash for NodeKey in the open/closed maps.
struct NodeKeyHash
{
  std::size_t operator()(const NodeKey & k) const
  {
    return static_cast<std::size_t>(k.my * 10000 + k.mx);
  }
};

/// @brief Priority-queue entry ordered by f = g + h (min-first).
struct OpenNode
{
  double f;
  int mx;
  int my;
  bool operator>(const OpenNode & o) const {return f > o.f;}
};
}  // namespace

AStarGridPlanner::AStarGridPlanner(AStarConfig config)
: config_(std::move(config))
{}

/// @brief True if the robot footprint fits at the center of cell (mx, my).
bool AStarGridPlanner::traversable(
  const gpr_common::GridMap & grid, int mx, int my) const
{
  return !gpr_common::GridMapUtils::is_pose_colliding(
    grid,
    grid.origin_x + (mx + 0.5) * grid.resolution,
    grid.origin_y + (my + 0.5) * grid.resolution,
    config_.obstacle_cost_threshold,
    config_.footprint_radius,
    config_.treat_unknown_as_free);
}

bool AStarGridPlanner::path_is_traversable(
  const gpr_common::GridMap & grid, const gpr_common::Polyline & path) const
{
  if (path.size() < 2U) {
    return false;
  }
  const double spacing = (grid.resolution > 0.0) ?
    grid.resolution : config_.fallback_grid_resolution_m;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path.points[i - 1U];
    const auto & b = path.points[i];
    if (gpr_common::GridMapUtils::is_segment_colliding(
        grid, a.x, a.y, b.x, b.y,
        config_.obstacle_cost_threshold, spacing,
        config_.footprint_radius, config_.treat_unknown_as_free,
        config_.collision_sample_resolution_factor))
    {
      return false;
    }
  }
  return true;
}

double AStarGridPlanner::transit_cost(
  const gpr_common::GridMap & grid,
  const gpr_common::Pose2D & from,
  const gpr_common::Pose2D & to) const
{
  // Geometric cost estimate for ATSP sequencing. We deliberately avoid A* here:
  // the solver queries O(N^2) transit costs, so running A* per pair blocks the
  // mission. Euclidean distance (with a penalty when the straight line is blocked)
  // is sufficient for ordering lanes. The obstacle-aware A* is still used to plan
  // the ACTUAL transit path once per executed job (transit_to_current_job).
  const double euclid = std::hypot(to.x - from.x, to.y - from.y);
  const double sample_spacing = (grid.resolution > 0.0) ?
    grid.resolution : config_.fallback_grid_resolution_m;
  const bool blocked = gpr_common::GridMapUtils::is_segment_colliding(
    grid, from.x, from.y, to.x, to.y,
    config_.obstacle_cost_threshold, sample_spacing,
    config_.footprint_radius, config_.treat_unknown_as_free,
    config_.collision_sample_resolution_factor);
  return blocked ? euclid * config_.blocked_transit_penalty : euclid;
}

std::optional<gpr_common::Polyline> AStarGridPlanner::plan_path(
  const gpr_common::GridMap & grid,
  const gpr_common::Pose2D & from,
  const gpr_common::Pose2D & to) const
{
  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!gpr_common::GridMapUtils::world_to_map(grid, from.x, from.y, sx, sy) ||
    !gpr_common::GridMapUtils::world_to_map(grid, to.x, to.y, gx, gy))
  {
    return std::nullopt;
  }

  if (sx == gx && sy == gy) {
    return std::nullopt;
  }

  if (!traversable(grid, sx, sy) || !traversable(grid, gx, gy)) {
    return std::nullopt;
  }

  const auto heuristic = [&](int mx, int my) {
      return std::hypot(mx - gx, my - gy) * grid.resolution;
    };

  std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<>> open;
  std::unordered_map<NodeKey, double, NodeKeyHash> g_score;
  std::unordered_map<NodeKey, NodeKey, NodeKeyHash> came_from;

  const NodeKey start{sx, sy};
  g_score[start] = 0.0;
  open.push({heuristic(sx, sy), sx, sy});

  const int neighbors[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    if (current.mx == gx && current.my == gy) {
      gpr_common::Polyline path;
      NodeKey cur{current.mx, current.my};
      while (!(cur.mx == sx && cur.my == sy)) {
        const double wx = grid.origin_x + (cur.mx + 0.5) * grid.resolution;
        const double wy = grid.origin_y + (cur.my + 0.5) * grid.resolution;
        path.points.push_back({wx, wy, 0.0});
        cur = came_from.at(cur);
      }
      path.points.push_back(from);
      std::reverse(path.points.begin(), path.points.end());
      path.points.push_back(to);
      if (!path_is_traversable(grid, path)) {
        return std::nullopt;
      }
      return path;
    }

    const NodeKey cur_key{current.mx, current.my};
    const double g_cur = g_score.at(cur_key);
    if (current.f > g_cur + heuristic(current.mx, current.my) + 1e-6) {
      continue;
    }

    for (const auto & n : neighbors) {
      const int nx = current.mx + n[0];
      const int ny = current.my + n[1];
      if (!traversable(grid, nx, ny)) {
        continue;
      }
      const double step = (n[0] == 0 || n[1] == 0) ? grid.resolution :
        grid.resolution * 1.41421356237;
      const NodeKey nk{nx, ny};
      const double tentative = g_cur + step;
      const auto it = g_score.find(nk);
      if (it == g_score.end() || tentative < it->second) {
        g_score[nk] = tentative;
        came_from[nk] = cur_key;
        open.push({tentative + heuristic(nx, ny), nx, ny});
      }
    }
  }
  return std::nullopt;
}

}  // namespace gpr_planning

#include "gpr_common/grid_map.hpp"

#include <algorithm>
#include <cmath>

namespace gpr_common
{

namespace
{
/// @brief Guard against divide-by-zero / degenerate geometry.
constexpr double kEpsilon = 1e-9;
}

bool GridMapUtils::world_to_map(
  const GridMap & grid, double x, double y, int & mx, int & my) noexcept
{
  if (grid.resolution <= kEpsilon) {
    return false;
  }
  mx = static_cast<int>(std::floor((x - grid.origin_x) / grid.resolution));
  my = static_cast<int>(std::floor((y - grid.origin_y) / grid.resolution));
  return mx >= 0 && mx < grid.width && my >= 0 && my < grid.height;
}

bool GridMapUtils::cell_cost(
  const GridMap & grid, int mx, int my, int8_t & cost) noexcept
{
  if (mx < 0 || mx >= grid.width || my < 0 || my >= grid.height) {
    return false;
  }
  const std::size_t idx = static_cast<std::size_t>(my * grid.width + mx);
  if (idx >= grid.data.size()) {
    return false;
  }
  cost = grid.data[idx];
  return true;
}

bool GridMapUtils::occupancy_cost_at(
  const GridMap & grid, double x, double y, int8_t & cost) noexcept
{
  int mx = 0;
  int my = 0;
  if (!world_to_map(grid, x, y, mx, my)) {
    return false;
  }
  return cell_cost(grid, mx, my, cost);
}

bool GridMapUtils::is_cell_blocked(
  int8_t cost, int obstacle_threshold, bool treat_unknown_as_free) noexcept
{
  if (cost < 0) {
    return !treat_unknown_as_free;
  }
  return cost >= obstacle_threshold;
}

bool GridMapUtils::is_pose_colliding(
  const GridMap & grid, double x, double y,
  int obstacle_threshold, double footprint_radius,
  bool treat_unknown_as_free)
{
  const auto check_cell = [&](int mx, int my) {
      int8_t cost = 0;
      if (!cell_cost(grid, mx, my, cost)) {
        return !treat_unknown_as_free;
      }
      return is_cell_blocked(cost, obstacle_threshold, treat_unknown_as_free);
    };

  int mx = 0;
  int my = 0;
  if (!world_to_map(grid, x, y, mx, my)) {
    return !treat_unknown_as_free;
  }

  if (footprint_radius <= kEpsilon) {
    return check_cell(mx, my);
  }

  const int radius_cells = static_cast<int>(
    std::ceil(footprint_radius / grid.resolution));
  const int r_sq = radius_cells * radius_cells;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (dx * dx + dy * dy > r_sq) {
        continue;
      }
      if (check_cell(mx + dx, my + dy)) {
        return true;
      }
    }
  }
  return false;
}

bool GridMapUtils::is_segment_colliding(
  const GridMap & grid,
  double x0, double y0, double x1, double y1,
  int obstacle_threshold, double segment_sample_spacing,
  double footprint_radius, bool treat_unknown_as_free,
  const double resolution_sample_factor)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double length = std::hypot(dx, dy);
  if (length < kEpsilon) {
    return is_pose_colliding(
      grid, x0, y0, obstacle_threshold, footprint_radius, treat_unknown_as_free);
  }

  const double spacing = std::max(
    segment_sample_spacing, grid.resolution * resolution_sample_factor);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / spacing)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    if (is_pose_colliding(
        grid, x0 + t * dx, y0 + t * dy,
        obstacle_threshold, footprint_radius, treat_unknown_as_free))
    {
      return true;
    }
  }
  return false;
}

}  // namespace gpr_common

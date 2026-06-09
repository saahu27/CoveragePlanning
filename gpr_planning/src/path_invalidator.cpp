#include "gpr_planning/path_invalidator.hpp"

#include <algorithm>
#include <cmath>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/grid_map.hpp"
#include "gpr_common/polygon_geometry.hpp"

namespace gpr_planning
{

namespace
{
/// @brief True when @p x,@p y lies within @p margin of the mission polygon edge.
bool near_scan_boundary(
  double x, double y, const gpr_common::ScanRegion & region, double margin) noexcept
{
  if (margin <= 0.0 || region.vertices.size() < 3U) {
    return false;
  }
  return gpr_common::distance_to_polygon_boundary(x, y, region.vertices) <= margin;
}

/// @brief True if a grid cell should count as blocking (ignoring perimeter inflation).
bool cell_blocks_for_invalidator(
  const gpr_common::GridMap & grid, int mx, int my,
  const PathInvalidatorConfig & config)
{
  int8_t cost = 0;
  if (!gpr_common::GridMapUtils::cell_cost(grid, mx, my, cost)) {
    return !config.treat_unknown_as_free;
  }
  if (cost < 0) {
    return config.block_only_observed_occupied ? false : !config.treat_unknown_as_free;
  }
  if (!gpr_common::GridMapUtils::is_cell_blocked(
      cost, config.obstacle_cost_threshold, config.treat_unknown_as_free))
  {
    return false;
  }
  if (config.boundary_ignore_margin > 0.0) {
    const double wx = grid.origin_x + (mx + 0.5) * grid.resolution;
    const double wy = grid.origin_y + (my + 0.5) * grid.resolution;
    if (near_scan_boundary(wx, wy, config.region, config.boundary_ignore_margin)) {
      return false;
    }
  }
  return true;
}

bool collides_at(
  const gpr_common::GridMap & grid, double x, double y,
  const PathInvalidatorConfig & config)
{
  constexpr double kEpsilon = 1e-9;
  int mx = 0;
  int my = 0;
  if (!gpr_common::GridMapUtils::world_to_map(grid, x, y, mx, my)) {
    return !config.treat_unknown_as_free;
  }

  const double clearance = PathInvalidator::effective_footprint_radius(config);
  if (clearance <= kEpsilon) {
    return cell_blocks_for_invalidator(grid, mx, my, config);
  }

  const int radius_cells = static_cast<int>(
    std::ceil(clearance / grid.resolution));
  const int r_sq = radius_cells * radius_cells;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (dx * dx + dy * dy > r_sq) {
        continue;
      }
      if (cell_blocks_for_invalidator(grid, mx + dx, my + dy, config)) {
        return true;
      }
    }
  }
  return false;
}

bool segment_collides(
  const gpr_common::GridMap & grid,
  double x0, double y0, double x1, double y1,
  const PathInvalidatorConfig & config)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double length = std::hypot(dx, dy);
  if (length < 1e-9) {
    return collides_at(grid, x0, y0, config);
  }
  const double spacing = std::max(
    config.segment_sample_spacing,
    grid.resolution * config.collision_sample_resolution_factor);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / spacing)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    if (collides_at(grid, x0 + t * dx, y0 + t * dy, config)) {
      return true;
    }
  }
  return false;
}
}  // namespace

std::vector<gpr_common::CoverageSegment> PathInvalidator::invalidate_segments(
  const gpr_common::Polyline & initial_path,
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & config)
{
  std::vector<gpr_common::CoverageSegment> segments;
  if (initial_path.empty() || grid.empty()) {
    return segments;
  }

  gpr_common::Polyline current;
  const auto flush = [&]() {
      if (current.size() < 2U) {
        current.points.clear();
        return;
      }
      gpr_common::CoverageSegment seg;
      seg.centerline = current;
      seg.id = gpr_common::compute_segment_id(
        seg.centerline, std::nullopt, config.legacy_segment_quantize_m);
      seg.outcome = gpr_common::SegmentOutcome::Pending;
      segments.push_back(std::move(seg));
      current.points.clear();
    };

  for (std::size_t i = 0; i < initial_path.size(); ++i) {
    const auto & p = initial_path.points[i];
    if (collides_at(grid, p.x, p.y, config)) {
      flush();
      continue;
    }
    if (i > 0U) {
      const auto & prev = initial_path.points[i - 1U];
      if (segment_collides(grid, prev.x, prev.y, p.x, p.y, config)) {
        flush();
      }
    }
    if (!current.empty()) {
      const auto & last = current.points.back();
      if (segment_collides(grid, last.x, last.y, p.x, p.y, config)) {
        flush();
      }
    }
    current.points.push_back(p);
  }
  flush();
  return segments;
}

bool PathInvalidator::is_blocked(
  const gpr_common::Polyline & centerline,
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & config)
{
  if (centerline.empty() || grid.empty()) {
    return false;
  }
  for (std::size_t i = 0; i < centerline.size(); ++i) {
    const auto & p = centerline.points[i];
    if (collides_at(grid, p.x, p.y, config)) {
      return true;
    }
    if (i > 0U) {
      const auto & prev = centerline.points[i - 1U];
      if (segment_collides(grid, prev.x, prev.y, p.x, p.y, config)) {
        return true;
      }
    }
  }
  return false;
}

std::vector<gpr_common::Polyline> PathInvalidator::free_subpolylines(
  const gpr_common::Polyline & centerline,
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & config)
{
  const auto segments = invalidate_segments(centerline, grid, config);
  std::vector<gpr_common::Polyline> polylines;
  polylines.reserve(segments.size());
  for (const auto & seg : segments) {
    if (seg.centerline.size() >= 2U) {
      polylines.push_back(seg.centerline);
    }
  }
  return polylines;
}

std::vector<gpr_common::Polyline> PathInvalidator::blocked_subpolylines(
  const gpr_common::Polyline & centerline,
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & config)
{
  std::vector<gpr_common::Polyline> blocked;
  if (centerline.size() < 2U || grid.empty()) {
    return blocked;
  }

  gpr_common::Polyline current;
  const auto flush = [&]() {
      if (current.size() >= 2U) {
        blocked.push_back(current);
      }
      current.points.clear();
    };

  for (std::size_t i = 0; i < centerline.size(); ++i) {
    const auto & p = centerline.points[i];
    const bool point_blocked = collides_at(grid, p.x, p.y, config);
    bool edge_blocked = false;
    if (i > 0U) {
      const auto & prev = centerline.points[i - 1U];
      edge_blocked = segment_collides(grid, prev.x, prev.y, p.x, p.y, config);
    }
    if (!point_blocked && !edge_blocked) {
      flush();
      continue;
    }
    current.points.push_back(p);
  }
  flush();
  return blocked;
}

double PathInvalidator::effective_footprint_radius(
  const PathInvalidatorConfig & config) noexcept
{
  // Nav2 inflates once on the threshold planning grid; mission uses robot
  // footprint clearance only (map_inflation_radius should be 0).
  return config.map_inflation_radius > 0.0 ?
         std::max(config.footprint_radius, config.map_inflation_radius) :
         config.footprint_radius;
}

}  // namespace gpr_planning

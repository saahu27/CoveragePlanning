#include "gpr_perception/occupancy_grid_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace gpr_perception
{

namespace
{
constexpr double kAreaEpsilon = 1e-9;

/// @brief Throw if the scan area has non-positive width/height.
void validate_scan_area(const gpr_common::ScanArea & area)
{
  if (area.x_max <= area.x_min + kAreaEpsilon) {
    throw OccupancyGridConfigError("scan_area x bounds are invalid.");
  }
  if (area.y_max <= area.y_min + kAreaEpsilon) {
    throw OccupancyGridConfigError("scan_area y bounds are invalid.");
  }
}

/// @brief Clamp a probability away from 0 and 1 so log-odds stays finite.
double clamp_probability(double p)
{
  return std::clamp(p, kAreaEpsilon, 1.0 - kAreaEpsilon);
}
}  // namespace

/// @brief Convert an occupancy probability to its log-odds value.
double OccupancyGridMapper::probability_to_log_odds(double probability)
{
  const double p = clamp_probability(probability);
  return std::log(p / (1.0 - p));
}

void OccupancyGridMapper::validate_config(const OccupancyGridConfig & config)
{
  validate_scan_area(config.scan_area);
  if (config.resolution <= kEpsilon) {
    throw OccupancyGridConfigError("resolution must be positive.");
  }
}

OccupancyGridMapper::OccupancyGridMapper(OccupancyGridConfig config)
: config_(std::move(config))
{
  validate_config(config_);
  origin_x_ = config_.scan_area.x_min;
  origin_y_ = config_.scan_area.y_min;
  width_ = static_cast<int>(std::ceil(
      (config_.scan_area.x_max - config_.scan_area.x_min) / config_.resolution));
  height_ = static_cast<int>(std::ceil(
      (config_.scan_area.y_max - config_.scan_area.y_min) / config_.resolution));
  width_ = std::max(width_, 1);
  height_ = std::max(height_, 1);
  log_odds_hit_ = probability_to_log_odds(config_.prob_hit);
  log_odds_miss_ = probability_to_log_odds(config_.prob_miss);
  reset();
}

void OccupancyGridMapper::reset()
{
  const std::size_t n = static_cast<std::size_t>(width_ * height_);
  log_odds_.assign(n, 0.0);
  observed_.assign(n, 0U);
  ++revision_;
}

/// @brief Convert world coords to cell indices; false if outside the grid.
bool OccupancyGridMapper::world_to_map(double x, double y, int & mx, int & my) const noexcept
{
  mx = static_cast<int>(std::floor((x - origin_x_) / config_.resolution));
  my = static_cast<int>(std::floor((y - origin_y_) / config_.resolution));
  return cell_in_bounds(mx, my);
}

/// @brief Row-major flat index for a cell.
int OccupancyGridMapper::cell_index(int mx, int my) const noexcept
{
  return my * width_ + mx;
}

/// @brief True if the cell indices lie within the grid.
bool OccupancyGridMapper::cell_in_bounds(int mx, int my) const noexcept
{
  return mx >= 0 && mx < width_ && my >= 0 && my < height_;
}

/// @brief Add @p delta to a cell's log-odds (clamped) and mark it observed.
void OccupancyGridMapper::apply_log_odds_update(int mx, int my, double delta)
{
  if (!cell_in_bounds(mx, my)) {
    return;
  }
  const int idx = cell_index(mx, my);
  log_odds_[static_cast<std::size_t>(idx)] = std::clamp(
    log_odds_[static_cast<std::size_t>(idx)] + delta,
    -config_.log_odds_clamp, config_.log_odds_clamp);
  observed_[static_cast<std::size_t>(idx)] = 1U;
}

/// @brief Apply the hit update over a disk around an obstacle return.
void OccupancyGridMapper::mark_hit_disk(double hit_x, double hit_y)
{
  int center_mx = 0;
  int center_my = 0;
  if (!world_to_map(hit_x, hit_y, center_mx, center_my)) {
    return;
  }
  const int radius_cells = static_cast<int>(
    std::ceil(config_.hit_marking_radius / config_.resolution));
  const int r_sq = radius_cells * radius_cells;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (dx * dx + dy * dy > r_sq) {
        continue;
      }
      apply_log_odds_update(center_mx + dx, center_my + dy, log_odds_hit_);
    }
  }
}

/// @brief Bresenham walk marking cells free, and the endpoint occupied if a hit.
void OccupancyGridMapper::ray_trace(
  int x0, int y0, int x1, int y1, bool endpoint_is_obstacle)
{
  int x = x0;
  int y = y0;
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = (x0 < x1) ? 1 : -1;
  const int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    const bool at_endpoint = (x == x1 && y == y1);
    if (at_endpoint) {
      if (endpoint_is_obstacle) {
        apply_log_odds_update(x, y, log_odds_hit_);
      }
      break;
    }
    apply_log_odds_update(x, y, log_odds_miss_);
    const int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 < dx) { err += dx; y += sy; }
  }
}

void OccupancyGridMapper::integrate_ray(
  double origin_x, double origin_y,
  double hit_x, double hit_y,
  bool endpoint_is_obstacle)
{
  int mx0 = 0, my0 = 0, mx1 = 0, my1 = 0;
  if (!world_to_map(origin_x, origin_y, mx0, my0)) {
    return;
  }
  if (!world_to_map(hit_x, hit_y, mx1, my1)) {
    if (!endpoint_is_obstacle) {
      return;
    }
    hit_x = std::clamp(hit_x, origin_x_, origin_x_ + width_ * config_.resolution);
    hit_y = std::clamp(hit_y, origin_y_, origin_y_ + height_ * config_.resolution);
    if (!world_to_map(hit_x, hit_y, mx1, my1)) {
      return;
    }
  }
  ray_trace(mx0, my0, mx1, my1, endpoint_is_obstacle);
  if (endpoint_is_obstacle) {
    mark_hit_disk(hit_x, hit_y);
  }
}

/// @brief Threshold the log-odds field to costs (no inflation).
std::vector<int8_t> OccupancyGridMapper::build_threshold_export_grid() const
{
  const std::size_t n = static_cast<std::size_t>(width_ * height_);
  std::vector<int8_t> grid(n, -1);
  for (std::size_t idx = 0; idx < n; ++idx) {
    if (observed_[idx] == 0U) {
      continue;
    }
    const double odds = std::exp(log_odds_[idx]);
    const double prob = odds / (1.0 + odds);
    const int prob_percent = static_cast<int>(std::lround(prob * 100.0));
    if (prob_percent >= config_.occupied_export_threshold) {
      grid[idx] = static_cast<int8_t>(config_.occupied_export_threshold);
    } else {
      grid[idx] = static_cast<int8_t>(std::clamp(prob_percent, 0, 100));
    }
  }
  return grid;
}

/// @brief Threshold the log-odds field to costs and inflate lethal obstacles.
std::vector<int8_t> OccupancyGridMapper::build_inflated_export_grid() const
{
  const std::size_t n = static_cast<std::size_t>(width_ * height_);
  std::vector<int8_t> grid = build_threshold_export_grid();
  std::vector<uint8_t> hard_occupied(n, 0U);
  for (std::size_t idx = 0; idx < n; ++idx) {
    if (grid[idx] >= config_.occupied_export_threshold) {
      hard_occupied[idx] = 1U;
    }
  }
  const int inflate_cells = static_cast<int>(
    std::ceil(config_.inflation_radius / config_.resolution));
  if (inflate_cells <= 0) {
    return grid;
  }
  const int r_sq = inflate_cells * inflate_cells;
  for (int my = 0; my < height_; ++my) {
    for (int mx = 0; mx < width_; ++mx) {
      const std::size_t idx = static_cast<std::size_t>(cell_index(mx, my));
      if (hard_occupied[idx] == 0U) {
        continue;
      }
      for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
        for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
          if (dx * dx + dy * dy > r_sq) {
            continue;
          }
          const int nx = mx + dx;
          const int ny = my + dy;
          if (!cell_in_bounds(nx, ny)) {
            continue;
          }
          grid[static_cast<std::size_t>(cell_index(nx, ny))] = 100;
        }
      }
    }
  }
  return grid;
}

gpr_common::GridMap OccupancyGridMapper::to_grid_map() const
{
  gpr_common::GridMap map;
  map.resolution = config_.resolution;
  map.width = width_;
  map.height = height_;
  map.origin_x = origin_x_;
  map.origin_y = origin_y_;
  map.data = build_inflated_export_grid();
  map.revision = revision_;
  return map;
}

gpr_common::GridMap OccupancyGridMapper::to_planning_grid_map() const
{
  gpr_common::GridMap map;
  map.resolution = config_.resolution;
  map.width = width_;
  map.height = height_;
  map.origin_x = origin_x_;
  map.origin_y = origin_y_;
  map.data = build_threshold_export_grid();
  map.revision = revision_;
  return map;
}

}  // namespace gpr_perception

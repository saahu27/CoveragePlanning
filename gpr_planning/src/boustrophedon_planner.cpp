#include "gpr_planning/boustrophedon_planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "gpr_common/polygon_geometry.hpp"

namespace gpr_planning
{

namespace
{
constexpr double kAreaEpsilon = 1e-9;

/// @brief Throw if the scan area has non-positive width/height.
void validate_scan_area(const gpr_common::ScanArea & area)
{
  if (area.x_max <= area.x_min + kAreaEpsilon) {
    throw PlannerConfigurationError("scan_area x bounds invalid.");
  }
  if (area.y_max <= area.y_min + kAreaEpsilon) {
    throw PlannerConfigurationError("scan_area y bounds invalid.");
  }
}
}  // namespace

gpr_common::ScanDirection BoustrophedonPlanner::scan_direction_from_string(
  const std::string & value)
{
  if (value == "x") {
    return gpr_common::ScanDirection::kX;
  }
  if (value == "y") {
    return gpr_common::ScanDirection::kY;
  }
  throw PlannerConfigurationError("scan_direction must be 'x' or 'y'.");
}

void BoustrophedonPlanner::validate_config(const BoustrophedonConfig & config)
{
  validate_scan_area(config.region.bounds);
  if (config.region.vertices.size() < 3U) {
    throw PlannerConfigurationError("scan region requires a valid polygon boundary.");
  }
  if (config.lane_spacing <= kEpsilon || config.waypoint_spacing <= kEpsilon) {
    throw PlannerConfigurationError("spacings must be positive.");
  }
}

BoustrophedonPlanner::BoustrophedonPlanner(BoustrophedonConfig config)
: config_(std::move(config))
{
  validate_config(config_);
}

/// @brief Lane offsets from min to max at @p spacing, always including max.
std::vector<double> BoustrophedonPlanner::compute_lane_coordinates(
  double min_coord, double max_coord, double spacing)
{
  std::vector<double> coords;
  for (double c = min_coord; c <= max_coord + kEpsilon; c += spacing) {
    coords.push_back(c);
  }
  if (coords.back() < max_coord - kEpsilon) {
    coords.push_back(max_coord);
  }
  return coords;
}

/// @brief Append evenly-spaced waypoints along a straight edge to @p polyline.
/// @param skip_first Omit the start point to avoid duplicating a shared vertex.
void BoustrophedonPlanner::append_discretized_segment(
  gpr_common::Polyline & polyline,
  double x0, double y0, double x1, double y1,
  bool skip_first) const
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double length = std::hypot(dx, dy);
  if (length < kEpsilon) {
    return;
  }
  const int steps = std::max(1, static_cast<int>(std::ceil(length / config_.waypoint_spacing)));
  const double yaw = std::atan2(dy, dx);
  const int start = skip_first ? 1 : 0;
  for (int i = start; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    polyline.points.push_back({x0 + t * dx, y0 + t * dy, yaw});
  }
}

/// @brief Discretize a single edge into its own polyline.
gpr_common::Polyline BoustrophedonPlanner::discretize_segment(
  double x0, double y0, double x1, double y1) const
{
  gpr_common::Polyline polyline;
  append_discretized_segment(polyline, x0, y0, x1, y1, false);
  return polyline;
}

/// @brief Build inset boundary chords on parallel scan lines (polygon-aware).
std::vector<BoustrophedonPlanner::Lane> BoustrophedonPlanner::compute_lanes() const
{
  std::vector<Lane> lanes;
  const gpr_common::ScanRegion effective_region =
    config_.region.inset(config_.coverage_inset);
  const auto & bounds = effective_region.bounds;
  const auto & boundary = effective_region.vertices;

  std::vector<double> lane_coords;
  if (config_.scan_direction == gpr_common::ScanDirection::kX) {
    lane_coords = compute_lane_coordinates(bounds.x_min, bounds.x_max, config_.lane_spacing);
  } else {
    lane_coords = compute_lane_coordinates(bounds.y_min, bounds.y_max, config_.lane_spacing);
  }

  uint32_t lane_index = 0U;
  for (const double coord : lane_coords) {
    const auto chords = gpr_common::chords_on_scan_line(
      boundary, config_.scan_direction, coord);
    for (const auto & chord : chords) {
      const bool forward = (lane_index % 2U == 0U);
      double x0 = chord.a.x;
      double y0 = chord.a.y;
      double x1 = chord.b.x;
      double y1 = chord.b.y;
      if (!forward) {
        std::swap(x0, x1);
        std::swap(y0, y1);
      }
      lanes.push_back({lane_index, x0, y0, x1, y1});
      ++lane_index;
    }
  }
  return lanes;
}

gpr_common::Polyline BoustrophedonPlanner::generate() const
{
  gpr_common::Polyline polyline;
  const auto lanes = compute_lanes();
  bool skip_first = false;
  double prev_x = 0, prev_y = 0;
  for (std::size_t i = 0; i < lanes.size(); ++i) {
    const auto & lane = lanes[i];
    if (i > 0U) {
      append_discretized_segment(polyline, prev_x, prev_y, lane.x0, lane.y0, skip_first);
      skip_first = true;
    }
    append_discretized_segment(polyline, lane.x0, lane.y0, lane.x1, lane.y1, skip_first);
    skip_first = true;
    prev_x = lane.x1;
    prev_y = lane.y1;
  }
  return polyline;
}

std::vector<gpr_common::CoverageSegment> BoustrophedonPlanner::generate_segments() const
{
  std::vector<gpr_common::CoverageSegment> segments;
  const auto lanes = compute_lanes();
  for (const auto & lane : lanes) {
    const double dx = lane.x1 - lane.x0;
    const double dy = lane.y1 - lane.y0;
    const double length = std::hypot(dx, dy);
    if (length < kEpsilon) {
      continue;
    }
    const int pieces = std::max(
      1, static_cast<int>(std::ceil(length / config_.segment_length)));
    for (int s = 0; s < pieces; ++s) {
      const double t0 = static_cast<double>(s) / static_cast<double>(pieces);
      const double t1 = static_cast<double>(s + 1) / static_cast<double>(pieces);
      const double sx0 = lane.x0 + t0 * dx;
      const double sy0 = lane.y0 + t0 * dy;
      const double sx1 = lane.x0 + t1 * dx;
      const double sy1 = lane.y0 + t1 * dy;
      gpr_common::CoverageSegment seg;
      seg.centerline = discretize_segment(sx0, sy0, sx1, sy1);
      if (seg.centerline.size() < 2U) {
        continue;
      }
      seg.lane_index = lane.index;
      seg.id = gpr_common::make_segment_id(lane.index, static_cast<uint32_t>(s));
      seg.outcome = gpr_common::SegmentOutcome::Pending;
      seg.blocked = false;
      segments.push_back(std::move(seg));
    }
  }
  return segments;
}

std::vector<BoustrophedonPlanner::LaneCenterline> BoustrophedonPlanner::generate_lane_centerlines()
const
{
  std::vector<LaneCenterline> lanes_out;
  const auto lanes = compute_lanes();
  lanes_out.reserve(lanes.size());
  for (const auto & lane : lanes) {
    LaneCenterline lc;
    lc.index = lane.index;
    lc.centerline = discretize_segment(lane.x0, lane.y0, lane.x1, lane.y1);
    if (lc.centerline.size() >= 2U) {
      lanes_out.push_back(std::move(lc));
    }
  }
  return lanes_out;
}

std::vector<gpr_common::Polyline> BoustrophedonPlanner::generate_connectors() const
{
  std::vector<gpr_common::Polyline> connectors;
  const auto lanes = compute_lanes();
  if (lanes.size() < 2U) {
    return connectors;
  }
  connectors.reserve(lanes.size() - 1U);
  for (std::size_t i = 1; i < lanes.size(); ++i) {
    const auto & prev = lanes[i - 1U];
    const auto & lane = lanes[i];
    auto connector = discretize_segment(prev.x1, prev.y1, lane.x0, lane.y0);
    if (connector.size() >= 2U) {
      connectors.push_back(std::move(connector));
    }
  }
  return connectors;
}

}  // namespace gpr_planning

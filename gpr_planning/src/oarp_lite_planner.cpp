#include "gpr_planning/oarp_lite_planner.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace gpr_planning
{

namespace
{
bool point_near_polyline(const gpr_common::Polyline & line, double x, double y, double tol)
{
  for (std::size_t i = 0; i + 1U < line.points.size(); ++i) {
    const auto & a = line.points[i];
    const auto & b = line.points[i + 1U];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-12) {
      if (std::hypot(x - a.x, y - a.y) <= tol) {
        return true;
      }
      continue;
    }
    const double t = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / len_sq, 0.0, 1.0);
    const double px = a.x + t * dx;
    const double py = a.y + t * dy;
    if (std::hypot(x - px, y - py) <= tol) {
      return true;
    }
  }
  return false;
}
}  // namespace

OarpLitePlanner::OarpLitePlanner(
  BoustrophedonConfig boustrophedon_config, OarpLiteConfig oarp_config)
: planner_(std::move(boustrophedon_config)), oarp_config_(oarp_config)
{}

bool OarpLitePlanner::has_open_catalog_on_lane(
  const uint32_t lane_index,
  const std::vector<gpr_common::CoverageSegment> & catalog) const
{
  for (const auto & seg : catalog) {
    if (seg.lane_index != lane_index) {
      continue;
    }
    if (seg.source == gpr_common::SegmentSource::OarpRank) {
      continue;
    }
    if (gpr_common::is_segment_open(seg) ||
      gpr_common::is_segment_probe(seg))
    {
      return true;
    }
  }
  return false;
}

bool OarpLitePlanner::overlaps_completed(
  const gpr_common::Polyline & candidate,
  const std::vector<gpr_common::CoverageSegment> & catalog,
  const std::optional<uint32_t> lane_index) const
{
  for (const auto & seg : catalog) {
    if (seg.outcome != gpr_common::SegmentOutcome::Completed) {
      continue;
    }
    if (lane_index.has_value() && seg.lane_index != lane_index) {
      continue;
    }
    for (const auto & p : candidate.points) {
      if (point_near_polyline(seg.centerline, p.x, p.y, oarp_config_.overlap_dist_m)) {
        return true;
      }
    }
  }
  return false;
}

bool OarpLitePlanner::has_uncovered_ranks(
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & inv_config,
  const std::vector<gpr_common::CoverageSegment> & catalog) const
{
  if (!oarp_config_.enabled || grid.empty()) {
    return false;
  }
  return !generate_ranks(grid, inv_config, catalog, 0U).empty();
}

std::vector<gpr_common::CoverageSegment> OarpLitePlanner::generate_ranks(
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & inv_config,
  const std::vector<gpr_common::CoverageSegment> & catalog,
  const std::uint32_t replan_generation) const
{
  std::vector<gpr_common::CoverageSegment> ranks;
  if (!oarp_config_.enabled || grid.empty()) {
    return ranks;
  }

  const auto lanes = planner_.generate_lane_centerlines();
  const double piece_len = planner_.config().segment_length;
  std::uint32_t rank_index = 0U;

  for (const auto & lane : lanes) {
    if (has_open_catalog_on_lane(lane.index, catalog)) {
      continue;
    }

    const auto free_chunks = PathInvalidator::free_subpolylines(lane.centerline, grid, inv_config);
    for (const auto & chunk : free_chunks) {
      if (chunk.length() < oarp_config_.min_rank_length_m) {
        continue;
      }
      if (overlaps_completed(chunk, catalog, lane.index)) {
        continue;
      }

      const int pieces = std::max(
        1, static_cast<int>(std::ceil(chunk.length() / piece_len)));
      const auto & pts = chunk.points;
      if (pts.size() < 2U) {
        continue;
      }

      for (int s = 0; s < pieces; ++s) {
        const double t0 = static_cast<double>(s) / static_cast<double>(pieces);
        const double t1 = static_cast<double>(s + 1) / static_cast<double>(pieces);
        const double cx0 = pts.front().x + t0 * (pts.back().x - pts.front().x);
        const double cy0 = pts.front().y + t0 * (pts.back().y - pts.front().y);
        const double cx1 = pts.front().x + t1 * (pts.back().x - pts.front().x);
        const double cy1 = pts.front().y + t1 * (pts.back().y - pts.front().y);

        gpr_common::Polyline sub;
        const double dx = cx1 - cx0;
        const double dy = cy1 - cy0;
        const double yaw = std::atan2(dy, dx);
        const double sub_len = std::hypot(dx, dy);
        if (sub_len < oarp_config_.min_rank_length_m) {
          continue;
        }
        const int steps = std::max(
          1, static_cast<int>(std::ceil(
               sub_len / planner_.config().waypoint_spacing)));
        for (int i = 0; i <= steps; ++i) {
          const double t = static_cast<double>(i) / static_cast<double>(steps);
          sub.points.push_back({cx0 + t * dx, cy0 + t * dy, yaw});
        }
        if (sub.size() < 2U) {
          continue;
        }
        if (overlaps_completed(sub, catalog, lane.index)) {
          continue;
        }

        gpr_common::CoverageSegment rank;
        rank.centerline = std::move(sub);
        rank.lane_index = lane.index;
        rank.id = gpr_common::make_oarp_rank_id(replan_generation, rank_index++);
        rank.source = gpr_common::SegmentSource::OarpRank;
        rank.outcome = gpr_common::SegmentOutcome::Pending;
        rank.blocked = false;
        ranks.push_back(std::move(rank));
      }
    }
  }

  return ranks;
}

}  // namespace gpr_planning

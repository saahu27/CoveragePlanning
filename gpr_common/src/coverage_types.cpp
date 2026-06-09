#include "gpr_common/coverage_types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gpr_common
{

double uncovered_arc_length_m(const CoverageSegment & seg) noexcept
{
  const double total = polyline_arc_length(seg.centerline);
  if (total <= 1e-9) {
    return 0.0;
  }
  const auto merged = merge_intervals(seg.covered_intervals_m);
  return std::max(0.0, total - intervals_union_length(merged));
}

Polyline schedulable_centerline(
  const CoverageSegment & segment, const DriveDirection direction)
{
  Polyline line = segment.centerline;
  if (segment.outcome == SegmentOutcome::PartiallyCompleted &&
    !segment.covered_intervals_m.empty())
  {
    const double total = polyline_arc_length(line);
    const auto uncovered = complement_intervals(
      merge_intervals(segment.covered_intervals_m), total);
    if (!uncovered.empty()) {
      auto best = uncovered.front();
      double best_len = best.second - best.first;
      for (const auto & iv : uncovered) {
        const double len = iv.second - iv.first;
        if (len > best_len) {
          best = iv;
          best_len = len;
        }
      }
      line = sub_polyline_by_arc_length(line, best.first, best.second);
    } else {
      line.points.clear();
    }
  }
  if (direction == DriveDirection::Reverse && line.points.size() > 1U) {
    std::reverse(line.points.begin(), line.points.end());
    for (auto & p : line.points) {
      p.yaw += M_PI;
    }
  }
  return line;
}

Polyline job_polyline(const CoverageSegment & segment, DriveDirection direction)
{
  return schedulable_centerline(segment, direction);
}

Pose2D job_entry_pose(const CoverageSegment & segment, DriveDirection direction)
{
  if (segment.centerline.empty()) {
    return {};
  }
  return direction == DriveDirection::Forward ?
         segment.centerline.front_pose() :
         segment.centerline.back_pose();
}

Pose2D job_exit_pose(const CoverageSegment & segment, DriveDirection direction)
{
  if (segment.centerline.empty()) {
    return {};
  }
  return direction == DriveDirection::Forward ?
         segment.centerline.back_pose() :
         segment.centerline.front_pose();
}

Polyline trim_polyline_ahead(
  const Polyline & line, const Pose2D & robot, const double prune_distance_m)
{
  if (line.size() < 2U) {
    return line;
  }

  double best_dist_sq = std::numeric_limits<double>::infinity();
  std::size_t best_seg = 0U;
  double best_t = 0.0;

  for (std::size_t i = 0U; i + 1U < line.points.size(); ++i) {
    const auto & a = line.points[i];
    const auto & b = line.points[i + 1U];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len_sq = vx * vx + vy * vy;
    double t = 0.0;
    if (len_sq > 1e-12) {
      t = ((robot.x - a.x) * vx + (robot.y - a.y) * vy) / len_sq;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double px = a.x + t * vx;
    const double py = a.y + t * vy;
    const double dx = px - robot.x;
    const double dy = py - robot.y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_seg = i;
      best_t = t;
    }
  }

  const auto & seg_a = line.points[best_seg];
  const auto & seg_b = line.points[best_seg + 1U];
  Pose2D start{
    seg_a.x + best_t * (seg_b.x - seg_a.x),
    seg_a.y + best_t * (seg_b.y - seg_a.y),
    seg_a.yaw + best_t * (seg_b.yaw - seg_a.yaw)};

  std::size_t append_from = best_seg + 1U;
  if (best_t > 1.0 - 1e-6 && append_from < line.points.size()) {
    start = line.points[append_from];
    ++append_from;
  } else if (best_t < 1e-6 && best_dist_sq <= prune_distance_m * prune_distance_m &&
    best_seg > 0U)
  {
    start = seg_b;
    append_from = best_seg + 1U;
  }

  Polyline trimmed;
  trimmed.points.push_back(start);
  for (std::size_t i = append_from; i < line.points.size(); ++i) {
    if (trimmed.points.back().x == line.points[i].x &&
      trimmed.points.back().y == line.points[i].y)
    {
      continue;
    }
    trimmed.points.push_back(line.points[i]);
  }

  if (trimmed.size() < 2U) {
    return {};
  }
  return trimmed;
}

namespace
{
double point_to_polyline_distance(const Pose2D & p, const Polyline & line)
{
  if (line.size() < 2U) {
    return line.empty() ? std::numeric_limits<double>::infinity() :
           p.distance_to(line.front_pose());
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0U; i + 1U < line.size(); ++i) {
    const auto & a = line.points[i];
    const auto & b = line.points[i + 1U];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len_sq = vx * vx + vy * vy;
    double t = 0.0;
    if (len_sq > 1e-12) {
      t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len_sq;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double px = a.x + t * vx;
    const double py = a.y + t * vy;
    best = std::min(best, std::hypot(p.x - px, p.y - py));
  }
  return best;
}
}  // namespace

double polyline_covered_fraction(
  const Polyline & segment, const Polyline & driven, const double tol_m)
{
  if (segment.empty() || driven.size() < 2U || tol_m <= 0.0) {
    return 0.0;
  }
  std::size_t close = 0U;
  for (const auto & p : segment.points) {
    if (point_to_polyline_distance(p, driven) <= tol_m) {
      ++close;
    }
  }
  return static_cast<double>(close) / static_cast<double>(segment.points.size());
}

SegmentId compute_segment_id(
  const Polyline & centerline, std::optional<uint32_t> lane_index,
  const double quantize_m)
{
  std::size_t h = 1469598103934665603ULL;
  const auto mix = [&](std::size_t v) {
      h ^= v;
      h *= 1099511628211ULL;
    };

  if (lane_index.has_value()) {
    mix(static_cast<std::size_t>(lane_index.value()));
  }
  if (!centerline.empty()) {
    const auto q = [&](double v) {
        return static_cast<std::size_t>(std::lround(v / quantize_m));
      };
    mix(q(centerline.points.front().x));
    mix(q(centerline.points.front().y));
    mix(q(centerline.points.back().x));
    mix(q(centerline.points.back().y));
    mix(centerline.size());
  }
  return static_cast<SegmentId>(h);
}

SegmentId make_segment_id(uint32_t lane_index, uint32_t sub_ordinal)
{
  std::size_t h = 1469598103934665603ULL;
  const auto mix = [&](std::size_t v) {
      h ^= v;
      h *= 1099511628211ULL;
    };
  mix(static_cast<std::size_t>(lane_index));
  mix(static_cast<std::size_t>(sub_ordinal));
  return static_cast<SegmentId>(h);
}

SegmentId make_split_segment_id(SegmentId baseline_id, uint32_t split_ordinal)
{
  std::size_t h = 1469598103934665603ULL;
  const auto mix = [&](std::size_t v) {
      h ^= v;
      h *= 1099511628211ULL;
    };
  mix(0x53504C4954ULL);  // "SPLIT"
  mix(static_cast<std::size_t>(baseline_id));
  mix(static_cast<std::size_t>(split_ordinal));
  return static_cast<SegmentId>(h);
}

SegmentId make_oarp_rank_id(uint32_t replan_generation, uint32_t rank_index)
{
  std::size_t h = 1469598103934665603ULL;
  const auto mix = [&](std::size_t v) {
      h ^= v;
      h *= 1099511628211ULL;
    };
  mix(0x4F415250ULL);  // "OARP"
  mix(static_cast<std::size_t>(replan_generation));
  mix(static_cast<std::size_t>(rank_index));
  return static_cast<SegmentId>(h);
}

SegmentId make_pass_job_id(const uint32_t lane_index, const uint32_t pass_ordinal)
{
  std::size_t h = 1469598103934665603ULL;
  const auto mix = [&](std::size_t v) {
      h ^= v;
      h *= 1099511628211ULL;
    };
  mix(0x50415353ULL);  // "PASS"
  mix(static_cast<std::size_t>(lane_index));
  mix(static_cast<std::size_t>(pass_ordinal));
  return static_cast<SegmentId>(h);
}

std::vector<CoverageJob> schedulable_jobs_from_segments(
  const std::vector<CoverageSegment> & segments, const bool schedule_probes)
{
  std::vector<CoverageJob> jobs;
  for (const auto & seg : segments) {
    if (!is_segment_schedulable(seg, schedule_probes)) {
      continue;
    }
    jobs.push_back({seg.id, DriveDirection::Forward});
    jobs.push_back({seg.id, DriveDirection::Reverse});
  }
  return jobs;
}

std::vector<CoverageJob> open_jobs_from_segments(
  const std::vector<CoverageSegment> & segments)
{
  return schedulable_jobs_from_segments(segments, false);
}

}  // namespace gpr_common

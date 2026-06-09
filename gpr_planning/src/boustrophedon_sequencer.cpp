#include "gpr_planning/boustrophedon_sequencer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace gpr_planning
{

namespace
{

bool endpoints_touch(
  const gpr_common::Polyline & a, const gpr_common::Polyline & b, const double gap_m)
{
  if (a.empty() || b.empty()) {
    return false;
  }
  return a.back_pose().distance_to(b.front_pose()) <= gap_m ||
         a.back_pose().distance_to(b.back_pose()) <= gap_m ||
         a.front_pose().distance_to(b.front_pose()) <= gap_m ||
         a.front_pose().distance_to(b.back_pose()) <= gap_m;
}

gpr_common::Polyline reverse_polyline(const gpr_common::Polyline & line)
{
  gpr_common::Polyline out = line;
  std::reverse(out.points.begin(), out.points.end());
  for (auto & p : out.points) {
    p.yaw += M_PI;
  }
  return out;
}
}  // namespace

BoustrophedonSequencer::BoustrophedonSequencer(BoustrophedonSequencerConfig config)
: config_(config)
{}

gpr_common::DriveDirection BoustrophedonSequencer::lane_drive_direction(
  const uint32_t lane_index, const gpr_common::ScanDirection scan_direction) const
{
  (void)scan_direction;
  return (lane_index % 2U == 0U) ? gpr_common::DriveDirection::Forward :
         gpr_common::DriveDirection::Reverse;
}

double BoustrophedonSequencer::segment_lane_key(
  const gpr_common::CoverageSegment & seg,
  const gpr_common::ScanDirection scan_direction,
  const gpr_common::DriveDirection direction) const
{
  const auto line = oriented_centerline(seg, direction);
  if (line.empty()) {
    return 0.0;
  }
  return (scan_direction == gpr_common::ScanDirection::kX) ?
         line.front_pose().y : line.front_pose().x;
}

gpr_common::Polyline BoustrophedonSequencer::oriented_centerline(
  const gpr_common::CoverageSegment & seg,
  const gpr_common::DriveDirection direction) const
{
  return gpr_common::job_polyline(seg, direction);
}

gpr_common::Polyline BoustrophedonSequencer::merge_polylines(
  const gpr_common::Polyline & a, const gpr_common::Polyline & b) const
{
  if (a.empty()) {
    return b;
  }
  if (b.empty()) {
    return a;
  }
  gpr_common::Polyline merged = a;
  if (merged.back_pose().distance_to(b.front_pose()) <= config_.merge_gap_m) {
    merged.points.insert(merged.points.end(), b.points.begin() + 1, b.points.end());
    return merged;
  }
  if (merged.back_pose().distance_to(b.back_pose()) <= config_.merge_gap_m) {
    const auto rev = reverse_polyline(b);
    merged.points.insert(merged.points.end(), rev.points.begin() + 1, rev.points.end());
    return merged;
  }
  if (merged.front_pose().distance_to(b.front_pose()) <= config_.merge_gap_m) {
    gpr_common::Polyline reversed_a = reverse_polyline(a);
    reversed_a.points.insert(reversed_a.points.end(), b.points.begin() + 1, b.points.end());
    return reversed_a;
  }
  merged.points.insert(merged.points.end(), b.points.begin(), b.points.end());
  return merged;
}

gpr_common::DriveDirection BoustrophedonSequencer::prefer_drive_direction(
  const gpr_common::Polyline & centerline, const gpr_common::Pose2D & robot) noexcept
{
  if (centerline.size() < 2U) {
    return gpr_common::DriveDirection::Forward;
  }
  const double df = centerline.front_pose().distance_to(robot);
  const double dr = centerline.back_pose().distance_to(robot);
  return dr < df ? gpr_common::DriveDirection::Reverse :
         gpr_common::DriveDirection::Forward;
}

std::vector<CoveragePass> BoustrophedonSequencer::plan(
  const std::vector<gpr_common::CoverageSegment> & schedulable,
  const BoustrophedonConfig & boustrophedon_config,
  const std::optional<gpr_common::Pose2D> & robot_pose,
  const gpr_common::GridMap * grid,
  const AStarGridPlanner * astar) const
{
  std::map<uint32_t, std::vector<gpr_common::CoverageSegment>> by_lane;
  for (const auto & seg : schedulable) {
    if (!seg.lane_index.has_value() || seg.centerline.size() < 2U) {
      continue;
    }
    by_lane[seg.lane_index.value()].push_back(seg);
  }

  std::vector<CoveragePass> passes;
  for (const auto & [lane_index, lane_segments] : by_lane) {
    const auto direction = lane_drive_direction(lane_index, boustrophedon_config.scan_direction);
    std::vector<gpr_common::CoverageSegment> ordered = lane_segments;
    std::sort(
      ordered.begin(), ordered.end(),
      [&](const gpr_common::CoverageSegment & a, const gpr_common::CoverageSegment & b) {
        return segment_lane_key(a, boustrophedon_config.scan_direction, direction) <
               segment_lane_key(b, boustrophedon_config.scan_direction, direction);
      });

    std::vector<gpr_common::CoverageSegment> run;
    const auto flush_run = [&]() {
        if (run.empty()) {
          return;
        }
        gpr_common::Polyline merged;
        std::vector<gpr_common::SegmentId> ids;
        for (const auto & seg : run) {
          const auto piece = oriented_centerline(seg, direction);
          merged = merge_polylines(merged, piece);
          ids.push_back(seg.id);
        }
        if (merged.size() < 2U || merged.length() < config_.merge_endpoint_epsilon_m) {
          run.clear();
          return;
        }
        CoveragePass pass;
        pass.lane_index = lane_index;
        pass.segment_ids = ids;
        pass.centerline = merged;
        pass.job.direction = direction;
        pass.job.segment_id = gpr_common::make_pass_job_id(
          lane_index, static_cast<uint32_t>(passes.size()));
        pass.job.covers = ids;
        passes.push_back(std::move(pass));
        run.clear();
      };

    for (const auto & seg : ordered) {
      if (run.empty()) {
        run.push_back(seg);
        continue;
      }
      const auto prev_line = oriented_centerline(run.back(), direction);
      const auto next_line = oriented_centerline(seg, direction);
      if (endpoints_touch(prev_line, next_line, config_.merge_gap_m)) {
        run.push_back(seg);
      } else {
        flush_run();
        run.push_back(seg);
      }
    }
    flush_run();
  }

  if (!robot_pose.has_value()) {
    std::sort(
      passes.begin(), passes.end(),
      [&](const CoveragePass & a, const CoveragePass & b) {
        if (a.lane_index != b.lane_index) {
          return a.lane_index < b.lane_index;
        }
        const auto dir = lane_drive_direction(a.lane_index, boustrophedon_config.scan_direction);
        gpr_common::CoverageSegment tmp_a;
        tmp_a.centerline = a.centerline;
        gpr_common::CoverageSegment tmp_b;
        tmp_b.centerline = b.centerline;
        return segment_lane_key(tmp_a, boustrophedon_config.scan_direction, dir) <
               segment_lane_key(tmp_b, boustrophedon_config.scan_direction, dir);
      });
    return passes;
  }

  std::vector<CoveragePass> ordered;
  ordered.reserve(passes.size());
  std::vector<CoveragePass> remaining = passes;
  gpr_common::Pose2D cursor = robot_pose.value();

  while (!remaining.empty()) {
    std::size_t best_idx = 0U;
    gpr_common::DriveDirection best_dir = gpr_common::DriveDirection::Forward;
    double best_cost = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0U; i < remaining.size(); ++i) {
      const auto dir = prefer_drive_direction(remaining[i].centerline, cursor);
      gpr_common::CoverageSegment tmp;
      tmp.centerline = remaining[i].centerline;
      const auto entry = gpr_common::job_entry_pose(tmp, dir);
      double cost = std::hypot(entry.x - cursor.x, entry.y - cursor.y);
      if (grid != nullptr && astar != nullptr && !grid->empty()) {
        cost = astar->transit_cost(*grid, cursor, entry);
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_idx = i;
        best_dir = dir;
      }
    }

    CoveragePass chosen = remaining[best_idx];
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best_idx));
    chosen.job.direction = best_dir;
    ordered.push_back(std::move(chosen));

    gpr_common::CoverageSegment tmp;
    tmp.centerline = ordered.back().centerline;
    cursor = gpr_common::job_exit_pose(tmp, ordered.back().job.direction);
  }

  return ordered;
}

}  // namespace gpr_planning

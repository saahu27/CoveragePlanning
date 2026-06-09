#include "gpr_planning/astar_grid_planner.hpp"
#include "gpr_planning/segment_catalog.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "gpr_common/swath_coverage.hpp"

namespace gpr_planning
{

namespace
{
bool is_schedulable(
  const gpr_common::CoverageSegment & seg, const SegmentCatalogConfig & config) noexcept
{
  return gpr_common::is_segment_schedulable(seg, config.schedule_blocked_probes);
}

bool segment_matches_cover_ids(
  const gpr_common::CoverageSegment & seg,
  const std::vector<gpr_common::SegmentId> & cover_ids)
{
  const auto base = gpr_common::effective_baseline_id(seg);
  for (const auto cid : cover_ids) {
    if (seg.id == cid || base == cid) {
      return true;
    }
    if (seg.baseline_id.has_value() && *seg.baseline_id == cid) {
      return true;
    }
  }
  return false;
}

bool segment_matches_job_scope(
  const gpr_common::CoverageSegment & seg,
  const std::vector<gpr_common::SegmentId> & cover_ids,
  const std::optional<uint32_t> pass_lane_index,
  const bool pass_mode)
{
  (void)pass_lane_index;
  (void)pass_mode;
  return segment_matches_cover_ids(seg, cover_ids);
}

void refresh_segment_blocked(
  gpr_common::CoverageSegment & seg,
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & config)
{
  if (seg.outcome == gpr_common::SegmentOutcome::Completed) {
    seg.blocked = false;
    return;
  }
  if (seg.outcome == gpr_common::SegmentOutcome::PartiallyCompleted) {
    const auto tail = gpr_common::schedulable_centerline(
      seg, gpr_common::DriveDirection::Forward);
    if (tail.size() < 2U) {
      seg.blocked = false;
      return;
    }
    seg.blocked = PathInvalidator::is_blocked(tail, grid, config);
    return;
  }
  if (seg.outcome == gpr_common::SegmentOutcome::Pending && seg.centerline.size() >= 2U) {
    seg.blocked = PathInvalidator::is_blocked(seg.centerline, grid, config);
  }
}
}  // namespace

SegmentCatalog::SegmentCatalog(SegmentCatalogConfig config)
: config_(config)
{}

bool SegmentCatalog::is_open(const gpr_common::CoverageSegment & seg) const noexcept
{
  if (!gpr_common::is_segment_open(seg)) {
    return false;
  }
  if (seg.outcome == gpr_common::SegmentOutcome::PartiallyCompleted) {
    return gpr_common::uncovered_arc_length_m(seg) >= config_.min_split_length_m;
  }
  return true;
}

void SegmentCatalog::initialize(std::vector<gpr_common::CoverageSegment> master)
{
  segments_ = std::move(master);
  split_counter_ = 0U;
  oarp_generation_ = 0U;
  ++revision_;
}

void SegmentCatalog::purge_pending_oarp_ranks()
{
  segments_.erase(
    std::remove_if(
      segments_.begin(), segments_.end(),
      [](const gpr_common::CoverageSegment & seg) {
        return seg.source == gpr_common::SegmentSource::OarpRank &&
               seg.outcome == gpr_common::SegmentOutcome::Pending;
      }),
    segments_.end());
}

void SegmentCatalog::apply_split_update(
  gpr_common::CoverageSegment seg,
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & config,
  std::vector<gpr_common::CoverageSegment> & updated,
  bool & schedulable_changed)
{
  const bool was_schedulable = is_schedulable(seg, config_);

  if (seg.attempted && seg.blocked) {
    updated.push_back(std::move(seg));
    return;
  }

  if (seg.source == gpr_common::SegmentSource::OarpRank) {
    seg.blocked = PathInvalidator::is_blocked(seg.centerline, grid, config);
    if (is_schedulable(seg, config_) != was_schedulable) {
      schedulable_changed = true;
    }
    updated.push_back(std::move(seg));
    return;
  }

  const auto free_parts = PathInvalidator::free_subpolylines(seg.centerline, grid, config);
  const double total_len = seg.centerline.length();
  double free_len = 0.0;
  for (const auto & part : free_parts) {
    free_len += part.length();
  }

  if (free_parts.empty()) {
    seg.blocked = true;
    updated.push_back(std::move(seg));
    if (is_schedulable(updated.back(), config_) != was_schedulable) {
      schedulable_changed = true;
    }
    return;
  }

  if (free_parts.size() == 1U &&
    free_len >= total_len - config_.split_length_tolerance_m)
  {
    seg.centerline = free_parts.front();
    seg.blocked = PathInvalidator::is_blocked(seg.centerline, grid, config);
    updated.push_back(std::move(seg));
    if (is_schedulable(updated.back(), config_) != was_schedulable) {
      schedulable_changed = true;
    }
    return;
  }

  const gpr_common::SegmentId baseline = gpr_common::effective_baseline_id(seg);
  const bool inherited_attempted = seg.attempted;
  const auto lane = seg.lane_index;
  const auto child_source = seg.source == gpr_common::SegmentSource::Baseline ?
    gpr_common::SegmentSource::Split : seg.source;

  const std::size_t before = updated.size();
  for (const auto & part : free_parts) {
    if (part.length() < config_.min_split_length_m) {
      continue;
    }
    gpr_common::CoverageSegment child;
    child.centerline = part;
    child.lane_index = lane;
    child.baseline_id = baseline;
    child.id = gpr_common::make_split_segment_id(baseline, split_counter_++);
    child.source = child_source;
    child.blocked = PathInvalidator::is_blocked(child.centerline, grid, config);
    child.attempted = inherited_attempted && seg.blocked;
    updated.push_back(std::move(child));
  }

  if (updated.size() == before) {
    seg.blocked = true;
    updated.push_back(std::move(seg));
  }

  schedulable_changed = true;
}

void SegmentCatalog::update_partial_segment(
  gpr_common::CoverageSegment & seg,
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & config,
  bool & schedulable_changed)
{
  const bool was_schedulable = is_schedulable(seg, config_);
  const auto tail = gpr_common::schedulable_centerline(
    seg, gpr_common::DriveDirection::Forward);
  if (tail.size() < 2U) {
    seg.blocked = false;
  } else {
    seg.blocked = PathInvalidator::is_blocked(tail, grid, config);
  }
  if (is_schedulable(seg, config_) != was_schedulable) {
    schedulable_changed = true;
  }
}

void SegmentCatalog::update_blocked(
  const gpr_common::GridMap & grid, const PathInvalidatorConfig & config)
{
  if (grid.empty()) {
    return;
  }

  std::vector<gpr_common::CoverageSegment> updated;
  updated.reserve(segments_.size());
  bool schedulable_changed = false;

  for (auto & seg : segments_) {
    if (seg.outcome == gpr_common::SegmentOutcome::PartiallyCompleted) {
      update_partial_segment(seg, grid, config, schedulable_changed);
      updated.push_back(std::move(seg));
      continue;
    }
    if (seg.outcome != gpr_common::SegmentOutcome::Pending) {
      updated.push_back(std::move(seg));
      continue;
    }
    apply_split_update(std::move(seg), grid, config, updated, schedulable_changed);
  }

  segments_ = std::move(updated);
  if (schedulable_changed) {
    ++revision_;
  }
}

void SegmentCatalog::refresh_oarp_ranks(
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & inv_config,
  const OarpLitePlanner & oarp_planner,
  const std::optional<gpr_common::Pose2D> & robot_pose,
  const AStarGridPlanner * astar)
{
  if (!oarp_planner.config().enabled || grid.empty()) {
    return;
  }
  if (!schedulable_segments().empty()) {
    return;
  }
  if (oarp_generation_ >= oarp_planner.config().max_replan_generations) {
    return;
  }

  purge_pending_oarp_ranks();
  const auto next_generation = oarp_generation_ + 1U;
  const auto ranks = oarp_planner.generate_ranks(
    grid, inv_config, segments_, next_generation);
  if (ranks.empty()) {
    return;
  }

  if (robot_pose.has_value() && astar != nullptr) {
    const auto & first = ranks.front();
    if (first.centerline.size() >= 2U) {
      const auto entry = first.centerline.front_pose();
      const auto path = astar->plan_path(grid, *robot_pose, entry);
      if (!path.has_value() || path->size() < 2U) {
        return;
      }
    }
  }

  oarp_generation_ = next_generation;
  segments_.insert(segments_.end(), ranks.begin(), ranks.end());
  ++revision_;
}

void SegmentCatalog::mark_completed(const gpr_common::SegmentId id)
{
  for (auto & seg : segments_) {
    if (seg.id == id) {
      seg.outcome = gpr_common::SegmentOutcome::Completed;
      seg.blocked = false;
    }
  }
  ++revision_;
}

void SegmentCatalog::mark_completed_for_job(
  const std::vector<gpr_common::SegmentId> & cover_ids,
  const gpr_common::Polyline & driven_path,
  const double overlap_tol_m,
  const double min_overlap_fraction)
{
  gpr_common::SwathCoverageConfig config;
  config.lateral_max_m = overlap_tol_m;
  config.min_complete_fraction = min_overlap_fraction;
  config.min_partial_fraction = min_overlap_fraction;
  config.partial_enabled = false;
  apply_swath_coverage_for_job(cover_ids, driven_path, config, nullptr);
}

namespace
{
void apply_coverage_result_to_segment(
  gpr_common::CoverageSegment & seg,
  const gpr_common::SwathCoverageResult & measured,
  const gpr_common::SwathCoverageConfig & config,
  const gpr_common::GridMap * grid,
  const PathInvalidatorConfig * inv_config)
{
  std::vector<std::pair<double, double>> merged = measured.covered_intervals_m;
  if (!seg.covered_intervals_m.empty()) {
    merged.insert(merged.end(), seg.covered_intervals_m.begin(), seg.covered_intervals_m.end());
    merged = gpr_common::merge_intervals(std::move(merged));
  }
  seg.covered_intervals_m = merged;
  seg.last_mean_lateral_error_m = measured.mean_lateral_error_m;
  seg.last_max_lateral_error_m = measured.max_lateral_error_m;

  const double total = gpr_common::polyline_arc_length(seg.centerline);
  const double frac = total > 1e-9 ?
    gpr_common::intervals_union_length(merged) / total : 0.0;
  if (frac >= config.min_complete_fraction) {
    seg.outcome = gpr_common::SegmentOutcome::Completed;
    seg.blocked = false;
  } else if (frac > 1e-6 &&
    (config.partial_enabled && frac >= config.min_partial_fraction))
  {
    seg.outcome = gpr_common::SegmentOutcome::PartiallyCompleted;
  } else if (frac > 1e-6) {
    // Coverage progress below partial threshold: freeze as partial, do not re-queue.
    seg.outcome = gpr_common::SegmentOutcome::PartiallyCompleted;
  }

  if (grid != nullptr && inv_config != nullptr && !grid->empty()) {
    refresh_segment_blocked(seg, *grid, *inv_config);
  }
}
}  // namespace

void SegmentCatalog::apply_swath_coverage_for_job(
  const std::vector<gpr_common::SegmentId> & cover_ids,
  const gpr_common::Polyline & coverage_trace,
  const gpr_common::SwathCoverageConfig & config,
  const gpr_common::Polyline * pass_centerline,
  const gpr_common::DriveDirection drive_direction,
  const std::optional<uint32_t> pass_lane_index,
  const gpr_common::GridMap * grid,
  const PathInvalidatorConfig * inv_config)
{
  if (cover_ids.empty() || coverage_trace.size() < 2U) {
    return;
  }

  std::optional<gpr_common::SwathCoverageResult> pass_measured;
  if (pass_centerline != nullptr && pass_centerline->size() >= 2U) {
    pass_measured = gpr_common::compute_swath_coverage(
      *pass_centerline, coverage_trace, config);
    for (auto & seg : segments_) {
      if (seg.outcome == gpr_common::SegmentOutcome::Skipped) {
        continue;
      }
      if (!segment_matches_job_scope(seg, cover_ids, pass_lane_index, true)) {
        continue;
      }
      const auto measured = gpr_common::coverage_for_segment_from_pass(
        seg.centerline, *pass_centerline, *pass_measured);
      if (pass_measured->covered_fraction >= config.min_complete_fraction) {
        const double total = gpr_common::polyline_arc_length(seg.centerline);
        if (total <= 1e-9) {
          continue;
        }
        seg.outcome = gpr_common::SegmentOutcome::Completed;
        seg.blocked = false;
        seg.covered_intervals_m = {{0.0, total}};
        seg.last_mean_lateral_error_m = pass_measured->mean_lateral_error_m;
        seg.last_max_lateral_error_m = pass_measured->max_lateral_error_m;
      } else if (measured.covered_fraction <= 1e-6) {
        continue;
      } else {
        apply_coverage_result_to_segment(seg, measured, config, grid, inv_config);
      }
    }
    ++revision_;
    return;
  }

  for (auto & seg : segments_) {
    if (seg.outcome == gpr_common::SegmentOutcome::Skipped ||
      seg.outcome == gpr_common::SegmentOutcome::Completed)
    {
      continue;
    }
    if (!segment_matches_job_scope(seg, cover_ids, pass_lane_index, false)) {
      continue;
    }
    (void)drive_direction;
    const auto measured = gpr_common::compute_swath_coverage(
      seg.centerline, coverage_trace, config);
    apply_coverage_result_to_segment(seg, measured, config, grid, inv_config);
  }
  ++revision_;
}

void SegmentCatalog::mark_job_skipped(const gpr_common::SegmentId id)
{
  for (auto & seg : segments_) {
    if (seg.id == id) {
      seg.outcome = gpr_common::SegmentOutcome::Skipped;
      seg.blocked = false;
    }
  }
  ++revision_;
}

void SegmentCatalog::mark_blocked(const gpr_common::SegmentId id)
{
  for (auto & seg : segments_) {
    if (seg.id == id && seg.outcome == gpr_common::SegmentOutcome::Pending) {
      seg.blocked = true;
    }
  }
  ++revision_;
}

void SegmentCatalog::mark_blocked_for_job(
  const std::vector<gpr_common::SegmentId> & cover_ids)
{
  for (auto & seg : segments_) {
    if (seg.outcome != gpr_common::SegmentOutcome::Pending) {
      continue;
    }
    if (segment_matches_cover_ids(seg, cover_ids)) {
      seg.blocked = true;
    }
  }
  ++revision_;
}

void SegmentCatalog::mark_attempted(const gpr_common::SegmentId id)
{
  for (auto & seg : segments_) {
    if (seg.id == id) {
      seg.attempted = true;
    }
  }
}

void SegmentCatalog::mark_attempted_for_job(
  const std::vector<gpr_common::SegmentId> & cover_ids)
{
  for (auto & seg : segments_) {
    if (segment_matches_cover_ids(seg, cover_ids)) {
      seg.attempted = true;
    }
  }
  ++revision_;
}

std::vector<gpr_common::CoverageSegment> SegmentCatalog::open_segments() const
{
  std::vector<gpr_common::CoverageSegment> open;
  for (const auto & seg : segments_) {
    if (is_open(seg)) {
      open.push_back(seg);
    }
  }
  return open;
}

std::vector<gpr_common::CoverageSegment> SegmentCatalog::schedulable_segments() const
{
  std::vector<gpr_common::CoverageSegment> work;
  for (const auto & seg : segments_) {
    if (is_schedulable(seg, config_)) {
      work.push_back(seg);
    }
  }
  return work;
}

bool SegmentCatalog::has_schedulable_work(
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & inv_config,
  const OarpLitePlanner & oarp_planner) const
{
  if (!schedulable_segments().empty()) {
    return true;
  }
  if (!oarp_planner.config().enabled ||
    oarp_generation_ >= oarp_planner.config().max_replan_generations)
  {
    return false;
  }
  return oarp_planner.has_uncovered_ranks(grid, inv_config, segments_);
}

const gpr_common::CoverageSegment * SegmentCatalog::find(const gpr_common::SegmentId id) const
{
  for (const auto & seg : segments_) {
    if (seg.id == id) {
      return &seg;
    }
  }
  return nullptr;
}

namespace
{
bool segment_has_schedulable_work(
  const gpr_common::CoverageSegment & seg,
  const SegmentCatalogConfig & config) noexcept
{
  return gpr_common::is_segment_schedulable(seg, config.schedule_blocked_probes);
}

bool segment_id_actionable(
  const gpr_common::SegmentId id, const SegmentCatalog & catalog)
{
  if (const auto * seg = catalog.find(id)) {
    return segment_has_schedulable_work(*seg, catalog.config());
  }
  for (const auto & seg : catalog.segments()) {
    if (!segment_has_schedulable_work(seg, catalog.config())) {
      continue;
    }
    if (seg.id == id || gpr_common::effective_baseline_id(seg) == id) {
      return true;
    }
    if (seg.baseline_id.has_value() && *seg.baseline_id == id) {
      return true;
    }
  }
  return false;
}
}  // namespace

bool job_still_actionable(
  const gpr_common::CoverageJob & job, const SegmentCatalog & catalog)
{
  std::vector<gpr_common::SegmentId> ids = job.covers.empty() ?
    std::vector<gpr_common::SegmentId>{job.segment_id} : job.covers;
  if (!job.covers.empty() &&
    std::find(ids.begin(), ids.end(), job.segment_id) == ids.end())
  {
    ids.push_back(job.segment_id);
  }
  for (const auto id : ids) {
    if (segment_id_actionable(id, catalog)) {
      return true;
    }
  }
  return false;
}

}  // namespace gpr_planning

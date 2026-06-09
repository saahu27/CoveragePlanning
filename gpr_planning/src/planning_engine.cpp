#include "gpr_planning/planning_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include "gpr_planning/planning_log.hpp"
#include "gpr_planning/sequence_solver_utils.hpp"

#include <sstream>

namespace gpr_planning
{

namespace
{
bool job_overlaps(
  const gpr_common::CoverageJob & held, const gpr_common::CoverageJob & candidate)
{
  if (held.segment_id == candidate.segment_id) {
    return true;
  }
  const auto references = [](const gpr_common::SegmentId id,
    const gpr_common::CoverageJob & job) {
      if (job.segment_id == id) {
        return true;
      }
      return std::find(job.covers.begin(), job.covers.end(), id) != job.covers.end();
    };
  if (references(held.segment_id, candidate)) {
    return true;
  }
  for (const auto id : held.covers) {
    if (references(id, candidate)) {
      return true;
    }
  }
  return false;
}

gpr_common::CoverageSegment pass_to_schedulable_segment(const CoveragePass & pass)
{
  gpr_common::CoverageSegment seg;
  seg.centerline = pass.centerline;
  seg.id = pass.job.segment_id;
  seg.lane_index = pass.lane_index;
  seg.outcome = gpr_common::SegmentOutcome::Pending;
  return seg;
}

bool segment_in_passes(
  const gpr_common::SegmentId id, const std::vector<CoveragePass> & passes)
{
  for (const auto & pass : passes) {
    if (std::find(pass.segment_ids.begin(), pass.segment_ids.end(), id) !=
      pass.segment_ids.end())
    {
      return true;
    }
  }
  return false;
}

void attach_pass_metadata(
  std::vector<gpr_common::CoverageJob> & schedule,
  const std::vector<CoveragePass> & passes)
{
  for (auto & job : schedule) {
    for (const auto & pass : passes) {
      if (pass.job.segment_id != job.segment_id) {
        continue;
      }
      job.covers = pass.segment_ids;
      break;
    }
  }
}

SequenceSolverOptions build_solver_options(
  const std::vector<CoveragePass> & passes,
  const std::optional<gpr_common::Pose2D> & route_end_pose,
  std::unordered_map<gpr_common::SegmentId, gpr_common::DriveDirection> & fixed_directions)
{
  fixed_directions.clear();
  for (const auto & pass : passes) {
    fixed_directions[pass.job.segment_id] = pass.job.direction;
  }
  SequenceSolverOptions options;
  options.fixed_directions = fixed_directions.empty() ? nullptr : &fixed_directions;
  options.route_end_pose = route_end_pose;
  return options;
}

std::vector<gpr_common::CoverageSegment> build_atsp_job_pool(
  const std::vector<CoveragePass> & passes,
  const std::vector<gpr_common::CoverageSegment> & schedulable)
{
  std::vector<gpr_common::CoverageSegment> pool;
  pool.reserve(passes.size() + schedulable.size());
  for (const auto & pass : passes) {
    pool.push_back(pass_to_schedulable_segment(pass));
  }
  for (const auto & seg : schedulable) {
    if (!seg.lane_index.has_value() || !segment_in_passes(seg.id, passes)) {
      pool.push_back(seg);
    }
  }
  return pool;
}

std::vector<gpr_common::CoverageSegment> collect_unassigned_segments(
  const std::vector<gpr_common::CoverageSegment> & schedulable,
  const std::vector<CoveragePass> & passes)
{
  std::vector<gpr_common::CoverageSegment> unassigned;
  for (const auto & seg : schedulable) {
    if (!segment_in_passes(seg.id, passes)) {
      unassigned.push_back(seg);
    }
  }
  return unassigned;
}

void populate_pass_index_map(
  std::unordered_map<gpr_common::SegmentId, std::size_t> & index,
  const std::vector<CoveragePass> & passes)
{
  index.clear();
  for (std::size_t i = 0; i < passes.size(); ++i) {
    index[passes[i].job.segment_id] = i;
  }
}
}  // namespace

PlanningEngine::PlanningEngine(
  BoustrophedonConfig boustrophedon_config,
  PathInvalidatorConfig invalidator_config,
  AStarConfig astar_config,
  std::string frame_id,
  const AtspSolverConfig atsp_config,
  SegmentCatalogConfig catalog_config,
  OarpLiteConfig oarp_config,
  const bool use_boustrophedon_sequencer,
  BoustrophedonSequencerConfig boustrophedon_sequencer_config,
  const bool show_completed_in_markers,
  const bool require_reachable_transit,
  gpr_common::SwathCoverageConfig swath_coverage_config,
  const double transit_skip_distance,
  const double executed_trace_step_m,
  const double flush_pose_min_step_m,
  const PlanningDiagnosticsConfig diagnostics_config,
  const gpr_metrics::MetricsConfig metrics_config)
: planner_(std::move(boustrophedon_config)),
  invalidator_config_(invalidator_config),
  astar_config_(astar_config),
  astar_(astar_config_),
  solver_(create_sequence_solver(atsp_config)),
  frame_id_(std::move(frame_id)),
  catalog_(catalog_config),
  oarp_planner_(planner_.config(), oarp_config),
  boustrophedon_sequencer_(std::move(boustrophedon_sequencer_config)),
  use_boustrophedon_sequencer_(use_boustrophedon_sequencer),
  show_completed_in_markers_(show_completed_in_markers),
  require_reachable_transit_(require_reachable_transit),
  swath_coverage_config_(std::move(swath_coverage_config)),
  transit_skip_distance_(transit_skip_distance),
  executed_trace_step_m_(executed_trace_step_m),
  flush_pose_min_step_m_(flush_pose_min_step_m),
  diagnostics_config_(diagnostics_config),
  metrics_config_(metrics_config)
{
  reporter_.configure(
    planner_.config().region.bounds,
    planner_.config().coverage_inset,
    planner_.config().lane_spacing,
    metrics_config_);
  schedule_.reserve(64);
  scheduled_passes_.reserve(32);
}

void PlanningEngine::set_log(PlanningLog log) noexcept
{
  log_ = std::move(log);
}

const gpr_common::GridMap & PlanningEngine::latest_grid() const noexcept
{
  static const gpr_common::GridMap kEmpty;
  return latest_grid_ ? *latest_grid_ : kEmpty;
}

void PlanningEngine::set_viz_refresh_callback(std::function<void()> callback)
{
  viz_refresh_callback_ = std::move(callback);
}

void PlanningEngine::notify_viz_refresh()
{
  if (viz_refresh_callback_) {
    viz_refresh_callback_();
  }
}

void PlanningEngine::generate_initial_coverage()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    mission_executed_trace_ = {};
  }
  initial_path_ = planner_.generate();
  const auto master = planner_.generate_segments();
  catalog_.initialize(master);
  last_empty_schedule_catalog_rev_ = 0U;
  reporter_.record_baseline(master);
  {
    std::ostringstream oss;
    oss << "generate_initial_coverage: " << initial_path_.size()
        << " waypoints, " << catalog_.segments().size()
        << " coverage segments, "
        << reporter_.baseline().planned_swath_length_m << " m baseline swath";
    planning_log_info(log_, oss.str());
  }
}

void PlanningEngine::update_from_grid(const gpr_common::GridMapConstPtr grid)
{
  if (!grid) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++grid_seq_;
  latest_grid_ = grid;
  last_catalog_revision_ = catalog_.revision();
  catalog_.update_blocked(*grid, invalidator_config_);
  const std::size_t open_after = catalog_.open_segments().size();
  const std::size_t sched_after = catalog_.schedulable_segments().size();
  if (catalog_.catalog_changed_since(last_catalog_revision_)) {
    catalog_changed_ = true;
    last_reachability_valid_ = false;
  }
  if (!grid_sync_logged_ && !grid->empty()) {
    grid_sync_logged_ = true;
    std::size_t blocked_pending = 0;
    std::size_t probe_pending = 0;
    for (const auto & seg : catalog_.segments()) {
      if (seg.outcome == gpr_common::SegmentOutcome::Pending && seg.blocked) {
        ++blocked_pending;
        if (!seg.attempted) {
          ++probe_pending;
        }
      }
    }
    {
      std::ostringstream oss;
      oss << "first grid sync: " << open_after << " open, " << sched_after
          << " schedulable, " << blocked_pending << " blocked (pending), "
          << probe_pending << " probe-eligible, " << catalog_.segments().size()
          << " total segments (boundary_ignore_margin="
          << invalidator_config_.boundary_ignore_margin << " m)";
      planning_log_info(log_, oss.str());
    }
    const auto warn_threshold = static_cast<std::size_t>(std::ceil(
        static_cast<double>(catalog_.segments().size()) *
        diagnostics_config_.first_grid_blocked_warn_fraction));
    if (blocked_pending > warn_threshold) {
      planning_log_warn(
        log_,
        "Over half of segments blocked on first map update — check inflation, "
        "boundary_ignore_margin, and interior obstacles in RViz (/local_occupancy_grid).");
    }
  }
  {
    std::ostringstream oss;
    oss << "update_from_grid: " << catalog_.open_segments().size() << " open, "
        << catalog_.schedulable_segments().size() << " schedulable, "
        << catalog_.segments().size() << " total segments, changed="
        << static_cast<int>(catalog_changed_);
    planning_log_info_throttle(
      log_, last_update_grid_log_ns_,
      static_cast<std::int64_t>(diagnostics_config_.update_grid_log_throttle_ms),
      oss.str());
  }
  notify_viz_refresh();
}

void PlanningEngine::apply_snapshot_grid(
  const gpr_common::GridMapConstPtr grid, const std::uint64_t grid_seq)
{
  if (!grid) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_grid_ = std::move(grid);
  grid_seq_ = grid_seq;
}

bool PlanningEngine::recompute_schedule(const gpr_common::Pose2D & robot_pose)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto held_job = current_job_;
  const gpr_common::GridMap * const grid_ptr = latest_grid_ ? latest_grid_.get() : nullptr;
  catalog_.refresh_oarp_ranks(
    latest_grid(), invalidator_config_, oarp_planner_, robot_pose, &astar_);
  const auto schedulable = catalog_.schedulable_segments();
  {
    std::ostringstream oss;
    oss << "recompute_schedule: planning " << schedulable.size() << " schedulable segments...";
    planning_log_info(log_, oss.str());
  }
  const auto t0 = std::chrono::steady_clock::now();

  scheduled_passes_.clear();
  pass_index_by_job_id_.clear();
  schedule_.clear();

  // Shared pass merge (lane grouping); ordering differs by sequencer mode.
  const auto merged_passes = boustrophedon_sequencer_.plan(
    schedulable, planner_.config(), std::nullopt);

  if (use_boustrophedon_sequencer_) {
    scheduled_passes_ = boustrophedon_sequencer_.plan(
      schedulable, planner_.config(), robot_pose, grid_ptr, &astar_);
    for (const auto & pass : scheduled_passes_) {
      schedule_.push_back(pass.job);
    }
    const auto unassigned = collect_unassigned_segments(schedulable, scheduled_passes_);
    if (!unassigned.empty()) {
      std::unordered_map<gpr_common::SegmentId, gpr_common::DriveDirection> fixed_directions;
      const auto options = build_solver_options(
        scheduled_passes_, route_end_pose_, fixed_directions);
      const auto fallback = solver_->solve(
        robot_pose, unassigned, latest_grid(), astar_, options);
      schedule_.insert(schedule_.end(), fallback.begin(), fallback.end());
    }
  } else {
    scheduled_passes_ = merged_passes;
    const auto atsp_segments = build_atsp_job_pool(scheduled_passes_, schedulable);
    std::unordered_map<gpr_common::SegmentId, gpr_common::DriveDirection> fixed_directions;
    const auto options = build_solver_options(
      scheduled_passes_, route_end_pose_, fixed_directions);
    schedule_ = solver_->solve(robot_pose, atsp_segments, latest_grid(), astar_, options);
    attach_pass_metadata(schedule_, scheduled_passes_);
  }
  populate_pass_index_map(pass_index_by_job_id_, scheduled_passes_);

  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  ++schedule_revision_;
  if (held_job.has_value()) {
    current_job_ = held_job;
    schedule_.erase(
      std::remove_if(
        schedule_.begin(), schedule_.end(),
        [&held_job](const gpr_common::CoverageJob & job) {
          return job_overlaps(*held_job, job);
        }),
      schedule_.end());
  } else {
    current_job_.reset();
  }

  if (require_reachable_transit_) {
    filter_schedule_by_reachability(robot_pose, transit_skip_distance_);
  }
  update_reachability_state(robot_pose, schedulable);
  record_idle_schedule_state();

  {
    std::ostringstream oss;
    oss << "recompute_schedule: " << schedulable.size() << " schedulable -> "
        << schedule_.size() << " jobs (" << scheduled_passes_.size()
        << " merged passes) in " << static_cast<long>(ms) << " ms";
    planning_log_debug(log_, oss.str());
  }
  return !schedule_.empty();
}

const CoveragePass * PlanningEngine::find_pass(const gpr_common::SegmentId job_id) const
{
  const auto it = pass_index_by_job_id_.find(job_id);
  if (it == pass_index_by_job_id_.end() || it->second >= scheduled_passes_.size()) {
    return nullptr;
  }
  return &scheduled_passes_[it->second];
}

const CoveragePass * PlanningEngine::find_pass_for_job(
  const gpr_common::CoverageJob & job) const
{
  if (const auto * direct = find_pass(job.segment_id)) {
    return direct;
  }
  const auto references_pass = [&](const gpr_common::SegmentId id,
    const CoveragePass & pass) {
      if (pass.job.segment_id == id) {
        return true;
      }
      return std::find(pass.segment_ids.begin(), pass.segment_ids.end(), id) !=
             pass.segment_ids.end();
    };
  for (const auto & pass : scheduled_passes_) {
    if (references_pass(job.segment_id, pass)) {
      return &pass;
    }
    for (const auto id : job.covers) {
      if (references_pass(id, pass)) {
        return &pass;
      }
    }
  }
  return nullptr;
}

namespace
{
std::vector<gpr_common::SegmentId> cover_ids_for_job(
  const gpr_common::CoverageJob & job, const CoveragePass * pass)
{
  if (pass != nullptr && !pass->segment_ids.empty()) {
    return pass->segment_ids;
  }
  if (!job.covers.empty()) {
    return job.covers;
  }
  return {job.segment_id};
}
}  // namespace

bool PlanningEngine::has_pending_jobs() const noexcept
{
  return current_job_.has_value() || !schedule_.empty();
}

bool PlanningEngine::has_work_remaining() const noexcept
{
  if (has_pending_jobs()) {
    return true;
  }
  const auto schedulable = catalog_.schedulable_segments();
  if (!schedulable.empty()) {
    if (last_reachability_valid_ &&
      catalog_.revision() == last_reachability_catalog_rev_ &&
      !last_had_reachable_jobs_)
    {
      return false;
    }
    return true;
  }
  if (!catalog_.has_schedulable_work(
      latest_grid(), invalidator_config_, oarp_planner_))
  {
    return false;
  }
  // Allow one replan attempt per catalog revision when OARP might inject ranks.
  return catalog_.revision() != last_empty_schedule_catalog_rev_;
}

bool PlanningEngine::reachability_exhausted() const noexcept
{
  return last_reachability_valid_ &&
         catalog_.revision() == last_reachability_catalog_rev_ &&
         !last_had_reachable_jobs_ &&
         !catalog_.schedulable_segments().empty();
}

bool PlanningEngine::schedule_needs_rebuild() const noexcept
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto job_still_valid = [this](const gpr_common::CoverageJob & job) {
      return job_still_actionable(job, catalog_);
    };

  if (current_job_.has_value() && !job_still_valid(*current_job_)) {
    return true;
  }
  for (const auto & job : schedule_) {
    if (!job_still_valid(job)) {
      return true;
    }
  }
  if (current_job_.has_value() || !schedule_.empty()) {
    return false;
  }
  if (!catalog_.schedulable_segments().empty()) {
    return true;
  }
  if (!catalog_.has_schedulable_work(
      latest_grid(), invalidator_config_, oarp_planner_))
  {
    return false;
  }
  return catalog_.revision() != last_empty_schedule_catalog_rev_;
}

std::optional<gpr_common::CoverageJob> PlanningEngine::pop_next_job(
  const std::optional<gpr_common::Pose2D> & robot_pose,
  const double transit_skip_distance)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  while (!current_job_.has_value() && !schedule_.empty()) {
    const auto candidate = schedule_.front();
    if (require_reachable_transit_ && robot_pose.has_value() &&
      !is_job_reachable(candidate, *robot_pose, transit_skip_distance))
    {
      {
        std::ostringstream oss;
        oss << "pop_next_job: deferring unreachable job (segment "
            << static_cast<unsigned long>(candidate.segment_id) << ")";
        planning_log_info(log_, oss.str());
      }
      schedule_.erase(schedule_.begin());
      catalog_changed_ = true;
      continue;
    }
    current_job_ = candidate;
    schedule_.erase(schedule_.begin());
    // Already holding state_mutex_; do not call clear_executed_trace().
    executed_trace_ = {};
    executed_coverage_trace_ = {};
    executed_transit_trace_ = {};
  }
  return current_job_;
}

gpr_common::Polyline PlanningEngine::current_job_polyline() const
{
  if (!current_job_.has_value()) {
    return {};
  }
  if (const auto * pass = find_pass_for_job(*current_job_)) {
    gpr_common::CoverageSegment tmp;
    tmp.centerline = pass->centerline;
    return gpr_common::job_polyline(tmp, current_job_->direction);
  }
  const auto * seg = catalog_.find(current_job_->segment_id);
  if (seg == nullptr) {
    return {};
  }
  return gpr_common::job_polyline(*seg, current_job_->direction);
}

gpr_common::Polyline PlanningEngine::current_job_polyline_for_robot(
  const gpr_common::Pose2D & robot, const double prune_distance_m) const
{
  return gpr_common::trim_polyline_ahead(current_job_polyline(), robot, prune_distance_m);
}

gpr_common::Polyline PlanningEngine::current_job_followable_polyline_for_robot(
  const gpr_common::Pose2D & robot, const double prune_distance_m) const
{
  const auto trimmed = current_job_polyline_for_robot(robot, prune_distance_m);
  if (trimmed.size() < 2U || latest_grid().empty()) {
    return trimmed;
  }
  const auto free_parts = PathInvalidator::free_subpolylines(
    trimmed, latest_grid(), invalidator_config_);
  if (free_parts.empty()) {
    return {};
  }
  const auto * best = &free_parts.front();
  for (const auto & part : free_parts) {
    if (part.length() > best->length()) {
      best = &part;
    }
  }
  return *best;
}

std::optional<gpr_common::Polyline> PlanningEngine::plan_astar_path(
  const gpr_common::Pose2D & from, const gpr_common::Pose2D & to) const
{
  if (latest_grid().empty()) {
    return std::nullopt;
  }
  return astar_.plan_path(latest_grid(), from, to);
}

bool PlanningEngine::is_transit_path_followable(const gpr_common::Polyline & path) const
{
  if (path.size() < 2U || latest_grid().empty()) {
    return false;
  }
  return !PathInvalidator::is_blocked(path, latest_grid(), invalidator_config_);
}

bool PlanningEngine::is_current_job_followable(
  const gpr_common::Pose2D & robot, const double prune_distance_m) const
{
  return current_job_followable_polyline_for_robot(robot, prune_distance_m).size() >= 2U;
}

bool PlanningEngine::active_job_needs_replan(
  const gpr_common::Pose2D & robot,
  const double prune_distance_m,
  const double transit_skip_distance,
  const bool in_transit_phase) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!current_job_.has_value()) {
    return false;
  }
  if (!job_still_actionable(*current_job_, catalog_)) {
    return true;
  }
  if (in_transit_phase) {
    // Nav2 is already executing the committed transit; replan only if the job died.
    return false;
  }
  return !is_current_job_followable(robot, prune_distance_m);
}

std::optional<gpr_common::Polyline> PlanningEngine::transit_to_current_job(
  const gpr_common::Pose2D & robot_pose) const
{
  if (!current_job_.has_value()) {
    return std::nullopt;
  }
  const auto entry = current_job_entry_pose();
  if (!entry.has_value()) {
    return std::nullopt;
  }
  return plan_astar_path(robot_pose, *entry);
}

std::optional<gpr_common::Pose2D> PlanningEngine::current_job_entry_pose() const
{
  if (!current_job_.has_value()) {
    return std::nullopt;
  }
  return job_entry_pose(*current_job_);
}

std::optional<gpr_common::Pose2D> PlanningEngine::job_entry_pose(
  const gpr_common::CoverageJob & job) const
{
  if (const auto * pass = find_pass_for_job(job)) {
    gpr_common::CoverageSegment tmp;
    tmp.centerline = pass->centerline;
    return gpr_common::job_entry_pose(tmp, job.direction);
  }
  const auto * seg = catalog_.find(job.segment_id);
  if (seg == nullptr) {
    return std::nullopt;
  }
  return gpr_common::job_entry_pose(*seg, job.direction);
}

bool PlanningEngine::job_is_followable(
  const gpr_common::CoverageJob & job,
  const gpr_common::Pose2D & robot,
  const double prune_distance_m) const
{
  gpr_common::Polyline line;
  if (const auto * pass = find_pass_for_job(job)) {
    gpr_common::CoverageSegment tmp;
    tmp.centerline = pass->centerline;
    line = gpr_common::job_polyline(tmp, job.direction);
  } else {
    const auto * seg = catalog_.find(job.segment_id);
    if (seg == nullptr) {
      return false;
    }
    line = gpr_common::job_polyline(*seg, job.direction);
  }
  const auto trimmed = gpr_common::trim_polyline_ahead(line, robot, prune_distance_m);
  if (trimmed.size() < 2U || latest_grid().empty()) {
    return trimmed.size() >= 2U;
  }
  const auto free_parts = PathInvalidator::free_subpolylines(
    trimmed, latest_grid(), invalidator_config_);
  return !free_parts.empty();
}

void PlanningEngine::refresh_transit_display(
  const gpr_common::Pose2D & robot_pose, const double transit_skip_distance)
{
  if (!current_job_.has_value()) {
    clear_active_transit_path();
    return;
  }
  const auto entry_dist = current_job_entry_distance(robot_pose);
  if (entry_dist.has_value() && *entry_dist < transit_skip_distance) {
    clear_active_transit_path();
    return;
  }
  if (const auto transit = transit_to_current_job(robot_pose)) {
    if (transit->size() >= 2U && is_transit_path_followable(*transit)) {
      active_transit_path_ = *transit;
      active_transit_job_id_ = current_job_->segment_id;
      return;
    }
  }
  if (active_transit_job_id_ == current_job_->segment_id &&
    !active_transit_path_.empty() &&
    is_transit_path_followable(active_transit_path_))
  {
    return;
  }
  clear_active_transit_path();
}

void PlanningEngine::set_active_transit_path(const gpr_common::Polyline & path)
{
  active_transit_path_ = path;
  if (path.size() < 2U) {
    active_transit_job_id_.reset();
  } else if (current_job_.has_value()) {
    active_transit_job_id_ = current_job_->segment_id;
  }
}

void PlanningEngine::clear_active_transit_path()
{
  active_transit_path_ = {};
  active_transit_job_id_.reset();
}

std::optional<gpr_common::Polyline> PlanningEngine::plan_transit_path(
  const gpr_common::Pose2D & from, const gpr_common::Pose2D & to) const
{
  return plan_astar_path(from, to);
}

bool PlanningEngine::is_job_reachable(
  const gpr_common::CoverageJob & job,
  const gpr_common::Pose2D & robot_pose,
  const double transit_skip_distance) const
{
  const auto entry = job_entry_pose(job);
  if (!entry.has_value()) {
    return false;
  }
  const double entry_dist = std::hypot(entry->x - robot_pose.x, entry->y - robot_pose.y);
  if (entry_dist < transit_skip_distance) {
    return job_is_followable(job, robot_pose);
  }
  const auto transit = plan_astar_path(robot_pose, *entry);
  return transit.has_value() && transit->size() >= 2U &&
         is_transit_path_followable(*transit);
}

void PlanningEngine::filter_schedule_by_reachability(
  const gpr_common::Pose2D & robot_pose, const double transit_skip_distance)
{
  const std::size_t before = schedule_.size();
  schedule_.erase(
    std::remove_if(
      schedule_.begin(), schedule_.end(),
      [&](const gpr_common::CoverageJob & job) {
        return !is_job_reachable(job, robot_pose, transit_skip_distance);
      }),
    schedule_.end());
  if (schedule_.size() < before) {
    {
      std::ostringstream oss;
      oss << "filter_schedule_by_reachability: deferred "
          << (before - schedule_.size()) << " unreachable jobs";
      planning_log_info(log_, oss.str());
    }
  }
}

void PlanningEngine::block_unreachable_schedulable(
  const gpr_common::Pose2D & robot_pose,
  const std::vector<gpr_common::CoverageSegment> & schedulable)
{
  std::size_t blocked = 0U;
  for (const auto & seg : schedulable) {
    gpr_common::CoverageJob forward_job;
    forward_job.segment_id = seg.id;
    forward_job.direction = gpr_common::DriveDirection::Forward;
    gpr_common::CoverageJob reverse_job = forward_job;
    reverse_job.direction = gpr_common::DriveDirection::Reverse;
    const bool forward_ok = is_job_reachable(
      forward_job, robot_pose, transit_skip_distance_);
    const bool reverse_ok = is_job_reachable(
      reverse_job, robot_pose, transit_skip_distance_);
    if (!forward_ok && !reverse_ok) {
      catalog_.mark_blocked(seg.id);
      ++blocked;
    }
  }
  if (blocked > 0U) {
    {
      std::ostringstream oss;
      oss << "blocked " << blocked
          << " schedulable segments with no reachable transit from robot";
      planning_log_warn(log_, oss.str());
    }
    catalog_changed_ = true;
    notify_viz_refresh();
  }
}

void PlanningEngine::record_idle_schedule_state()
{
  if (current_job_.has_value() || !schedule_.empty()) {
    last_empty_schedule_catalog_rev_ = 0U;
    return;
  }
  if (!catalog_.schedulable_segments().empty()) {
    last_empty_schedule_catalog_rev_ = 0U;
    return;
  }
  last_empty_schedule_catalog_rev_ = catalog_.revision();
  if (catalog_.has_schedulable_work(
      latest_grid(), invalidator_config_, oarp_planner_))
  {
    {
      std::ostringstream oss;
      oss << "recompute_schedule: no executable jobs at catalog rev "
          << static_cast<unsigned long>(catalog_.revision())
          << " — OARP/rank work exists but is unreachable or blocked";
      planning_log_info(log_, oss.str());
    }
  }
}

void PlanningEngine::update_reachability_state(
  const gpr_common::Pose2D & robot_pose,
  const std::vector<gpr_common::CoverageSegment> & schedulable)
{
  last_reachability_valid_ = true;
  last_had_reachable_jobs_ = !schedule_.empty();
  last_reachability_catalog_rev_ = catalog_.revision();

  if (require_reachable_transit_ && schedule_.empty() && !schedulable.empty()) {
    if (current_job_.has_value()) {
      // Mid-transit reachability can fail transiently; keep executing the held job.
      last_had_reachable_jobs_ = true;
      return;
    }
    block_unreachable_schedulable(robot_pose, schedulable);
    last_had_reachable_jobs_ = !schedule_.empty();
    last_reachability_catalog_rev_ = catalog_.revision();
    if (!last_had_reachable_jobs_) {
      {
        std::ostringstream oss;
        oss << "no reachable jobs for " << schedulable.size()
            << " schedulable segments — ending coverage pass";
        planning_log_warn(log_, oss.str());
      }
    }
  }
}

namespace
{
bool append_pose_to_trace(
  gpr_common::Polyline & trace, const gpr_common::Pose2D & pose, const double step_m)
{
  if (trace.empty()) {
    trace.points.push_back(pose);
    return true;
  }
  const auto & last = trace.points.back();
  if (std::hypot(pose.x - last.x, pose.y - last.y) >= step_m) {
    trace.points.push_back(pose);
    return true;
  }
  return false;
}

void force_append_pose_to_trace(
  gpr_common::Polyline & trace, const gpr_common::Pose2D & pose, const double min_step_m)
{
  if (trace.empty()) {
    trace.points.push_back(pose);
    return;
  }
  const auto & last = trace.points.back();
  if (std::hypot(pose.x - last.x, pose.y - last.y) >= min_step_m) {
    trace.points.push_back(pose);
  }
}
}  // namespace

bool PlanningEngine::append_executed_pose(
  const gpr_common::Pose2D & pose, const gpr_common::ExecutionTracePhase phase)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  append_pose_to_trace(executed_trace_, pose, executed_trace_step_m_);
  if (phase == gpr_common::ExecutionTracePhase::Coverage) {
    append_pose_to_trace(executed_coverage_trace_, pose, executed_trace_step_m_);
  } else {
    append_pose_to_trace(executed_transit_trace_, pose, executed_trace_step_m_);
  }
  return append_pose_to_trace(mission_executed_trace_, pose, executed_trace_step_m_);
}

void PlanningEngine::flush_coverage_pose(const gpr_common::Pose2D & pose)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  force_append_pose_to_trace(executed_coverage_trace_, pose, flush_pose_min_step_m_);
  force_append_pose_to_trace(executed_trace_, pose, flush_pose_min_step_m_);
  force_append_pose_to_trace(mission_executed_trace_, pose, flush_pose_min_step_m_);
}

void PlanningEngine::clear_executed_trace()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  executed_trace_ = {};
  executed_coverage_trace_ = {};
  executed_transit_trace_ = {};
}

const gpr_common::Polyline & PlanningEngine::mission_executed_trace() const noexcept
{
  return mission_executed_trace_;
}

bool PlanningEngine::job_completion_satisfied() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!current_job_.has_value() || executed_coverage_trace_.size() < 2U) {
    return false;
  }
  const auto & trace = executed_coverage_trace_;
  const auto & cfg = swath_coverage_config_;
  const auto * pass = find_pass_for_job(*current_job_);
  if (pass != nullptr) {
    const auto measured = gpr_common::compute_swath_coverage(
      pass->centerline, trace, cfg);
    if (measured.covered_fraction >= cfg.min_complete_fraction) {
      return true;
    }
    return cfg.partial_enabled &&
           measured.covered_fraction >= cfg.min_partial_fraction;
  }
  const auto ids = cover_ids_for_job(*current_job_, pass);
  bool any_match = false;
  for (const auto & seg : catalog_.segments()) {
    if (seg.outcome == gpr_common::SegmentOutcome::Skipped ||
      seg.outcome == gpr_common::SegmentOutcome::Completed)
    {
      continue;
    }
    bool matches = false;
    for (const auto id : ids) {
      if (seg.id == id ||
        (seg.baseline_id.has_value() && *seg.baseline_id == id))
      {
        matches = true;
        break;
      }
    }
    if (!matches) {
      continue;
    }
    any_match = true;
    const auto measured = gpr_common::compute_swath_coverage(seg.centerline, trace, cfg);
    if (measured.covered_fraction < cfg.min_complete_fraction &&
      !(cfg.partial_enabled && measured.covered_fraction >= cfg.min_partial_fraction))
    {
      return false;
    }
  }
  return any_match;
}

bool PlanningEngine::active_job_still_actionable() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!current_job_.has_value()) {
    return false;
  }
  return job_still_actionable(*current_job_, catalog_);
}

void PlanningEngine::release_current_job()
{
  current_job_.reset();
  last_executed_path_ = {};
  executed_trace_ = {};
  clear_active_transit_path();
}

void PlanningEngine::record_current_job_coverage()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!current_job_.has_value() || executed_coverage_trace_.size() < 2U) {
    return;
  }
  const auto * pass = find_pass_for_job(*current_job_);
  const auto ids = cover_ids_for_job(*current_job_, pass);
  const gpr_common::Polyline * pass_line = nullptr;
  gpr_common::Polyline pass_storage;
  if (pass != nullptr) {
    pass_storage = pass->centerline;
    pass_line = &pass_storage;
  }
  const std::optional<uint32_t> pass_lane_index = pass != nullptr ?
    std::optional<uint32_t>{pass->lane_index} : std::nullopt;
  const gpr_common::GridMap * grid_ptr =
    (latest_grid_ && !latest_grid_->empty()) ? latest_grid_.get() : nullptr;
  const PathInvalidatorConfig * inv_ptr = grid_ptr != nullptr ?
    &invalidator_config_ : nullptr;
  catalog_.apply_swath_coverage_for_job(
    ids, executed_coverage_trace_, swath_coverage_config_, pass_line,
    current_job_->direction, pass_lane_index, grid_ptr, inv_ptr);
  if (catalog_.catalog_changed_since(last_catalog_revision_)) {
    catalog_changed_ = true;
    last_catalog_revision_ = catalog_.revision();
  }
  notify_viz_refresh();
}

void PlanningEngine::mark_current_job_complete()
{
  mark_current_job_complete(executed_coverage_trace_);
}

void PlanningEngine::mark_current_job_complete(const gpr_common::Polyline & driven_path)
{
  if (!current_job_.has_value()) {
    return;
  }
  if (driven_path.size() < 2U) {
    planning_log_warn(
      log_,
      "mark_current_job_complete: driven trace too short — leaving segments pending");
    current_job_.reset();
    last_executed_path_ = {};
    executed_trace_ = {};
    executed_coverage_trace_ = {};
    executed_transit_trace_ = {};
    return;
  }
  record_current_job_coverage();
  current_job_.reset();
  last_executed_path_ = {};
  executed_trace_ = {};
  executed_coverage_trace_ = {};
  executed_transit_trace_ = {};
  clear_active_transit_path();
  notify_viz_refresh();
}

void PlanningEngine::skip_current_job()
{
  if (!current_job_.has_value()) {
    return;
  }
  const auto ids = current_job_->covers.empty() ?
    std::vector<gpr_common::SegmentId>{current_job_->segment_id} : current_job_->covers;
  for (const auto id : ids) {
    catalog_.mark_job_skipped(id);
  }
  catalog_changed_ = true;
  current_job_.reset();
  clear_active_transit_path();
  notify_viz_refresh();
}

void PlanningEngine::mark_current_job_blocked()
{
  if (!current_job_.has_value()) {
    return;
  }
  const auto ids = current_job_->covers.empty() ?
    std::vector<gpr_common::SegmentId>{current_job_->segment_id} : current_job_->covers;
  catalog_.mark_attempted_for_job(ids);
  catalog_.mark_blocked_for_job(ids);
  catalog_changed_ = true;
  current_job_.reset();
  last_executed_path_ = {};
  clear_active_transit_path();
  notify_viz_refresh();
}

void PlanningEngine::set_last_executed_path(const gpr_common::Polyline & path)
{
  last_executed_path_ = path;
}

void PlanningEngine::mark_current_job_attempted()
{
  if (!current_job_.has_value()) {
    return;
  }
  const auto ids = current_job_->covers.empty() ?
    std::vector<gpr_common::SegmentId>{current_job_->segment_id} : current_job_->covers;
  catalog_.mark_attempted_for_job(ids);
}

std::optional<double> PlanningEngine::current_job_entry_distance(
  const gpr_common::Pose2D & robot) const
{
  if (!current_job_.has_value()) {
    return std::nullopt;
  }
  const auto entry = current_job_entry_pose();
  if (!entry.has_value()) {
    return std::nullopt;
  }
  return std::hypot(entry->x - robot.x, entry->y - robot.y);
}

}  // namespace gpr_planning

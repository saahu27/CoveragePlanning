#include "gpr_mission/mission_behaviors.hpp"

#include <cmath>

#include "gpr_control/i_path_tracker.hpp"
#include "gpr_mission/planning_job_utils.hpp"
#include "gpr_planning/planning_ops.hpp"
#include "gpr_common/swath_coverage.hpp"
#include "gpr_common/types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gpr_mission
{

namespace
{

[[nodiscard]] gpr_planning::PlanningEngine * engine(MissionContext * c)
{
  return (c != nullptr && c->planning != nullptr) ? c->planning->engine.get() : nullptr;
}

}  // namespace

WaitForMap::WaitForMap(const std::string & name, const BT::NodeConfig & config)
: MissionSyncAction(name, config)
{}

BT::PortsList WaitForMap::providedPorts() {return {};}

BT::NodeStatus WaitForMap::tick()
{
  auto * c = mission_ctx();
  if (c->perception && c->perception->has_grid()) {
    c->state = gpr_common::MissionState::Planning;
    return BT::NodeStatus::SUCCESS;
  }
  c->state = gpr_common::MissionState::WaitingForMap;
  return BT::NodeStatus::RUNNING;
}

GenerateInitialCoverage::GenerateInitialCoverage(
  const std::string & name, const BT::NodeConfig & config)
: MissionSyncAction(name, config)
{}

BT::PortsList GenerateInitialCoverage::providedPorts() {return {};}

BT::NodeStatus GenerateInitialCoverage::tick()
{
  auto * c = mission_ctx();
  gpr_planning::generate_initial_coverage(*c->planning);
  auto * eng = engine(c);
  if (const auto pose = c->robot_pose()) {
    c->home_pose = *pose;
    eng->set_route_end_pose(*pose);
    RCLCPP_INFO(
      rclcpp::get_logger("gpr_mission"),
      "home pose recorded (%.2f, %.2f, %.2f rad)", pose->x, pose->y, pose->yaw);
  } else if (c->default_home_pose.has_value()) {
    c->home_pose = c->default_home_pose;
    eng->set_route_end_pose(c->default_home_pose);
    RCLCPP_INFO(
      rclcpp::get_logger("gpr_mission"),
      "home pose from config (%.2f, %.2f, %.2f rad)",
      c->home_pose->x, c->home_pose->y, c->home_pose->yaw);
  } else {
    eng->set_route_end_pose(std::nullopt);
    RCLCPP_WARN(
      rclcpp::get_logger("gpr_mission"),
      "home pose unavailable — return-home will be skipped");
  }
  return BT::NodeStatus::SUCCESS;
}

UpdateSegmentCatalog::UpdateSegmentCatalog(
  const std::string & name, const BT::NodeConfig & config)
: MissionSyncAction(name, config)
{}

BT::PortsList UpdateSegmentCatalog::providedPorts() {return {};}

BT::NodeStatus UpdateSegmentCatalog::tick()
{
  auto * c = mission_ctx();
  if (!c->perception->has_grid()) {
    return BT::NodeStatus::RUNNING;
  }
  engine(c)->update_from_grid(c->perception->latest_grid_ptr());
  return BT::NodeStatus::SUCCESS;
}

ScheduleNeedsRebuild::ScheduleNeedsRebuild(const std::string & name, const BT::NodeConfig & config)
: MissionCondition(name, config)
{}

BT::PortsList ScheduleNeedsRebuild::providedPorts() {return {};}

BT::NodeStatus ScheduleNeedsRebuild::tick()
{
  const auto * c = mission_ctx();
  if (c == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  return engine(const_cast<MissionContext *>(c))->schedule_needs_rebuild() ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

RecomputeSchedule::RecomputeSchedule(const std::string & name, const BT::NodeConfig & config)
: MissionStatefulAction(name, config)
{}

BT::PortsList RecomputeSchedule::providedPorts() {return {};}

BT::NodeStatus RecomputeSchedule::onStart()
{
  auto * c = mission_ctx();
  auto * eng = engine(c);
  if (eng == nullptr) {
    return BT::NodeStatus::FAILURE;
  }

  const bool force = c->force_replan.exchange(false);
  if (!force && eng->has_pending_jobs()) {
    return BT::NodeStatus::SUCCESS;
  }
  if (!eng->has_work_remaining()) {
    return BT::NodeStatus::SUCCESS;
  }
  if (eng->reachability_exhausted()) {
    return BT::NodeStatus::SUCCESS;
  }
  if (!force && !eng->schedule_needs_rebuild()) {
    if (eng->catalog_changed()) {
      eng->clear_catalog_changed_flag();
    }
    return BT::NodeStatus::SUCCESS;
  }
  if (c->planning_worker && c->planning_worker->busy()) {
    return BT::NodeStatus::RUNNING;
  }
  const auto pose = c->robot_pose();
  if (!pose) {
    return BT::NodeStatus::RUNNING;
  }
  eng->clear_catalog_changed_flag();
  request_id_ = submit_schedule_job(
    c,
    force ? gpr_planning::PlanningJobPriority::Urgent :
    gpr_planning::PlanningJobPriority::Normal,
    force);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RecomputeSchedule::onRunning()
{
  auto * c = mission_ctx();
  auto * eng = engine(c);
  if (eng == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  if (eng->has_pending_jobs()) {
    return BT::NodeStatus::SUCCESS;
  }
  return poll_planning_job(c, request_id_);
}

void RecomputeSchedule::onHalted()
{
  auto * c = mission_ctx();
  if (c != nullptr && c->planning_worker) {
    c->planning_worker->cancel_pending_below(gpr_planning::PlanningJobPriority::Normal);
  }
  request_id_ = 0U;
}

HasPendingJobs::HasPendingJobs(const std::string & name, const BT::NodeConfig & config)
: MissionCondition(name, config)
{}

BT::PortsList HasPendingJobs::providedPorts() {return {};}

BT::NodeStatus HasPendingJobs::tick()
{
  auto * c = mission_ctx();
  if (c->recovery_skip_count >= c->max_recovery_skips) {
    RCLCPP_WARN(
      rclcpp::get_logger("gpr_mission"),
      "coverage plateau: %u consecutive skips — ending mission",
      c->recovery_skip_count);
    return BT::NodeStatus::FAILURE;
  }
  return engine(c)->has_work_remaining() ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

ExecuteNextJob::ExecuteNextJob(const std::string & name, const BT::NodeConfig & config)
: MissionStatefulAction(name, config)
{}

BT::PortsList ExecuteNextJob::providedPorts() {return {};}

BT::NodeStatus ExecuteNextJob::onStart()
{
  auto * c = mission_ctx();
  auto * eng = engine(c);
  c->track_done = false;
  c->track_success = false;
  phase_ = Phase::Transit;
  interrupt_replan_request_id_ = 0U;

  const auto pose = c->robot_pose();
  if (!pose) {
    return BT::NodeStatus::RUNNING;
  }

  if (!eng->pop_next_job(pose, c->transit_skip_distance)) {
    if (!eng->has_work_remaining()) {
      RCLCPP_INFO(
        rclcpp::get_logger("gpr_mission"),
        "execute: no jobs in queue and no remaining work");
    }
    return BT::NodeStatus::SUCCESS;
  }

  const double prune = c->path_prune_distance;
  const auto transit = eng->transit_to_current_job(*pose);
  gpr_common::Polyline path;
  const auto entry_dist = eng->current_job_entry_distance(*pose);
  const bool needs_transit = !entry_dist.has_value() ||
    *entry_dist >= c->transit_skip_distance;
  if (needs_transit) {
    phase_ = Phase::Transit;
    if (!transit.has_value() || transit->size() < 2U) {
      RCLCPP_WARN(
        rclcpp::get_logger("gpr_mission"),
        "execute: no A* transit path to lane entry");
      return BT::NodeStatus::FAILURE;
    }
    path = *transit;
    if (!eng->is_transit_path_followable(path)) {
      RCLCPP_WARN(
        rclcpp::get_logger("gpr_mission"),
        "execute: no collision-free transit path to lane entry");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("gpr_mission"),
      "execute: starting transit (%zu poses)", path.size());
    gpr_planning::publish_transit_path(*c->planning, path);
  } else {
    if (entry_dist.has_value()) {
      RCLCPP_INFO(
        rclcpp::get_logger("gpr_mission"),
        "execute: skipping transit (entry %.2f m away)", *entry_dist);
    }
    gpr_planning::publish_transit_path(*c->planning, {});
    path = eng->current_job_followable_polyline_for_robot(*pose, prune);
    phase_ = Phase::Coverage;
    if (path.size() < 2U) {
      RCLCPP_WARN(
        rclcpp::get_logger("gpr_mission"),
        "execute: no followable coverage prefix at lane entry");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("gpr_mission"),
      "execute: starting coverage (%zu poses)", path.size());
  }

  if (phase_ == Phase::Coverage && !eng->is_current_job_followable(*pose, prune)) {
    RCLCPP_WARN(
      rclcpp::get_logger("gpr_mission"),
      "execute: coverage path collides with grid — skipping job");
    return BT::NodeStatus::FAILURE;
  }

  eng->clear_executed_trace();
  if (eng->append_executed_pose(
      *pose,
      phase_ == Phase::Coverage ? gpr_common::ExecutionTracePhase::Coverage :
      gpr_common::ExecutionTracePhase::Transit))
  {
    gpr_planning::publish_executed_trajectory(*c->planning);
  }
  eng->set_last_executed_path(path);
  eng->mark_current_job_attempted();
  c->control->follow(path, [c](gpr_control::TrackResult result) {
      c->track_done = true;
      c->track_success = (result == gpr_control::TrackResult::Succeeded);
    });
  return BT::NodeStatus::RUNNING;
}

namespace
{

std::optional<std::uint64_t> begin_interrupt_replan(
  MissionContext * c, const bool in_transit_phase)
{
  auto * eng = engine(c);
  if (!eng->catalog_changed()) {
    return std::nullopt;
  }
  const auto pose = c->robot_pose();
  if (!pose) {
    return std::nullopt;
  }
  if (!eng->active_job_needs_replan(
      *pose, c->path_prune_distance, c->transit_skip_distance, in_transit_phase))
  {
    eng->clear_catalog_changed_flag();
    return std::nullopt;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("gpr_mission"),
    in_transit_phase ?
    "execute: job invalidated during transit — interrupting for replan (keeping job)" :
    "execute: active job invalidated by map update — interrupting for replan");
  c->control->cancel();
  if (!eng->active_job_still_actionable()) {
    eng->release_current_job();
  }
  eng->clear_catalog_changed_flag();
  const std::uint64_t request_id = submit_schedule_job(
    c, gpr_planning::PlanningJobPriority::Urgent, true);
  if (request_id == 0U) {
    return std::nullopt;
  }
  return request_id;
}

}  // namespace

BT::NodeStatus ExecuteNextJob::onRunning()
{
  auto * c = mission_ctx();
  auto * eng = engine(c);

  if (interrupt_replan_request_id_ != 0U) {
    const auto status = poll_planning_job(
      c, interrupt_replan_request_id_,
      {.set_executing_state = false, .reset_track_on_terminal = true});
    if (status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    interrupt_replan_request_id_ = 0U;
    return status;
  }

  if (c->perception->has_grid()) {
    eng->update_from_grid(c->perception->latest_grid_ptr());
  }
  if (const auto replan_id = begin_interrupt_replan(c, phase_ == Phase::Transit)) {
    interrupt_replan_request_id_ = *replan_id;
    return BT::NodeStatus::RUNNING;
  }

  if (!c->track_done) {
    if (const auto pose = c->robot_pose()) {
      if (eng->append_executed_pose(
          *pose,
          phase_ == Phase::Coverage ? gpr_common::ExecutionTracePhase::Coverage :
          gpr_common::ExecutionTracePhase::Transit))
      {
        gpr_planning::publish_executed_trajectory(*c->planning);
      }
      if (phase_ == Phase::Transit) {
        gpr_planning::refresh_transit_display(
          *c->planning, *pose, c->transit_skip_distance);
      }
    }
    return BT::NodeStatus::RUNNING;
  }
  if (!c->track_success) {
    return BT::NodeStatus::FAILURE;
  }
  if (phase_ == Phase::Transit) {
    phase_ = Phase::Coverage;
    gpr_planning::publish_transit_path(*c->planning, {});
    c->track_done = false;
    c->track_success = false;
    const auto pose = c->robot_pose();
    if (!pose) {
      return BT::NodeStatus::FAILURE;
    }
    const double prune = c->path_prune_distance;
    const auto path = eng->current_job_followable_polyline_for_robot(*pose, prune);
    if (path.size() < 2U) {
      RCLCPP_WARN(
        rclcpp::get_logger("gpr_mission"),
        "execute: no followable coverage after transit");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("gpr_mission"),
      "execute: starting coverage (%zu poses)", path.size());
    eng->set_last_executed_path(path);
    eng->mark_current_job_attempted();
    c->control->follow(path, [c](gpr_control::TrackResult result) {
        c->track_done = true;
        c->track_success = (result == gpr_control::TrackResult::Succeeded);
      });
    return BT::NodeStatus::RUNNING;
  }
  if (const auto pose = c->robot_pose()) {
    eng->flush_coverage_pose(*pose);
  }
  eng->record_current_job_coverage();
  if (!eng->job_completion_satisfied()) {
    RCLCPP_WARN(
      rclcpp::get_logger("gpr_mission"),
      "execute: coverage leg finished but overlap threshold not met");
    return BT::NodeStatus::FAILURE;
  }
  c->recovery_skip_count = 0U;
  eng->mark_current_job_complete();
  return BT::NodeStatus::SUCCESS;
}

void ExecuteNextJob::onHalted()
{
  auto * c = mission_ctx();
  if (c == nullptr) {
    return;
  }
  c->control->cancel();
  if (c->planning_worker) {
    c->planning_worker->cancel_pending_below(gpr_planning::PlanningJobPriority::Normal);
  }
  interrupt_replan_request_id_ = 0U;
}

SkipCurrentJob::SkipCurrentJob(const std::string & name, const BT::NodeConfig & config)
: MissionSyncAction(name, config)
{}

BT::PortsList SkipCurrentJob::providedPorts() {return {};}

BT::NodeStatus SkipCurrentJob::tick()
{
  auto * c = mission_ctx();
  engine(c)->mark_current_job_blocked();
  c->force_replan = true;
  ++c->recovery_skip_count;
  RCLCPP_WARN(
    rclcpp::get_logger("gpr_mission"),
    "execute recovery: marked current job blocked (skip %u)",
    c->recovery_skip_count);
  return BT::NodeStatus::SUCCESS;
}

CancelControl::CancelControl(const std::string & name, const BT::NodeConfig & config)
: MissionSyncAction(name, config)
{}

BT::PortsList CancelControl::providedPorts() {return {};}

BT::NodeStatus CancelControl::tick()
{
  mission_ctx()->control->cancel();
  return BT::NodeStatus::SUCCESS;
}

MissionComplete::MissionComplete(const std::string & name, const BT::NodeConfig & config)
: MissionSyncAction(name, config)
{}

BT::PortsList MissionComplete::providedPorts() {return {};}

BT::NodeStatus MissionComplete::tick()
{
  auto * c = mission_ctx();
  RCLCPP_INFO(rclcpp::get_logger("gpr_mission"), "Mission complete — finalizing coverage report.");
  gpr_planning::finalize_coverage_report(
    *c->planning, c->planning_report_finalized,
    c->coverage_report_export_dir, c->coverage_report_export_file);
  c->state = gpr_common::MissionState::Complete;
  return BT::NodeStatus::SUCCESS;
}

ReturnToStart::ReturnToStart(const std::string & name, const BT::NodeConfig & config)
: MissionStatefulAction(name, config)
{}

BT::PortsList ReturnToStart::providedPorts() {return {};}

BT::NodeStatus ReturnToStart::onStart()
{
  auto * c = mission_ctx();
  auto * eng = engine(c);
  if (!c->return_home_enabled) {
    RCLCPP_INFO(rclcpp::get_logger("gpr_mission"), "return home: disabled");
    if (c->shutdown_on_complete) {
      c->shutdown_requested = true;
    }
    return BT::NodeStatus::SUCCESS;
  }
  if (!c->home_pose.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("gpr_mission"), "return home: no home pose — skipping");
    if (c->shutdown_on_complete) {
      c->shutdown_requested = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  const auto pose = c->robot_pose();
  if (!pose) {
    return BT::NodeStatus::RUNNING;
  }

  const double dx = pose->x - c->home_pose->x;
  const double dy = pose->y - c->home_pose->y;
  if (std::hypot(dx, dy) <= c->return_home_tolerance) {
    RCLCPP_INFO(
      rclcpp::get_logger("gpr_mission"),
      "return home: already at start (%.2f m away)", std::hypot(dx, dy));
    if (c->shutdown_on_complete) {
      c->shutdown_requested = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  c->state = gpr_common::MissionState::Executing;
  c->track_done = false;
  c->track_success = false;

  gpr_common::Polyline path;
  if (const auto planned = eng->plan_transit_path(*pose, *c->home_pose)) {
    path = *planned;
  }

  if (path.size() < 2U) {
    RCLCPP_WARN(
      rclcpp::get_logger("gpr_mission"),
      "return home: no collision-free path to start — skipping");
    if (c->shutdown_on_complete) {
      c->shutdown_requested = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("gpr_mission"),
    "return home: navigating to (%.2f, %.2f)", c->home_pose->x, c->home_pose->y);
  gpr_planning::publish_transit_path(*c->planning, path);
  if (eng->append_executed_pose(*pose, gpr_common::ExecutionTracePhase::Transit)) {
    gpr_planning::publish_executed_trajectory(*c->planning);
  }
  c->control->follow(path, [c](gpr_control::TrackResult result) {
      c->track_done = true;
      c->track_success = (result == gpr_control::TrackResult::Succeeded);
    });
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnToStart::onRunning()
{
  auto * c = mission_ctx();
  auto * eng = engine(c);
  if (!c->track_done) {
    if (const auto pose = c->robot_pose()) {
      if (eng->append_executed_pose(*pose, gpr_common::ExecutionTracePhase::Transit)) {
        gpr_planning::publish_executed_trajectory(*c->planning);
      }
    }
    return BT::NodeStatus::RUNNING;
  }
  gpr_planning::publish_transit_path(*c->planning, {});
  if (c->track_success) {
    RCLCPP_INFO(rclcpp::get_logger("gpr_mission"), "return home: reached start pose");
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("gpr_mission"),
      "return home: navigation failed — shutting down anyway");
  }
  if (c->shutdown_on_complete) {
    c->shutdown_requested = true;
  }
  return BT::NodeStatus::SUCCESS;
}

void ReturnToStart::onHalted()
{
  mission_ctx()->control->cancel();
}

}  // namespace gpr_mission

#include "gpr_mission/planning_job_utils.hpp"

namespace gpr_mission
{

std::optional<gpr_planning::PlanningSnapshot> build_planning_snapshot(
  MissionContext * context, const gpr_common::Pose2D & robot_pose)
{
  if (context == nullptr || !context->planning || !context->planning->engine ||
    !context->perception)
  {
    return std::nullopt;
  }
  gpr_planning::PlanningSnapshot snap;
  snap.robot_pose = robot_pose;
  snap.grid_seq = context->perception->grid_seq();
  snap.grid = context->perception->latest_grid_ptr();
  snap.catalog_rev = context->planning->engine->catalog().revision();
  return snap;
}

std::uint64_t submit_schedule_job(
  MissionContext * context,
  const gpr_planning::PlanningJobPriority priority,
  const bool force)
{
  if (context == nullptr || !context->planning_worker) {
    return 0U;
  }
  const auto pose = context->robot_pose();
  if (!pose) {
    return 0U;
  }
  const auto snap = build_planning_snapshot(context, *pose);
  if (!snap) {
    return 0U;
  }
  gpr_planning::PlanningJobRequest req;
  req.kind = gpr_planning::PlanningJobKind::RecomputeSchedule;
  req.priority = priority;
  req.force = force;
  req.snapshot = *snap;
  return context->planning_worker->submit(std::move(req));
}

BT::NodeStatus poll_planning_job(
  MissionContext * context,
  const std::uint64_t request_id,
  const PlanningJobPollOptions options)
{
  if (context == nullptr || !context->planning_worker || request_id == 0U) {
    return BT::NodeStatus::FAILURE;
  }
  const auto result = context->planning_worker->poll(request_id);
  if (!result) {
    return BT::NodeStatus::RUNNING;
  }
  switch (result->status) {
    case gpr_planning::PlanningJobStatus::Pending:
    case gpr_planning::PlanningJobStatus::Running:
      return BT::NodeStatus::RUNNING;
    case gpr_planning::PlanningJobStatus::Succeeded:
    case gpr_planning::PlanningJobStatus::Superseded:
    case gpr_planning::PlanningJobStatus::Cancelled:
      if (options.set_executing_state) {
        context->state = gpr_common::MissionState::Executing;
      }
      if (options.reset_track_on_terminal) {
        context->track_done = true;
        context->track_success = false;
      }
      return BT::NodeStatus::SUCCESS;
    case gpr_planning::PlanningJobStatus::Failed:
      return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace gpr_mission

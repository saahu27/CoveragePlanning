#ifndef GPR_MISSION__PLANNING_JOB_UTILS_HPP_
#define GPR_MISSION__PLANNING_JOB_UTILS_HPP_

#include <cstdint>
#include <optional>

#include "behaviortree_cpp/behavior_tree.h"
#include "gpr_mission/mission_context.hpp"
#include "gpr_planning/planning_snapshot.hpp"

namespace gpr_mission
{

struct PlanningJobPollOptions
{
  bool set_executing_state{true};
  bool reset_track_on_terminal{false};
};

[[nodiscard]] std::optional<gpr_planning::PlanningSnapshot> build_planning_snapshot(
  MissionContext * context, const gpr_common::Pose2D & robot_pose);

[[nodiscard]] std::uint64_t submit_schedule_job(
  MissionContext * context,
  gpr_planning::PlanningJobPriority priority,
  bool force);

[[nodiscard]] BT::NodeStatus poll_planning_job(
  MissionContext * context,
  std::uint64_t request_id,
  PlanningJobPollOptions options = {});

}  // namespace gpr_mission

#endif  // GPR_MISSION__PLANNING_JOB_UTILS_HPP_

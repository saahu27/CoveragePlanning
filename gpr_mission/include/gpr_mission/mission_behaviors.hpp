#ifndef GPR_MISSION__MISSION_BEHAVIORS_HPP_
#define GPR_MISSION__MISSION_BEHAVIORS_HPP_

#include <cstdint>

#include "behaviortree_cpp/behavior_tree.h"
#include "gpr_mission/mission_context.hpp"
#include "gpr_mission/mission_context_access.hpp"

namespace gpr_mission
{

/// @brief Synchronous action base providing access to the MissionContext.
class MissionSyncAction : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;

protected:
  [[nodiscard]] MissionContext * mission_ctx() {return get_mission_context(config());}
};

/// @brief Condition base providing access to the MissionContext.
class MissionCondition : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;

protected:
  [[nodiscard]] MissionContext * mission_ctx() {return get_mission_context(config());}
};

/// @brief Async (stateful) action base providing access to the MissionContext.
class MissionStatefulAction : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;

protected:
  [[nodiscard]] MissionContext * mission_ctx() {return get_mission_context(config());}
};

/// @brief Blocks until the first occupancy grid is available.
class WaitForMap : public MissionSyncAction
{
public:
  WaitForMap(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// @brief Generates the boustrophedon path and master segment set.
class GenerateInitialCoverage : public MissionSyncAction
{
public:
  GenerateInitialCoverage(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// @brief Refreshes per-segment blocked state from the latest grid.
class UpdateSegmentCatalog : public MissionSyncAction
{
public:
  UpdateSegmentCatalog(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// @brief Succeeds when the schedule must be rebuilt (open segments, stale queue).
class ScheduleNeedsRebuild : public MissionCondition
{
public:
  ScheduleNeedsRebuild(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// @brief Rebuilds the job schedule when needed (async worker thread).
class RecomputeSchedule : public MissionStatefulAction
{
public:
  RecomputeSchedule(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts();

private:
  std::uint64_t request_id_{0};
};

/// @brief Succeeds while open segments or queued jobs remain.
class HasPendingJobs : public MissionCondition
{
public:
  HasPendingJobs(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// @brief Drives the next job: transit to it, then cover it; marks it complete.
class ExecuteNextJob : public MissionStatefulAction
{
public:
  ExecuteNextJob(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts();

private:
  enum class Phase { Transit, Coverage };
  Phase phase_{Phase::Transit};
  std::uint64_t interrupt_replan_request_id_{0};
};

/// @brief Abandons the current job (recovery path).
class SkipCurrentJob : public MissionSyncAction
{
public:
  SkipCurrentJob(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// @brief Cancels any active path-following request (recovery path).
class CancelControl : public MissionSyncAction
{
public:
  CancelControl(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// @brief Marks the mission complete when no coverage remains.
class MissionComplete : public MissionSyncAction
{
public:
  MissionComplete(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// @brief After coverage, drive back to the pose captured at mission start.
class ReturnToStart : public MissionStatefulAction
{
public:
  ReturnToStart(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  static BT::PortsList providedPorts();
};

}  // namespace gpr_mission

#endif  // GPR_MISSION__MISSION_BEHAVIORS_HPP_

#ifndef GPR_MISSION__MISSION_CONTEXT_HPP_
#define GPR_MISSION__MISSION_CONTEXT_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>

#include "gpr_common/types.hpp"
#include "gpr_control/control_bridge.hpp"
#include "gpr_perception/perception_bridge.hpp"
#include "gpr_planning/planning_ops.hpp"
#include "gpr_planning/planning_worker.hpp"
#include "tf2_ros/buffer.h"

namespace gpr_mission
{

/// @brief Shared state passed to every behavior-tree node via the blackboard.
struct MissionContext
{
  std::shared_ptr<gpr_perception::PerceptionBridge> perception;
  std::shared_ptr<gpr_planning::PlanningServices> planning;
  std::shared_ptr<gpr_planning::PlanningWorker> planning_worker;
  std::shared_ptr<gpr_control::ControlBridge> control;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::string frame_id{"map"};
  std::string robot_base_frame{"base_link"};
  std::string coverage_report_export_dir{"results"};
  std::string coverage_report_export_file;
  double transit_skip_distance{0.35};
  double path_prune_distance{0.15};
  bool return_home_enabled{true};
  bool shutdown_on_complete{true};
  double return_home_tolerance{0.35};
  std::optional<gpr_common::Pose2D> home_pose;
  std::optional<gpr_common::Pose2D> default_home_pose;
  bool planning_report_finalized{false};

  gpr_common::MissionState state{gpr_common::MissionState::Idle};
  std::atomic<bool> track_done{false};
  std::atomic<bool> track_success{false};
  std::atomic<bool> force_replan{false};
  std::atomic<bool> shutdown_requested{false};
  std::uint32_t recovery_skip_count{0};
  std::uint32_t max_recovery_skips{12U};

  [[nodiscard]] std::optional<gpr_common::Pose2D> robot_pose() const;
};

}  // namespace gpr_mission

#endif  // GPR_MISSION__MISSION_CONTEXT_HPP_

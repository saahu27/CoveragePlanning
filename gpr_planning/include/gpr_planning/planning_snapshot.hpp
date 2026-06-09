#ifndef GPR_PLANNING__PLANNING_SNAPSHOT_HPP_
#define GPR_PLANNING__PLANNING_SNAPSHOT_HPP_

#include <cstdint>
#include <string>

#include "gpr_common/grid_map.hpp"
#include "gpr_common/types.hpp"

namespace gpr_planning
{

/// @brief Immutable planning input captured at job submit time.
struct PlanningSnapshot
{
  std::uint64_t grid_seq{0};
  std::uint64_t catalog_rev{0};
  gpr_common::GridMapConstPtr grid;
  gpr_common::Pose2D robot_pose{};
};

enum class PlanningJobKind : std::uint8_t
{
  RecomputeSchedule,
};

enum class PlanningJobPriority : std::uint8_t
{
  Background = 0,
  Normal = 1,
  Urgent = 2,
};

struct PlanningJobRequest
{
  std::uint64_t request_id{0};
  PlanningJobKind kind{PlanningJobKind::RecomputeSchedule};
  PlanningJobPriority priority{PlanningJobPriority::Normal};
  PlanningSnapshot snapshot;
  bool force{false};
};

enum class PlanningJobStatus : std::uint8_t
{
  Pending,
  Running,
  Succeeded,
  Superseded,
  Cancelled,
  Failed,
};

struct PlanningJobResult
{
  std::uint64_t request_id{0};
  PlanningJobStatus status{PlanningJobStatus::Pending};
  std::uint64_t input_catalog_rev{0};
  std::uint64_t input_grid_seq{0};
  std::uint64_t output_schedule_rev{0};
  bool has_work{false};
  bool reachability_exhausted{false};
  std::string error;
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__PLANNING_SNAPSHOT_HPP_

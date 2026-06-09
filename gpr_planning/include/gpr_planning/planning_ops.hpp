#ifndef GPR_PLANNING__PLANNING_OPS_HPP_
#define GPR_PLANNING__PLANNING_OPS_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "gpr_common/grid_map.hpp"
#include "gpr_common/types.hpp"
#include "gpr_planning/planning_engine.hpp"
#include "gpr_planning/planning_log.hpp"

namespace rclcpp
{
class Node;
}

namespace gpr_planning
{

class RosIoFacade;

/// @brief Engine + ROS I/O handles used by the planning worker and mission layer.
struct PlanningServices
{
  std::shared_ptr<PlanningEngine> engine;
  std::shared_ptr<RosIoFacade> io;
};

[[nodiscard]] PlanningLog make_planning_log_from_node(
  const std::shared_ptr<rclcpp::Node> & node);

void generate_initial_coverage(PlanningServices & services);

[[nodiscard]] bool recompute_and_publish(
  PlanningServices & services, const gpr_common::Pose2D & robot_pose);

void apply_snapshot_grid(
  PlanningServices & services,
  gpr_common::GridMapConstPtr grid,
  std::uint64_t grid_seq);

void publish_transit_path(PlanningServices & services, const gpr_common::Polyline & path);

void publish_executed_trajectory(PlanningServices & services);

void refresh_transit_display(
  PlanningServices & services,
  const gpr_common::Pose2D & robot_pose,
  double transit_skip_distance);

void publish_coverage_report(PlanningServices & services);

void finalize_coverage_report(
  PlanningServices & services,
  bool & report_finalized,
  const std::string & export_dir = "",
  const std::string & export_file = "");

}  // namespace gpr_planning

#endif  // GPR_PLANNING__PLANNING_OPS_HPP_

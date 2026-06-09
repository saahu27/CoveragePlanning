#include "gpr_planning/planning_ops.hpp"

#include "gpr_metrics/coverage_reporter.hpp"
#include "gpr_planning/ros_io_facade.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gpr_planning
{

PlanningLog make_planning_log_from_node(const std::shared_ptr<rclcpp::Node> & node)
{
  PlanningLog plog;
  plog.now_ns = [clock = node->get_clock()]() {
      return clock->now().nanoseconds();
    };
  plog.write = [logger = node->get_logger()](
    const PlanningLogLevel level, const std::string & message) {
      switch (level) {
        case PlanningLogLevel::Debug:
          RCLCPP_DEBUG(logger, "%s", message.c_str());
          break;
        case PlanningLogLevel::Info:
          RCLCPP_INFO(logger, "%s", message.c_str());
          break;
        case PlanningLogLevel::Warn:
          RCLCPP_WARN(logger, "%s", message.c_str());
          break;
        case PlanningLogLevel::Error:
          RCLCPP_ERROR(logger, "%s", message.c_str());
          break;
      }
    };
  return plog;
}

void generate_initial_coverage(PlanningServices & services)
{
  services.engine->generate_initial_coverage();
  services.io->publish_initial_coverage(*services.engine);
}

bool recompute_and_publish(
  PlanningServices & services, const gpr_common::Pose2D & robot_pose)
{
  const bool has_work = services.engine->recompute_schedule(robot_pose);
  services.io->publish_schedule(*services.engine);
  return has_work;
}

void apply_snapshot_grid(
  PlanningServices & services,
  const gpr_common::GridMapConstPtr grid,
  const std::uint64_t grid_seq)
{
  services.engine->apply_snapshot_grid(grid, grid_seq);
}

void publish_transit_path(PlanningServices & services, const gpr_common::Polyline & path)
{
  services.engine->set_active_transit_path(path);
  services.io->publish_transit_path(*services.engine, path);
}

void publish_executed_trajectory(PlanningServices & services)
{
  services.io->publish_executed_trajectory(*services.engine);
}

void refresh_transit_display(
  PlanningServices & services,
  const gpr_common::Pose2D & robot_pose,
  const double transit_skip_distance)
{
  services.engine->refresh_transit_display(robot_pose, transit_skip_distance);
  services.io->publish_transit_path(*services.engine, services.engine->active_transit_path());
}

void publish_coverage_report(PlanningServices & services)
{
  services.io->publish_coverage_report(*services.engine);
}

void finalize_coverage_report(
  PlanningServices & services,
  bool & report_finalized,
  const std::string & export_dir,
  const std::string & export_file)
{
  if (report_finalized) {
    return;
  }
  report_finalized = true;
  if (!services.engine->reporter().has_baseline()) {
    RCLCPP_WARN(
      services.io->node()->get_logger(), "finalize_coverage_report: no baseline recorded.");
    return;
  }
  publish_coverage_report(services);
  const auto metrics = services.engine->reporter().compute_metrics(
    services.engine->catalog().segments());
  const auto uncovered = services.engine->reporter().compute_uncovered(
    services.engine->catalog().segments());
  const auto summary = services.engine->reporter().format_text_report(metrics, uncovered);
  RCLCPP_INFO(services.io->node()->get_logger(), "\n%s", summary.c_str());
  std::string path = export_file;
  if (path.empty() && !export_dir.empty()) {
    const auto stamp = services.io->node()->now();
    path = gpr_metrics::CoverageReporter::make_export_path(
      export_dir, services.engine->reporter().baseline().mission_id,
      stamp.seconds(), stamp.nanoseconds());
  }
  if (!path.empty()) {
    if (services.engine->reporter().export_text_report(path, metrics, uncovered)) {
      RCLCPP_INFO(services.io->node()->get_logger(), "Wrote coverage report to %s", path.c_str());
    } else {
      RCLCPP_WARN(
        services.io->node()->get_logger(), "Failed to write coverage report to %s", path.c_str());
    }
  }
}

}  // namespace gpr_planning

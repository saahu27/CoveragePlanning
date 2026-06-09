#ifndef GPR_PLANNING__COVERAGE_REPORT_CONVERSIONS_HPP_
#define GPR_PLANNING__COVERAGE_REPORT_CONVERSIONS_HPP_

#include <string>
#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_metrics/coverage_metrics.hpp"
#include "gpr_metrics/coverage_reporter.hpp"
#include "gpr_metrics/uncovered_region.hpp"
#include "gpr_msgs/msg/coverage_job.hpp"
#include "gpr_msgs/msg/coverage_report.hpp"
#include "gpr_msgs/msg/coverage_segment.hpp"
#include "gpr_msgs/msg/schedule.hpp"
#include "gpr_msgs/msg/uncovered_region.hpp"
#include "rclcpp/time.hpp"

namespace gpr_planning
{

[[nodiscard]] gpr_msgs::msg::CoverageSegment to_ros_segment(
  const gpr_common::CoverageSegment & seg);

[[nodiscard]] gpr_msgs::msg::UncoveredRegion to_ros_uncovered(
  const gpr_metrics::UncoveredRegion & region);

[[nodiscard]] gpr_msgs::msg::CoverageReport to_ros_report(
  const gpr_metrics::CoverageMetrics & metrics,
  const std::vector<gpr_common::CoverageSegment> & segments,
  const std::vector<gpr_metrics::UncoveredRegion> & uncovered,
  uint64_t mission_id,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

[[nodiscard]] gpr_msgs::msg::CoverageJob to_ros_job(
  const gpr_common::CoverageJob & job);

[[nodiscard]] gpr_msgs::msg::Schedule to_ros_schedule(
  const std::vector<gpr_common::CoverageJob> & jobs,
  std::uint64_t catalog_revision,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

}  // namespace gpr_planning

#endif  // GPR_PLANNING__COVERAGE_REPORT_CONVERSIONS_HPP_

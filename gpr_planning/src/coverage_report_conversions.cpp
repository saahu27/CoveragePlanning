#include "gpr_planning/coverage_report_conversions.hpp"

#include "geometry_msgs/msg/pose.hpp"

namespace gpr_planning
{

namespace
{
geometry_msgs::msg::Pose to_ros_pose(const gpr_common::Pose2D & p)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.x;
  pose.position.y = p.y;
  const double half_yaw = p.yaw * 0.5;
  pose.orientation.z = std::sin(half_yaw);
  pose.orientation.w = std::cos(half_yaw);
  return pose;
}
}  // namespace

gpr_msgs::msg::CoverageSegment to_ros_segment(const gpr_common::CoverageSegment & seg)
{
  gpr_msgs::msg::CoverageSegment msg;
  msg.id = seg.id;
  msg.lane_index = seg.lane_index.value_or(0U);
  msg.outcome = static_cast<uint8_t>(seg.outcome);
  msg.blocked = seg.blocked;
  msg.centerline.reserve(seg.centerline.points.size());
  for (const auto & p : seg.centerline.points) {
    msg.centerline.push_back(to_ros_pose(p));
  }
  return msg;
}

gpr_msgs::msg::UncoveredRegion to_ros_uncovered(const gpr_metrics::UncoveredRegion & region)
{
  gpr_msgs::msg::UncoveredRegion msg;
  msg.reason = static_cast<uint8_t>(region.reason);
  msg.lane_index = region.lane_index;
  msg.segment_ids = region.segment_ids;
  msg.swath_length_m = region.swath_length_m;
  msg.swath_area_m2 = region.swath_area_m2;
  msg.bbox_x_min = region.bbox_x_min;
  msg.bbox_y_min = region.bbox_y_min;
  msg.bbox_x_max = region.bbox_x_max;
  msg.bbox_y_max = region.bbox_y_max;
  return msg;
}

gpr_msgs::msg::CoverageReport to_ros_report(
  const gpr_metrics::CoverageMetrics & metrics,
  const std::vector<gpr_common::CoverageSegment> & segments,
  const std::vector<gpr_metrics::UncoveredRegion> & uncovered,
  uint64_t mission_id,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  gpr_msgs::msg::CoverageReport msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.mission_id = mission_id;

  msg.coverage_pct = metrics.coverage_pct;
  msg.blocked_pct = metrics.blocked_pct;
  msg.skipped_pct = metrics.skipped_pct;
  msg.remaining_pct = metrics.remaining_pct;
  msg.plan_retention_pct = metrics.plan_retention_pct;

  msg.baseline_swath_length_m = metrics.baseline_swath_length_m;
  msg.completed_swath_length_m = metrics.completed_swath_length_m;
  msg.blocked_swath_length_m = metrics.blocked_swath_length_m;
  msg.skipped_swath_length_m = metrics.skipped_swath_length_m;
  msg.pending_swath_length_m = metrics.pending_swath_length_m;

  msg.baseline_swath_area_m2 = metrics.baseline_swath_area_m2;
  msg.completed_swath_area_m2 = metrics.completed_swath_area_m2;

  msg.segments_total = metrics.segments_total;
  msg.segments_completed = metrics.segments_completed;
  msg.segments_blocked = metrics.segments_blocked;
  msg.segments_skipped = metrics.segments_skipped;
  msg.segments_pending = metrics.segments_pending;

  msg.uncovered_regions.reserve(uncovered.size());
  for (const auto & region : uncovered) {
    msg.uncovered_regions.push_back(to_ros_uncovered(region));
  }
  msg.segments.reserve(segments.size());
  for (const auto & seg : segments) {
    msg.segments.push_back(to_ros_segment(seg));
  }
  return msg;
}

gpr_msgs::msg::CoverageJob to_ros_job(const gpr_common::CoverageJob & job)
{
  gpr_msgs::msg::CoverageJob msg;
  msg.segment_id = job.segment_id;
  msg.direction = static_cast<uint8_t>(job.direction);
  return msg;
}

gpr_msgs::msg::Schedule to_ros_schedule(
  const std::vector<gpr_common::CoverageJob> & jobs,
  const std::uint64_t catalog_revision,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  gpr_msgs::msg::Schedule msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.catalog_revision = catalog_revision;
  msg.jobs.reserve(jobs.size());
  for (const auto & job : jobs) {
    msg.jobs.push_back(to_ros_job(job));
  }
  return msg;
}

}  // namespace gpr_planning

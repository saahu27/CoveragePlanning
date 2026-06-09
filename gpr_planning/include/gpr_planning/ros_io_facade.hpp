#ifndef GPR_PLANNING__ROS_IO_FACADE_HPP_
#define GPR_PLANNING__ROS_IO_FACADE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gpr_common/types.hpp"
#include "gpr_metrics/uncovered_region.hpp"
#include "gpr_msgs/msg/coverage_report.hpp"
#include "gpr_msgs/msg/schedule.hpp"
#include "gpr_planning/planning_engine.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace gpr_planning
{

/// @brief ROS topic names and RViz visualization tunables.
struct RosIoFacadeConfig
{
  std::string coverage_report_topic{"/coverage_report"};
  std::string schedule_topic{"/coverage_schedule"};
  std::string executed_path_topic{"/coverage_executed_path"};
  std::string updated_coverage_path_markers_topic{"/updated_coverage_path_markers"};
  std::string initial_coverage_path_topic{"/initial_coverage_path"};
  std::string updated_coverage_path_topic{"/updated_coverage_path"};
  std::string updated_coverage_strips_topic{"/updated_coverage_path_strips"};
  std::string transit_path_topic{"/coverage_transit_path"};
  std::string swath_heatmap_topic{"/coverage_swath_heatmap"};
  std::string uncovered_markers_topic{"/coverage_report/uncovered_markers"};
  /// @brief Green vs gold threshold for baseline arc coloring (defaults match coverage).
  double complete_fraction{0.85};
  double partial_fraction{0.35};
  double marker_line_width{0.05};
  /// @brief Minimum polyline densification step (m) when no grid is available.
  double segment_densify_step_min_m{0.05};
  /// @brief Densify step = max(min, grid_resolution * factor) when grid exists.
  double segment_densify_resolution_factor{0.5};
  /// @brief Heatmap sample step (m) when no grid is available.
  double heatmap_sample_step_m{0.1};
  /// @brief Heatmap sample step lower bound when grid exists.
  double heatmap_sample_step_min_m{0.08};
  /// @brief Tolerance (m) when matching baseline arcs to catalog pieces.
  double baseline_part_overlap_tol_m{0.25};
  /// @brief Minimum mutual covered fraction to treat arcs as overlapping.
  double baseline_part_overlap_fraction{0.5};
  /// @brief Throttle period for coverage report info logs (ms).
  int coverage_report_log_throttle_ms{5000};
  /// @brief Heatmap cube width as fraction of sample step.
  double heatmap_cube_width_factor{0.8};
};

/// @brief ROS publishers and visualization for planning state (reads from PlanningEngine).
class RosIoFacade
{
public:
  RosIoFacade(
    const std::shared_ptr<rclcpp::Node> & node,
    const RosIoFacadeConfig & config = {});

  void refresh_all(const PlanningEngine & engine, bool reset_markers = false);
  void publish_initial_coverage(const PlanningEngine & engine);
  void publish_markers(const PlanningEngine & engine, bool reset_markers = false);
  void publish_updated_coverage_path(const PlanningEngine & engine);
  void publish_transit_path(const PlanningEngine & engine, const gpr_common::Polyline & path);
  void publish_executed_trajectory(const PlanningEngine & engine);
  void publish_schedule(const PlanningEngine & engine);
  void publish_coverage_report(const PlanningEngine & engine);
  void publish_swath_heatmap(const PlanningEngine & engine);
  void publish_uncovered_markers(
    const PlanningEngine & engine,
    const std::vector<gpr_metrics::UncoveredRegion> & regions);

  [[nodiscard]] const std::shared_ptr<rclcpp::Node> & node() const noexcept {return node_;}

private:
  RosIoFacadeConfig config_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr initial_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr updated_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr updated_coverage_strips_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr transit_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executed_path_;
  rclcpp::Publisher<gpr_msgs::msg::CoverageReport>::SharedPtr coverage_report_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr uncovered_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr swath_heatmap_;
  rclcpp::Publisher<gpr_msgs::msg::Schedule>::SharedPtr schedule_;
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__ROS_IO_FACADE_HPP_

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "gpr_mission/coverage_stack.hpp"
#include "gpr_mission/mission_behaviors.hpp"
#include "gpr_mission/mission_context_provider.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace
{
const char * mission_state_to_string(gpr_common::MissionState state)
{
  switch (state) {
    case gpr_common::MissionState::Idle: return "IDLE";
    case gpr_common::MissionState::WaitingForMap: return "WAITING_FOR_MAP";
    case gpr_common::MissionState::Planning: return "PLANNING";
    case gpr_common::MissionState::Executing: return "EXECUTING";
    case gpr_common::MissionState::Recovering: return "RECOVERING";
    case gpr_common::MissionState::Complete: return "COMPLETE";
    case gpr_common::MissionState::Failed: return "FAILED";
  }
  return "UNKNOWN";
}
}  // namespace

class GprMissionNode : public rclcpp::Node
{
public:
  GprMissionNode()
  : Node("gpr_mission")
  {
    declare_parameter("frame_id", "map");
    declare_parameter("robot_base_frame", "base_link");
    declare_parameter("tree_xml_file", std::string(""));
    declare_parameter("mission.tick_rate_hz", 2.0);
    declare_parameter("mission.groot2_port", 1668);
    declare_parameter("mission.return_home", true);
    declare_parameter("mission.return_home_tolerance", 0.35);
    declare_parameter("mission.shutdown_on_complete", true);
    declare_parameter("mission.executor_threads", 4);
    declare_parameter("mission.max_recovery_skips", 12);
    declare_parameter("status_topic", "/coverage_status");
    declare_parameter("coverage_report.topic", "/coverage_report");
    declare_parameter("coverage_schedule.topic", "/coverage_schedule");
    declare_parameter("coverage_report.export_dir", "results");
    declare_parameter("coverage_report.export_path", std::string(""));

    declare_parameter("scenario", std::string("rectangle"));
    declare_parameter("scan_area.x_min", -4.0);
    declare_parameter("scan_area.x_max", 4.0);
    declare_parameter("scan_area.y_min", -2.5);
    declare_parameter("scan_area.y_max", 2.5);
    declare_parameter("scan_area.polygon.x_min", -4.0);
    declare_parameter("scan_area.polygon.x_max", 4.0);
    declare_parameter("scan_area.polygon.y_min", -2.0);
    declare_parameter("scan_area.polygon.y_max", 2.5);
    declare_parameter("scan_area.polygon_vertices", std::vector<double>{});

    declare_parameter("local_mapping.resolution", 0.05);
    declare_parameter("local_mapping.inflation_radius", 0.35);
    declare_parameter("local_mapping.publish_rate_hz", 5.0);
    declare_parameter("local_mapping.hit_marking_radius", 0.15);
    declare_parameter("local_mapping.prob_hit", 0.85);
    declare_parameter("local_mapping.prob_miss", 0.35);
    declare_parameter("local_mapping.log_odds_clamp", 5.0);
    declare_parameter("local_mapping.occupied_export_threshold", 50);
    declare_parameter("local_mapping.scan_topic", "/scan");
    declare_parameter("local_mapping.map_topic", "/local_occupancy_grid");
    declare_parameter("local_mapping.nav_map_topic", "/local_occupancy_grid_planning");
    declare_parameter("local_mapping.scan_tf_timeout_sec", 0.2);
    declare_parameter("local_mapping.tf_warn_throttle_ms", 3000);

    declare_parameter("lane_spacing", 0.5);
    declare_parameter("waypoint_spacing", 0.1);
    declare_parameter("coverage_inset", 0.60);
    declare_parameter("segment_length", 1.0);
    declare_parameter("scan_direction", "x");

    declare_parameter("path_invalidator.obstacle_cost_threshold", 50);
    declare_parameter("path_invalidator.segment_sample_spacing", 0.05);
    declare_parameter("path_invalidator.footprint_radius", 0.22);
    declare_parameter("path_invalidator.treat_unknown_as_free", true);
    declare_parameter("path_invalidator.block_only_observed_occupied", true);
    declare_parameter("path_invalidator.boundary_ignore_margin", 0.0);
    declare_parameter("path_invalidator.map_inflation_radius", 0.0);
    declare_parameter("path_invalidator.collision_sample_resolution_factor", 0.5);
    declare_parameter("path_invalidator.legacy_segment_quantize_m", 0.05);

    declare_parameter("planning.executed_trace_step_m", 0.05);
    declare_parameter("planning.flush_pose_min_step_m", 0.0001);
    declare_parameter("planning.update_grid_log_throttle_ms", 2000);
    declare_parameter("planning.first_grid_blocked_warn_fraction", 0.5);

    declare_parameter("sequencer.require_reachable_transit", true);
    declare_parameter("sequencer.job_completion_overlap_fraction", 0.55);
    declare_parameter("coverage.lateral_max_m", 0.22);
    declare_parameter("coverage.heading_max_deg", 35.0);
    declare_parameter("coverage.sample_half_width_m", 0.025);
    declare_parameter("coverage.min_complete_fraction", 0.85);
    declare_parameter("coverage.min_partial_fraction", 0.35);
    declare_parameter("coverage.partial_enabled", true);
    declare_parameter("segment_catalog.schedule_blocked_probes", false);
    declare_parameter("segment_catalog.min_split_length_m", 0.25);
    declare_parameter("segment_catalog.mark_completed_overlap_tol_m", 0.25);
    declare_parameter("segment_catalog.mark_completed_min_overlap_fraction", 0.55);
    declare_parameter("segment_catalog.split_length_tolerance_m", 0.001);
    declare_parameter("oarp_lite.enabled", true);
    declare_parameter("oarp_lite.min_rank_length_m", 0.35);
    declare_parameter("oarp_lite.max_replan_generations", 2);
    declare_parameter("oarp_lite.overlap_dist_m", 0.2);
    declare_parameter("astar.obstacle_cost_threshold", 50);
    declare_parameter("astar.treat_unknown_as_free", true);
    declare_parameter("astar.footprint_radius", 0.22);
    declare_parameter("astar.blocked_transit_penalty", 4.0);
    declare_parameter("astar.fallback_grid_resolution_m", 0.05);
    declare_parameter("astar.collision_sample_resolution_factor", 0.5);
    declare_parameter("sequencer.time_limit_sec", 2.0);
    declare_parameter("sequencer.use_boustrophedon", true);
    declare_parameter("sequencer.merge_gap_m", 0.2);
    declare_parameter("sequencer.merge_endpoint_epsilon_m", 0.001);
    declare_parameter("sequencer.atsp.direction_disjunction_penalty", 100000000.0);
    declare_parameter("sequencer.atsp.unreachable_transit_cost", 1000000000.0);
    declare_parameter("sequencer.atsp.cost_scale", 1000.0);
    declare_parameter("sequencer.atsp.first_solution_strategy", std::string("PATH_CHEAPEST_ARC"));

    declare_parameter("metrics.mission_id_start", 1);
    declare_parameter("metrics.implicit_blocked_length_epsilon_m", 0.000001);
    declare_parameter("visualization.show_completed_segments", true);
    declare_parameter("visualization.executed_path_topic", "/coverage_executed_path");
    declare_parameter("visualization.updated_coverage_path_markers_topic", "/updated_coverage_path_markers");
    declare_parameter("visualization.initial_coverage_path_topic", "/initial_coverage_path");
    declare_parameter("visualization.updated_coverage_path_topic", "/updated_coverage_path");
    declare_parameter("visualization.updated_coverage_strips_topic", "/updated_coverage_path_strips");
    declare_parameter("visualization.transit_path_topic", "/coverage_transit_path");
    declare_parameter("visualization.swath_heatmap_topic", "/coverage_swath_heatmap");
    declare_parameter("visualization.uncovered_markers_topic", "/coverage_report/uncovered_markers");
    declare_parameter("visualization.complete_fraction", 0.85);
    declare_parameter("visualization.partial_fraction", 0.35);
    declare_parameter("visualization.marker_line_width", 0.05);
    declare_parameter("visualization.segment_densify_step_min_m", 0.05);
    declare_parameter("visualization.segment_densify_resolution_factor", 0.5);
    declare_parameter("visualization.heatmap_sample_step_m", 0.1);
    declare_parameter("visualization.heatmap_sample_step_min_m", 0.08);
    declare_parameter("visualization.baseline_part_overlap_tol_m", 0.25);
    declare_parameter("visualization.baseline_part_overlap_fraction", 0.5);
    declare_parameter("visualization.coverage_report_log_throttle_ms", 5000);
    declare_parameter("visualization.heatmap_cube_width_factor", 0.8);
    declare_parameter("control.follow_path_action", "follow_path");
    declare_parameter("control.controller_id", "FollowPath");
    declare_parameter("control.goal_checker_id", "general_goal_checker");
    declare_parameter("control.path_prune_distance", 0.15);
    declare_parameter("control.min_follow_distance", 0.25);
    declare_parameter("control.allow_instant_success", false);
    declare_parameter("control.transit_skip_distance", 0.35);
    declare_parameter("control.action_server_wait_timeout_sec", 2.0);
    declare_parameter("sim.spawn_x", -3.25);
    declare_parameter("sim.spawn_y", -1.75);
    declare_parameter("sim.spawn_yaw", 0.0);
    declare_parameter("sim.spawn_z", 0.01);
    declare_parameter("sim.polygon.spawn_x", -3.0);
    declare_parameter("sim.polygon.spawn_y", -1.5);
  }

  void initialize()
  {
    bt_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    control_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    stack_ = std::make_unique<gpr_mission::CoverageStack>(shared_from_this());
    gpr_mission::CoverageStack::InitOptions options;
    options.sensor_callback_group = sensor_callback_group_;
    options.control_callback_group = control_callback_group_;
    stack_->initialize(options);

    mission_provider_ =
      std::make_shared<gpr_mission::MissionContextHolder>(&stack_->context());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *stack_->context().tf_buffer);

    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(),
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    create_behavior_tree();

    const double tick_rate_hz = get_parameter("mission.tick_rate_hz").as_double();
    const auto tick_period = std::chrono::duration<double>(
      tick_rate_hz > 0.0 ? 1.0 / tick_rate_hz : 0.5);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(tick_period),
      std::bind(&GprMissionNode::tick_tree, this),
      bt_callback_group_);
  }

  void shutdown_stack()
  {
    if (stack_) {
      stack_->shutdown();
    }
  }

private:
  void create_behavior_tree()
  {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<gpr_mission::WaitForMap>("WaitForMap");
    factory.registerNodeType<gpr_mission::GenerateInitialCoverage>("GenerateInitialCoverage");
    factory.registerNodeType<gpr_mission::UpdateSegmentCatalog>("UpdateSegmentCatalog");
    factory.registerNodeType<gpr_mission::ScheduleNeedsRebuild>("ScheduleNeedsRebuild");
    factory.registerNodeType<gpr_mission::RecomputeSchedule>("RecomputeSchedule");
    factory.registerNodeType<gpr_mission::HasPendingJobs>("HasPendingJobs");
    factory.registerNodeType<gpr_mission::ExecuteNextJob>("ExecuteNextJob");
    factory.registerNodeType<gpr_mission::SkipCurrentJob>("SkipCurrentJob");
    factory.registerNodeType<gpr_mission::CancelControl>("CancelControl");
    factory.registerNodeType<gpr_mission::MissionComplete>("MissionComplete");
    factory.registerNodeType<gpr_mission::ReturnToStart>("ReturnToStart");

    auto blackboard = BT::Blackboard::create();
    blackboard->set<std::shared_ptr<gpr_mission::MissionContextProvider>>(
      "mission_provider", mission_provider_);

    std::string xml = get_parameter("tree_xml_file").as_string();
    if (xml.empty()) {
      xml = ament_index_cpp::get_package_share_directory("gpr_mission") +
        "/bt_xml/coverage_mission.xml";
    }
    tree_ = factory.createTreeFromFile(xml, blackboard);
    groot_ = std::make_unique<BT::Groot2Publisher>(
      tree_, static_cast<unsigned>(get_parameter("mission.groot2_port").as_int()));
  }

  void tick_tree()
  {
    auto & context = stack_->context();
    const auto status = tree_.tickOnce();
    publish_status();

    if (context.shutdown_requested) {
      RCLCPP_INFO(get_logger(), "Mission complete — shutting down.");
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    if (status == BT::NodeStatus::FAILURE) {
      context.state = gpr_common::MissionState::Failed;
      publish_status();
      RCLCPP_ERROR(get_logger(), "Mission tree failed.");
      timer_->cancel();
    }
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    msg.data = mission_state_to_string(stack_->context().state);
    status_pub_->publish(msg);
  }

  std::unique_ptr<gpr_mission::CoverageStack> stack_;
  std::shared_ptr<gpr_mission::MissionContextProvider> mission_provider_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr bt_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> groot_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GprMissionNode>();
  node->initialize();

  const int threads = static_cast<int>(node->get_parameter("mission.executor_threads").as_int());
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions{}, std::max(2, threads));
  executor.add_node(node);
  executor.spin();

  node->shutdown_stack();
  rclcpp::shutdown();
  return 0;
}

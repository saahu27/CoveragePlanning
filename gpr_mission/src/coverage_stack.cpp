#include "gpr_mission/coverage_stack.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

#include "gpr_common/scan_region.hpp"
#include "gpr_common/swath_coverage.hpp"
#include "gpr_control/control_bridge.hpp"
#include "gpr_metrics/metrics_config.hpp"
#include "gpr_perception/perception_bridge.hpp"
#include "gpr_planning/planning_ops.hpp"
#include "gpr_planning/ros_io_facade.hpp"

namespace gpr_mission
{

namespace
{

double param_as_double(const rclcpp::Parameter & param)
{
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(param.as_int());
  }
  return param.as_double();
}

int64_t param_as_int(const rclcpp::Parameter & param)
{
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return static_cast<int64_t>(param.as_double());
  }
  return param.as_int();
}

double load_param_double(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const double fallback)
{
  if (!node->has_parameter(name)) {
    return fallback;
  }
  return param_as_double(node->get_parameter(name));
}

int64_t load_param_int(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const int64_t fallback)
{
  if (!node->has_parameter(name)) {
    return fallback;
  }
  return param_as_int(node->get_parameter(name));
}

std::vector<double> load_polygon_vertices(const rclcpp::Node::SharedPtr & node)
{
  if (!node->has_parameter("scan_area.polygon_vertices")) {
    return {};
  }
  const auto & param = node->get_parameter("scan_area.polygon_vertices");
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    return {};
  }
  return param.as_double_array();
}

gpr_common::ScanRegion load_scan_region(const rclcpp::Node::SharedPtr & node)
{
  const std::string scenario = node->get_parameter("scenario").as_string();
  if (scenario == "polygon") {
    const gpr_common::ScanArea bounds{
      load_param_double(node, "scan_area.polygon.x_min", load_param_double(node, "scan_area.x_min", -4.0)),
      load_param_double(node, "scan_area.polygon.x_max", load_param_double(node, "scan_area.x_max", 4.0)),
      load_param_double(node, "scan_area.polygon.y_min", load_param_double(node, "scan_area.y_min", -2.5)),
      load_param_double(node, "scan_area.polygon.y_max", load_param_double(node, "scan_area.y_max", 2.5))};
    const auto flat = load_polygon_vertices(node);
    if (flat.size() >= 6U && flat.size() % 2U == 0U) {
      std::vector<gpr_common::Point2D> vertices;
      vertices.reserve(flat.size() / 2U);
      for (std::size_t i = 0; i + 1U < flat.size(); i += 2U) {
        vertices.push_back(gpr_common::Point2D{flat[i], flat[i + 1U]});
      }
      return gpr_common::ScanRegion::from_vertices(std::move(vertices));
    }
    throw std::invalid_argument(
      "scenario:=polygon requires scan_area.polygon_vertices (CCW, at least 3 vertices).");
  }

  const gpr_common::ScanArea bounds{
    load_param_double(node, "scan_area.x_min", -4.0),
    load_param_double(node, "scan_area.x_max", 4.0),
    load_param_double(node, "scan_area.y_min", -2.5),
    load_param_double(node, "scan_area.y_max", 2.5)};
  return gpr_common::ScanRegion::from_rectangle(bounds);
}

}  // namespace

CoverageStack::CoverageStack(const rclcpp::Node::SharedPtr & node)
: node_(node)
{}

void CoverageStack::initialize(const InitOptions & options)
{
  auto node = node_;
  context_.frame_id = node->get_parameter("frame_id").as_string();
  context_.robot_base_frame = node->get_parameter("robot_base_frame").as_string();
  context_.coverage_report_export_dir =
    node->get_parameter("coverage_report.export_dir").as_string();
  context_.coverage_report_export_file =
    node->get_parameter("coverage_report.export_path").as_string();
  context_.transit_skip_distance =
    load_param_double(node, "control.transit_skip_distance", 0.35);
  context_.path_prune_distance =
    load_param_double(node, "control.path_prune_distance", 0.15);
  context_.return_home_enabled = node->get_parameter("mission.return_home").as_bool();
  context_.return_home_tolerance =
    load_param_double(node, "mission.return_home_tolerance", 0.35);
  context_.shutdown_on_complete =
    node->get_parameter("mission.shutdown_on_complete").as_bool();
  context_.max_recovery_skips = static_cast<std::uint32_t>(std::max<int64_t>(
      1L, load_param_int(node, "mission.max_recovery_skips", 12)));
  context_.default_home_pose = gpr_common::Pose2D{
    load_param_double(node, "sim.spawn_x", -3.25),
    load_param_double(node, "sim.spawn_y", -1.75),
    load_param_double(node, "sim.spawn_yaw", 0.0)};
  context_.tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  const auto scan_region = load_scan_region(node);

  gpr_perception::OccupancyGridConfig map_config;
  map_config.scan_area = scan_region.bounds;
  map_config.resolution = load_param_double(node, "local_mapping.resolution", 0.05);
  map_config.inflation_radius =
    load_param_double(node, "local_mapping.inflation_radius", 0.35);
  map_config.hit_marking_radius =
    load_param_double(node, "local_mapping.hit_marking_radius", 0.15);
  map_config.prob_hit = load_param_double(node, "local_mapping.prob_hit", 0.85);
  map_config.prob_miss = load_param_double(node, "local_mapping.prob_miss", 0.35);
  map_config.log_odds_clamp = load_param_double(node, "local_mapping.log_odds_clamp", 5.0);
  map_config.occupied_export_threshold = static_cast<int>(
    load_param_int(node, "local_mapping.occupied_export_threshold", 50));

  context_.perception = std::make_shared<gpr_perception::PerceptionBridge>(
    node,
    gpr_perception::OccupancyGridMapper(map_config),
    context_.frame_id,
    node->get_parameter("local_mapping.scan_topic").as_string(),
    node->get_parameter("local_mapping.map_topic").as_string(),
    node->get_parameter("local_mapping.nav_map_topic").as_string(),
    load_param_double(node, "local_mapping.publish_rate_hz", 5.0),
    load_param_double(node, "local_mapping.scan_tf_timeout_sec", 0.2),
    static_cast<int>(load_param_int(node, "local_mapping.tf_warn_throttle_ms", 3000)),
    options.sensor_callback_group);

  gpr_planning::BoustrophedonConfig bconfig;
  bconfig.region = scan_region;
  bconfig.lane_spacing = load_param_double(node, "lane_spacing", 0.5);
  bconfig.waypoint_spacing = load_param_double(node, "waypoint_spacing", 0.1);
  bconfig.coverage_inset = load_param_double(node, "coverage_inset", 0.60);
  bconfig.segment_length = load_param_double(node, "segment_length", 1.0);
  bconfig.scan_direction = gpr_planning::BoustrophedonPlanner::scan_direction_from_string(
    node->get_parameter("scan_direction").as_string());

  gpr_planning::PathInvalidatorConfig inv_config;
  inv_config.obstacle_cost_threshold = static_cast<int>(
    load_param_int(node, "path_invalidator.obstacle_cost_threshold", 50));
  inv_config.segment_sample_spacing =
    load_param_double(node, "path_invalidator.segment_sample_spacing", 0.05);
  inv_config.collision_sample_resolution_factor =
    load_param_double(node, "path_invalidator.collision_sample_resolution_factor", 0.5);
  inv_config.legacy_segment_quantize_m =
    load_param_double(node, "path_invalidator.legacy_segment_quantize_m", 0.05);
  inv_config.footprint_radius =
    load_param_double(node, "path_invalidator.footprint_radius", 0.22);
  inv_config.map_inflation_radius =
    load_param_double(node, "path_invalidator.map_inflation_radius", 0.0);
  inv_config.treat_unknown_as_free =
    node->get_parameter("path_invalidator.treat_unknown_as_free").as_bool();
  inv_config.block_only_observed_occupied =
    node->get_parameter("path_invalidator.block_only_observed_occupied").as_bool();
  inv_config.region = scan_region;
  inv_config.boundary_ignore_margin =
    load_param_double(node, "path_invalidator.boundary_ignore_margin", 0.0);

  gpr_planning::SegmentCatalogConfig catalog_config;
  catalog_config.schedule_blocked_probes =
    node->get_parameter("segment_catalog.schedule_blocked_probes").as_bool();
  catalog_config.min_split_length_m =
    load_param_double(node, "segment_catalog.min_split_length_m", 0.25);
  catalog_config.mark_completed_overlap_tol_m =
    load_param_double(node, "segment_catalog.mark_completed_overlap_tol_m", 0.25);
  catalog_config.mark_completed_min_overlap_fraction =
    load_param_double(node, "segment_catalog.mark_completed_min_overlap_fraction", 0.55);
  catalog_config.split_length_tolerance_m =
    load_param_double(node, "segment_catalog.split_length_tolerance_m", 0.001);

  gpr_planning::OarpLiteConfig oarp_config;
  oarp_config.enabled = node->get_parameter("oarp_lite.enabled").as_bool();
  oarp_config.min_rank_length_m =
    load_param_double(node, "oarp_lite.min_rank_length_m", 0.35);
  oarp_config.max_replan_generations = static_cast<std::uint32_t>(std::max<int64_t>(
      0L, load_param_int(node, "oarp_lite.max_replan_generations", 2)));
  oarp_config.overlap_dist_m =
    load_param_double(node, "oarp_lite.overlap_dist_m", 0.2);

  gpr_planning::AStarConfig astar_config;
  astar_config.obstacle_cost_threshold = static_cast<int>(
    load_param_int(node, "astar.obstacle_cost_threshold", 50));
  astar_config.treat_unknown_as_free =
    node->get_parameter("astar.treat_unknown_as_free").as_bool();
  astar_config.footprint_radius =
    load_param_double(node, "astar.footprint_radius", 0.22);
  astar_config.fallback_grid_resolution_m =
    load_param_double(node, "astar.fallback_grid_resolution_m", 0.05);
  astar_config.collision_sample_resolution_factor =
    load_param_double(node, "astar.collision_sample_resolution_factor", 0.5);
  astar_config.blocked_transit_penalty =
    load_param_double(node, "astar.blocked_transit_penalty", 4.0);

  gpr_planning::BoustrophedonSequencerConfig seq_config;
  seq_config.merge_gap_m = load_param_double(node, "sequencer.merge_gap_m", 0.2);
  seq_config.merge_endpoint_epsilon_m =
    load_param_double(node, "sequencer.merge_endpoint_epsilon_m", 0.001);

  gpr_planning::AtspSolverConfig atsp_config;
  atsp_config.time_limit_sec = load_param_double(node, "sequencer.time_limit_sec", 2.0);
  atsp_config.direction_disjunction_penalty =
    load_param_double(node, "sequencer.atsp.direction_disjunction_penalty", 1e8);
  atsp_config.unreachable_transit_cost =
    load_param_double(node, "sequencer.atsp.unreachable_transit_cost", 1e9);
  atsp_config.cost_scale = load_param_double(node, "sequencer.atsp.cost_scale", 1000.0);
  atsp_config.first_solution_strategy =
    node->get_parameter("sequencer.atsp.first_solution_strategy").as_string();

  gpr_common::SwathCoverageConfig swath_config;
  swath_config.lateral_max_m = node->has_parameter("coverage.lateral_max_m") ?
    load_param_double(node, "coverage.lateral_max_m", 0.22) :
    load_param_double(node, "path_invalidator.footprint_radius", 0.22);
  swath_config.heading_max_rad =
    load_param_double(node, "coverage.heading_max_deg", 35.0) * M_PI / 180.0;
  swath_config.sample_half_width_m =
    load_param_double(node, "coverage.sample_half_width_m", 0.025);
  swath_config.min_complete_fraction = node->has_parameter("coverage.min_complete_fraction") ?
    load_param_double(node, "coverage.min_complete_fraction", 0.85) :
    load_param_double(node, "sequencer.job_completion_overlap_fraction", 0.55);
  swath_config.min_partial_fraction =
    load_param_double(node, "coverage.min_partial_fraction", 0.35);
  swath_config.partial_enabled = node->get_parameter("coverage.partial_enabled").as_bool();

  gpr_planning::PlanningDiagnosticsConfig diagnostics_config;
  diagnostics_config.update_grid_log_throttle_ms =
    load_param_double(node, "planning.update_grid_log_throttle_ms", 2000.0);
  diagnostics_config.first_grid_blocked_warn_fraction =
    load_param_double(node, "planning.first_grid_blocked_warn_fraction", 0.5);

  gpr_metrics::MetricsConfig metrics_config;
  metrics_config.mission_id_start = static_cast<std::uint64_t>(std::max<int64_t>(
      1L, load_param_int(node, "metrics.mission_id_start", 1)));
  metrics_config.implicit_blocked_length_epsilon_m =
    load_param_double(node, "metrics.implicit_blocked_length_epsilon_m", 1e-6);

  context_.planning = std::make_shared<gpr_planning::PlanningServices>();
  context_.planning->engine = std::make_shared<gpr_planning::PlanningEngine>(
    bconfig, inv_config, astar_config, context_.frame_id,
    atsp_config,
    catalog_config, oarp_config,
    node->get_parameter("sequencer.use_boustrophedon").as_bool(), seq_config,
    node->get_parameter("visualization.show_completed_segments").as_bool(),
    node->get_parameter("sequencer.require_reachable_transit").as_bool(),
    swath_config,
    load_param_double(node, "control.transit_skip_distance", 0.35),
    load_param_double(node, "planning.executed_trace_step_m", 0.05),
    load_param_double(node, "planning.flush_pose_min_step_m", 0.0001),
    diagnostics_config,
    metrics_config);

  gpr_planning::RosIoFacadeConfig io_config;
  io_config.coverage_report_topic =
    node->get_parameter("coverage_report.topic").as_string();
  io_config.schedule_topic =
    node->get_parameter("coverage_schedule.topic").as_string();
  io_config.executed_path_topic =
    node->get_parameter("visualization.executed_path_topic").as_string();
  io_config.updated_coverage_path_markers_topic =
    node->get_parameter("visualization.updated_coverage_path_markers_topic").as_string();
  io_config.initial_coverage_path_topic =
    node->get_parameter("visualization.initial_coverage_path_topic").as_string();
  io_config.updated_coverage_path_topic =
    node->get_parameter("visualization.updated_coverage_path_topic").as_string();
  io_config.updated_coverage_strips_topic =
    node->get_parameter("visualization.updated_coverage_strips_topic").as_string();
  io_config.transit_path_topic =
    node->get_parameter("visualization.transit_path_topic").as_string();
  io_config.swath_heatmap_topic =
    node->get_parameter("visualization.swath_heatmap_topic").as_string();
  io_config.uncovered_markers_topic =
    node->get_parameter("visualization.uncovered_markers_topic").as_string();
  io_config.complete_fraction = node->has_parameter("visualization.complete_fraction") ?
    load_param_double(node, "visualization.complete_fraction", 0.85) :
    load_param_double(node, "coverage.min_complete_fraction", 0.85);
  io_config.partial_fraction = node->has_parameter("visualization.partial_fraction") ?
    load_param_double(node, "visualization.partial_fraction", 0.35) :
    load_param_double(node, "coverage.min_partial_fraction", 0.35);
  io_config.marker_line_width =
    load_param_double(node, "visualization.marker_line_width", 0.05);
  io_config.segment_densify_step_min_m =
    load_param_double(node, "visualization.segment_densify_step_min_m", 0.05);
  io_config.segment_densify_resolution_factor =
    load_param_double(node, "visualization.segment_densify_resolution_factor", 0.5);
  io_config.heatmap_sample_step_m =
    load_param_double(node, "visualization.heatmap_sample_step_m", 0.1);
  io_config.heatmap_sample_step_min_m =
    load_param_double(node, "visualization.heatmap_sample_step_min_m", 0.08);
  io_config.baseline_part_overlap_tol_m =
    load_param_double(node, "visualization.baseline_part_overlap_tol_m", 0.25);
  io_config.baseline_part_overlap_fraction =
    load_param_double(node, "visualization.baseline_part_overlap_fraction", 0.5);
  io_config.coverage_report_log_throttle_ms = static_cast<int>(
    load_param_int(node, "visualization.coverage_report_log_throttle_ms", 5000));
  io_config.heatmap_cube_width_factor =
    load_param_double(node, "visualization.heatmap_cube_width_factor", 0.8);

  context_.planning->io = std::make_shared<gpr_planning::RosIoFacade>(node, io_config);

  context_.planning->engine->set_log(gpr_planning::make_planning_log_from_node(node));
  context_.planning->engine->set_viz_refresh_callback(
    [planning = context_.planning]() {
      planning->io->refresh_all(*planning->engine);
    });

  context_.planning_worker =
    std::make_shared<gpr_planning::PlanningWorker>(context_.planning);
  context_.planning_worker->start();

  gpr_control::Nav2TrackerConfig tracker_config;
  tracker_config.action_name = node->get_parameter("control.follow_path_action").as_string();
  tracker_config.controller_id = node->get_parameter("control.controller_id").as_string();
  tracker_config.goal_checker_id = node->get_parameter("control.goal_checker_id").as_string();
  tracker_config.robot_base_frame = context_.robot_base_frame;
  tracker_config.path_prune_distance = context_.path_prune_distance;
  tracker_config.min_follow_distance =
    load_param_double(node, "control.min_follow_distance", 0.25);
  tracker_config.allow_instant_success =
    node->get_parameter("control.allow_instant_success").as_bool();
  tracker_config.action_server_wait_timeout_sec =
    load_param_double(node, "control.action_server_wait_timeout_sec", 2.0);

  context_.control = std::make_shared<gpr_control::ControlBridge>(
    node, context_.tf_buffer, context_.frame_id, tracker_config,
    options.control_callback_group);
}

void CoverageStack::shutdown()
{
  if (context_.planning_worker) {
    context_.planning_worker->stop();
    context_.planning_worker.reset();
  }
}

}  // namespace gpr_mission

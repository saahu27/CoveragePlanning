#include "gpr_planning/ros_io_facade.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "gpr_planning/coverage_report_conversions.hpp"
#include "gpr_common/coverage_types.hpp"
#include "gpr_common/swath_coverage.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gpr_planning
{

namespace
{
bool segment_sort_order(
  const gpr_common::CoverageSegment & a, const gpr_common::CoverageSegment & b)
{
  const uint32_t lane_a = a.lane_index.value_or(std::numeric_limits<uint32_t>::max());
  const uint32_t lane_b = b.lane_index.value_or(std::numeric_limits<uint32_t>::max());
  if (lane_a != lane_b) {
    return lane_a < lane_b;
  }
  return a.id < b.id;
}

std::vector<gpr_common::Polyline> collect_remaining_coverage_strips(
  const SegmentCatalog & catalog,
  const gpr_common::GridMap & grid,
  const PathInvalidatorConfig & invalidator_config)
{
  std::vector<gpr_common::CoverageSegment> pending;
  pending.reserve(catalog.segments().size());
  for (const auto & seg : catalog.segments()) {
    if (seg.blocked || seg.centerline.size() < 2U) {
      continue;
    }
    if (seg.outcome != gpr_common::SegmentOutcome::Pending) {
      continue;
    }
    pending.push_back(seg);
  }
  std::sort(pending.begin(), pending.end(), segment_sort_order);

  const bool have_grid = !grid.empty();
  std::vector<gpr_common::Polyline> strips;
  for (const auto & seg : pending) {
    const auto schedulable = gpr_common::schedulable_centerline(
      seg, gpr_common::DriveDirection::Forward);
    if (schedulable.size() < 2U) {
      continue;
    }
    std::vector<gpr_common::Polyline> free_parts;
    if (have_grid) {
      free_parts = PathInvalidator::free_subpolylines(
        schedulable, grid, invalidator_config);
      if (free_parts.empty()) {
        continue;
      }
    } else {
      free_parts.push_back(schedulable);
    }
    for (auto & part : free_parts) {
      if (part.size() >= 2U) {
        strips.push_back(std::move(part));
      }
    }
  }
  return strips;
}

nav_msgs::msg::Path to_path_msg(
  const gpr_common::Polyline & line, const std::string & frame_id, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(line.points.size());
  for (const auto & p : line.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    const double half_yaw = p.yaw * 0.5;
    pose.pose.orientation.z = std::sin(half_yaw);
    pose.pose.orientation.w = std::cos(half_yaw);
    path.poses.push_back(pose);
  }
  return path;
}

gpr_common::Polyline densify_polyline(
  const gpr_common::Polyline & line, const double step_m)
{
  if (line.size() < 2U || step_m <= 0.0) {
    return line;
  }
  gpr_common::Polyline dense;
  for (std::size_t i = 0; i + 1U < line.size(); ++i) {
    const auto & a = line.points[i];
    const auto & b = line.points[i + 1U];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len < 1e-9) {
      continue;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(len / step_m)));
    const int start = dense.empty() ? 0 : 1;
    for (int s = start; s <= steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      dense.points.push_back({a.x + t * dx, a.y + t * dy, std::atan2(dy, dx)});
    }
  }
  return dense;
}

bool pose_is_finite(const gpr_common::Pose2D & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.yaw);
}

void append_pose_to_path(
  nav_msgs::msg::Path & path,
  const gpr_common::Pose2D & point,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (!pose_is_finite(point)) {
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;
  pose.pose.position.x = point.x;
  pose.pose.position.y = point.y;
  const double half_yaw = point.yaw * 0.5;
  pose.pose.orientation.z = std::sin(half_yaw);
  pose.pose.orientation.w = std::cos(half_yaw);
  path.poses.push_back(pose);
}
}  // namespace

RosIoFacade::RosIoFacade(
  const std::shared_ptr<rclcpp::Node> & node,
  const RosIoFacadeConfig & config)
: config_(config),
  node_(node)
{
  const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.updated_coverage_path_markers_topic, latched);
  initial_path_ = node->create_publisher<nav_msgs::msg::Path>(
    config_.initial_coverage_path_topic, latched);
  updated_path_ = node->create_publisher<nav_msgs::msg::Path>(
    config_.updated_coverage_path_topic, latched);
  updated_coverage_strips_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.updated_coverage_strips_topic, latched);
  transit_path_ = node->create_publisher<nav_msgs::msg::Path>(
    config_.transit_path_topic, latched);
  coverage_report_ = node->create_publisher<gpr_msgs::msg::CoverageReport>(
    config_.coverage_report_topic, latched);
  uncovered_markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.uncovered_markers_topic, latched);
  swath_heatmap_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.swath_heatmap_topic, latched);
  schedule_ = node->create_publisher<gpr_msgs::msg::Schedule>(
    config_.schedule_topic, latched);
  executed_path_ = node->create_publisher<nav_msgs::msg::Path>(
    config_.executed_path_topic, volatile_qos);
}

void RosIoFacade::refresh_all(const PlanningEngine & engine, const bool reset_markers)
{
  publish_markers(engine, reset_markers);
  publish_updated_coverage_path(engine);
  publish_swath_heatmap(engine);
  publish_coverage_report(engine);
}

void RosIoFacade::publish_initial_coverage(const PlanningEngine & engine)
{
  const auto stamp = node_->now();
  initial_path_->publish(to_path_msg(engine.initial_path(), engine.frame_id(), stamp));
  executed_path_->publish(to_path_msg({}, engine.frame_id(), stamp));
  refresh_all(engine, true);
}

namespace
{
constexpr std::size_t kFreePartMarkerBase = 1U;
constexpr std::size_t kBlockedPartMarkerBase = 50U;

int segment_marker_id(const gpr_common::SegmentId id, const std::size_t part_index = 0)
{
  return static_cast<int>((id % 1000000ULL) * 100ULL + part_index);
}

void set_segment_marker_color(
  visualization_msgs::msg::Marker & marker,
  const gpr_common::CoverageSegment & seg,
  const bool display_blocked)
{
  marker.color.a = 1.0f;
  if (seg.outcome == gpr_common::SegmentOutcome::Completed) {
    marker.color.r = 0.1f;
    marker.color.g = 0.8f;
    marker.color.b = 0.1f;
    marker.pose.position.z = 0.08;
  } else if (seg.outcome == gpr_common::SegmentOutcome::PartiallyCompleted) {
    marker.color.r = 0.95f;
    marker.color.g = 0.75f;
    marker.color.b = 0.1f;
    marker.pose.position.z = 0.075;
  } else if (seg.outcome == gpr_common::SegmentOutcome::Skipped) {
    marker.color.r = 0.6f;
    marker.color.g = 0.1f;
    marker.color.b = 0.8f;
    marker.pose.position.z = 0.07;
  } else if (display_blocked) {
    if (!seg.attempted) {
      marker.color.r = 0.9f;
      marker.color.g = 0.4f;
      marker.color.b = 0.1f;
    } else {
      marker.color.r = 0.9f;
      marker.color.g = 0.1f;
      marker.color.b = 0.1f;
    }
    marker.pose.position.z = 0.08;
  } else if (seg.source == gpr_common::SegmentSource::OarpRank) {
    marker.color.r = 0.2f;
    marker.color.g = 0.6f;
    marker.color.b = 1.0f;
    marker.pose.position.z = 0.07;
  } else {
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.pose.position.z = 0.07;
  }
}

visualization_msgs::msg::Marker make_line_strip_marker(
  const PlanningEngine & engine,
  const rclcpp::Time & stamp,
  const gpr_common::SegmentId segment_id,
  const std::size_t part_index,
  const gpr_common::Polyline & display_line,
  const double line_width)
{
  visualization_msgs::msg::Marker line;
  line.header.frame_id = engine.frame_id();
  line.header.stamp = stamp;
  line.ns = "coverage_segments";
  line.id = segment_marker_id(segment_id, part_index);
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = line_width;
  for (const auto & p : display_line.points) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x;
    pt.y = p.y;
    line.points.push_back(pt);
  }
  return line;
}

void append_densified_segment_markers(
  visualization_msgs::msg::MarkerArray & array,
  const PlanningEngine & engine,
  const rclcpp::Time & stamp,
  const gpr_common::CoverageSegment & seg,
  const std::vector<gpr_common::Polyline> & parts,
  const std::size_t part_id_base,
  const bool as_blocked,
  const double viz_step,
  const double line_width)
{
  for (std::size_t i = 0; i < parts.size(); ++i) {
    const auto dense = densify_polyline(parts[i], viz_step);
    if (dense.size() < 2U) {
      continue;
    }
    auto line = make_line_strip_marker(
      engine, stamp, seg.id, part_id_base + i, dense, line_width);
    set_segment_marker_color(line, seg, as_blocked);
    array.markers.push_back(line);
  }
}

std::vector<gpr_common::Polyline> filter_short_polylines(
  const std::vector<gpr_common::Polyline> & parts,
  const double min_length_m)
{
  std::vector<gpr_common::Polyline> kept;
  kept.reserve(parts.size());
  for (const auto & part : parts) {
    if (part.length() >= min_length_m) {
      kept.push_back(part);
    }
  }
  return kept;
}

bool part_overlaps_polyline(
  const gpr_common::Polyline & part,
  const gpr_common::Polyline & line,
  const double tol_m,
  const double min_overlap_fraction)
{
  if (part.size() < 2U || line.size() < 2U) {
    return false;
  }
  return gpr_common::polyline_covered_fraction(part, line, tol_m) >= min_overlap_fraction ||
         gpr_common::polyline_covered_fraction(line, part, tol_m) >= min_overlap_fraction;
}

bool segment_contributes_baseline_viz(
  const gpr_common::CoverageSegment & seg,
  const gpr_common::CoverageSegment & baseline);

std::vector<std::pair<double, double>> baseline_coverage_intervals(
  const gpr_common::CoverageSegment & baseline,
  const SegmentCatalog & catalog);

double part_arc_covered_fraction(
  const gpr_common::Polyline & part,
  const gpr_common::Polyline & baseline,
  const std::vector<std::pair<double, double>> & covered_intervals_m);

gpr_common::SegmentOutcome classify_baseline_part_viz_outcome(
  const gpr_common::CoverageSegment & baseline,
  const gpr_common::Polyline & part,
  const SegmentCatalog & catalog,
  const RosIoFacadeConfig & viz)
{
  const auto baseline_iv = baseline_coverage_intervals(baseline, catalog);
  if (baseline_iv.empty()) {
    return gpr_common::SegmentOutcome::Pending;
  }
  const double frac = part_arc_covered_fraction(
    part, baseline.centerline, baseline_iv);
  if (frac >= viz.complete_fraction) {
    return gpr_common::SegmentOutcome::Completed;
  }
  if (frac >= viz.partial_fraction || frac > 1e-6) {
    return gpr_common::SegmentOutcome::PartiallyCompleted;
  }
  return gpr_common::SegmentOutcome::Pending;
}

bool baseline_viz_fully_completed(
  const gpr_common::CoverageSegment & baseline,
  const SegmentCatalog & catalog,
  const RosIoFacadeConfig & viz)
{
  const auto baseline_iv = baseline_coverage_intervals(baseline, catalog);
  const double total = gpr_common::polyline_arc_length(baseline.centerline);
  if (total <= 1e-9 || baseline_iv.empty()) {
    return false;
  }
  return gpr_common::intervals_union_length(baseline_iv) / total >= viz.complete_fraction;
}

gpr_common::SegmentOutcome classify_free_part_viz_outcome(
  const gpr_common::CoverageSegment & style_seg,
  const gpr_common::Polyline & part,
  const SegmentCatalog & catalog,
  const bool baseline_arc_mode,
  const RosIoFacadeConfig & viz)
{
  if (baseline_arc_mode) {
    return classify_baseline_part_viz_outcome(style_seg, part, catalog, viz);
  }
  // Catalog segment: use segment outcome; refine only when already partial/complete.
  if (style_seg.outcome == gpr_common::SegmentOutcome::Pending) {
    return gpr_common::SegmentOutcome::Pending;
  }
  if (style_seg.outcome == gpr_common::SegmentOutcome::Completed ||
    style_seg.outcome == gpr_common::SegmentOutcome::Skipped)
  {
    return style_seg.outcome;
  }
  if (style_seg.covered_intervals_m.empty()) {
    return style_seg.outcome;
  }
  const double total = gpr_common::polyline_arc_length(style_seg.centerline);
  const double frac = total > 1e-9 ?
    gpr_common::intervals_union_length(
      gpr_common::merge_intervals(style_seg.covered_intervals_m)) / total :
    0.0;
  if (frac >= viz.complete_fraction) {
    return gpr_common::SegmentOutcome::Completed;
  }
  if (frac >= viz.partial_fraction || frac > 1e-6) {
    return gpr_common::SegmentOutcome::PartiallyCompleted;
  }
  return gpr_common::SegmentOutcome::Pending;
}

gpr_common::SegmentOutcome classify_baseline_viz_outcome(
  const gpr_common::CoverageSegment & baseline,
  const SegmentCatalog & catalog,
  const RosIoFacadeConfig & viz)
{
  if (baseline_viz_fully_completed(baseline, catalog, viz)) {
    return gpr_common::SegmentOutcome::Completed;
  }
  const auto baseline_iv = baseline_coverage_intervals(baseline, catalog);
  if (!baseline_iv.empty()) {
    const double total = gpr_common::polyline_arc_length(baseline.centerline);
    const double frac = total > 1e-9 ?
      gpr_common::intervals_union_length(baseline_iv) / total : 0.0;
    if (frac >= viz.partial_fraction || frac > 1e-6) {
      return gpr_common::SegmentOutcome::PartiallyCompleted;
    }
  }
  bool has_pending = false;
  for (const auto & seg : catalog.segments()) {
    if (!segment_contributes_baseline_viz(seg, baseline)) {
      continue;
    }
    if (seg.outcome == gpr_common::SegmentOutcome::Pending) {
      has_pending = true;
      break;
    }
  }
  return has_pending ?
         gpr_common::SegmentOutcome::Pending :
         gpr_common::SegmentOutcome::PartiallyCompleted;
}

bool segment_contributes_baseline_viz(
  const gpr_common::CoverageSegment & seg,
  const gpr_common::CoverageSegment & baseline)
{
  return gpr_common::effective_baseline_id(seg) == baseline.id;
}

std::vector<std::pair<double, double>> baseline_coverage_intervals(
  const gpr_common::CoverageSegment & baseline,
  const SegmentCatalog & catalog)
{
  std::vector<std::pair<double, double>> all;
  for (const auto & seg : catalog.segments()) {
    if (!segment_contributes_baseline_viz(seg, baseline) ||
      seg.covered_intervals_m.empty())
    {
      continue;
    }
    const auto mapped = gpr_common::map_covered_intervals_between_polylines(
      seg.covered_intervals_m, seg.centerline, baseline.centerline);
    all.insert(all.end(), mapped.begin(), mapped.end());
  }
  return gpr_common::merge_intervals(std::move(all));
}

double part_arc_covered_fraction(
  const gpr_common::Polyline & part,
  const gpr_common::Polyline & baseline,
  const std::vector<std::pair<double, double>> & covered_intervals_m)
{
  if (part.size() < 2U || baseline.size() < 2U || covered_intervals_m.empty()) {
    return 0.0;
  }
  const auto proj0 = gpr_common::project_point_onto_polyline(part.front_pose(), baseline);
  const auto proj1 = gpr_common::project_point_onto_polyline(part.back_pose(), baseline);
  if (!proj0.valid || !proj1.valid) {
    return 0.0;
  }
  const double lo = std::min(proj0.arc_s_m, proj1.arc_s_m);
  const double hi = std::max(proj0.arc_s_m, proj1.arc_s_m);
  const double span = hi - lo;
  if (span <= 1e-6) {
    return 0.0;
  }
  double covered = 0.0;
  for (const auto & iv : covered_intervals_m) {
    const double olo = std::max(lo, iv.first);
    const double ohi = std::min(hi, iv.second);
    if (ohi > olo) {
      covered += ohi - olo;
    }
  }
  return covered / span;
}

void publish_swath_collision_markers(
  visualization_msgs::msg::MarkerArray & array,
  const PlanningEngine & engine,
  const rclcpp::Time & stamp,
  const gpr_common::CoverageSegment & style_seg,
  const gpr_common::Polyline & centerline,
  const double viz_step,
  const bool have_grid,
  const double min_fragment_m,
  const RosIoFacadeConfig & viz,
  const bool baseline_arc_mode = false)
{
  if (centerline.size() < 2U) {
    return;
  }
  if (!have_grid) {
    append_densified_segment_markers(
      array, engine, stamp, style_seg, {centerline}, 0U,
      style_seg.blocked, viz_step, viz.marker_line_width);
    return;
  }

  const auto & grid = engine.latest_grid();
  const auto & inv = engine.invalidator_config();
  const auto blocked_parts = filter_short_polylines(
    PathInvalidator::blocked_subpolylines(centerline, grid, inv), min_fragment_m);
  const auto free_parts = filter_short_polylines(
    PathInvalidator::free_subpolylines(centerline, grid, inv), min_fragment_m);

  gpr_common::CoverageSegment blocked_style = style_seg;
  blocked_style.attempted = true;  // map-collision overlay is always drawn red
  append_densified_segment_markers(
    array, engine, stamp, blocked_style, blocked_parts,
    kBlockedPartMarkerBase, true, viz_step, viz.marker_line_width);

  for (std::size_t i = 0; i < free_parts.size(); ++i) {
    gpr_common::CoverageSegment free_style = style_seg;
    free_style.outcome = classify_free_part_viz_outcome(
      style_seg, free_parts[i], engine.catalog(), baseline_arc_mode, viz);
    append_densified_segment_markers(
      array, engine, stamp, free_style, {free_parts[i]},
      kFreePartMarkerBase + i, false, viz_step, viz.marker_line_width);
  }

  if (style_seg.blocked && blocked_parts.empty() && free_parts.empty()) {
    append_densified_segment_markers(
      array, engine, stamp, blocked_style, {centerline},
      kBlockedPartMarkerBase, true, viz_step, viz.marker_line_width);
  }
}

bool arc_s_is_covered(
  const double arc_s_m,
  const std::vector<std::pair<double, double>> & covered_intervals_m) noexcept
{
  for (const auto & iv : covered_intervals_m) {
    if (arc_s_m >= iv.first - 1e-6 && arc_s_m <= iv.second + 1e-6) {
      return true;
    }
  }
  return false;
}

void append_swath_heatmap_markers(
  visualization_msgs::msg::MarkerArray & array,
  const PlanningEngine & engine,
  const rclcpp::Time & stamp,
  const gpr_common::CoverageSegment & seg,
  const double sample_step_m,
  const double cube_width_factor,
  int & marker_id)
{
  if (seg.centerline.size() < 2U) {
    return;
  }
  const auto merged = gpr_common::merge_intervals(seg.covered_intervals_m);
  const double total = gpr_common::polyline_arc_length(seg.centerline);
  if (total <= 1e-6) {
    return;
  }

  visualization_msgs::msg::Marker covered;
  visualization_msgs::msg::Marker uncovered;
  for (auto * marker : {&covered, &uncovered}) {
    marker->header.frame_id = engine.frame_id();
    marker->header.stamp = stamp;
    marker->ns = "swath_heatmap";
    marker->type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker->action = visualization_msgs::msg::Marker::ADD;
    marker->scale.x = sample_step_m * cube_width_factor;
    marker->scale.y = 0.08;
    marker->scale.z = 0.04;
    marker->pose.orientation.w = 1.0;
    marker->color.a = 0.85f;
  }
  covered.id = marker_id++;
  covered.color.r = 0.1f;
  covered.color.g = 0.85f;
  covered.color.b = 0.1f;
  uncovered.id = marker_id++;
  uncovered.color.r = 1.0f;
  uncovered.color.g = 0.45f;
  uncovered.color.b = 0.0f;

  const int steps = std::max(1, static_cast<int>(std::ceil(total / sample_step_m)));
  for (int i = 0; i <= steps; ++i) {
    const double s = std::min(total, static_cast<double>(i) * sample_step_m);
    const auto slice = gpr_common::sub_polyline_by_arc_length(seg.centerline, s, s + 1e-6);
    if (slice.empty()) {
      continue;
    }
    geometry_msgs::msg::Point pt;
    pt.x = slice.front_pose().x;
    pt.y = slice.front_pose().y;
    pt.z = 0.05;
    if (arc_s_is_covered(s, merged)) {
      covered.points.push_back(pt);
    } else {
      uncovered.points.push_back(pt);
    }
  }

  if (!covered.points.empty()) {
    array.markers.push_back(covered);
  }
  if (!uncovered.points.empty()) {
    array.markers.push_back(uncovered);
  }
}
}  // namespace

void RosIoFacade::publish_markers(
  const PlanningEngine & engine, const bool reset_markers)
{
  visualization_msgs::msg::MarkerArray array;
  if (reset_markers) {
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
  }

  const bool have_grid = !engine.latest_grid().empty();
  const double viz_step = have_grid ?
    std::max(
      config_.segment_densify_step_min_m,
      engine.latest_grid().resolution * config_.segment_densify_resolution_factor) :
    config_.segment_densify_step_min_m;
  const double min_fragment_m = engine.catalog().config().min_split_length_m;
  const auto stamp = node_->now();

  if (engine.reporter().has_baseline()) {
    for (const auto & base : engine.reporter().baseline().segments) {
      if (!engine.show_completed_in_markers() &&
        baseline_viz_fully_completed(base, engine.catalog(), config_))
      {
        continue;
      }

      gpr_common::CoverageSegment style = base;
      style.outcome = gpr_common::SegmentOutcome::Pending;
      publish_swath_collision_markers(
        array, engine, stamp, style, base.centerline, viz_step, have_grid,
        min_fragment_m, config_, true);
    }
  }

  for (const auto & seg : engine.catalog().segments()) {
    if (seg.source != gpr_common::SegmentSource::OarpRank) {
      continue;
    }
    if (!engine.show_completed_in_markers() &&
      seg.outcome == gpr_common::SegmentOutcome::Completed)
    {
      continue;
    }
    publish_swath_collision_markers(
      array, engine, stamp, seg, seg.centerline, viz_step, have_grid,
      min_fragment_m, config_);
  }

  markers_->publish(array);
}

void RosIoFacade::publish_swath_heatmap(const PlanningEngine & engine)
{
  if (!swath_heatmap_) {
    return;
  }
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  const double sample_step = engine.latest_grid().empty() ? config_.heatmap_sample_step_m :
    std::max(config_.heatmap_sample_step_min_m, engine.latest_grid().resolution);
  const auto stamp = node_->now();
  int marker_id = 1;

  if (engine.reporter().has_baseline()) {
    for (const auto & base : engine.reporter().baseline().segments) {
      gpr_common::CoverageSegment heatmap_seg = base;
      heatmap_seg.covered_intervals_m = baseline_coverage_intervals(
        base, engine.catalog());
      append_swath_heatmap_markers(
        array, engine, stamp, heatmap_seg, sample_step,
        config_.heatmap_cube_width_factor, marker_id);
    }
  } else {
    for (const auto & seg : engine.catalog().segments()) {
      if (seg.source == gpr_common::SegmentSource::Baseline) {
        append_swath_heatmap_markers(
          array, engine, stamp, seg, sample_step,
          config_.heatmap_cube_width_factor, marker_id);
      }
    }
  }

  swath_heatmap_->publish(array);
}

void RosIoFacade::publish_updated_coverage_path(const PlanningEngine & engine)
{
  const auto strips = collect_remaining_coverage_strips(
    engine.catalog(), engine.latest_grid(), engine.invalidator_config());
  const auto stamp = node_->now();

  visualization_msgs::msg::MarkerArray strip_markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  strip_markers.markers.push_back(clear);

  nav_msgs::msg::Path path;
  path.header.frame_id = engine.frame_id();
  path.header.stamp = stamp;

  int marker_id = 1;
  for (const auto & strip : strips) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = engine.frame_id();
    line.header.stamp = stamp;
    line.ns = "updated_coverage";
    line.id = marker_id++;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = config_.marker_line_width;
    line.pose.position.z = 0.06;
    line.color.a = 1.0f;
    line.color.r = 1.0f;
    line.color.g = 128.0f / 255.0f;
    line.color.b = 0.0f;

    for (const auto & point : strip.points) {
      if (!pose_is_finite(point)) {
        continue;
      }
      geometry_msgs::msg::Point pt;
      pt.x = point.x;
      pt.y = point.y;
      line.points.push_back(pt);
    }
    if (line.points.size() >= 2U) {
      strip_markers.markers.push_back(line);
    }
  }

  // nav_msgs/Path connects poses sequentially (no disjoint-swath breaks). Prefer
  // /updated_coverage_path_strips (MarkerArray) in RViz for clean multi-swath lines.
  for (const auto & strip : strips) {
    for (const auto & point : strip.points) {
      append_pose_to_path(path, point, engine.frame_id(), stamp);
    }
  }

  updated_coverage_strips_->publish(strip_markers);
  updated_path_->publish(path);
}

void RosIoFacade::publish_transit_path(
  const PlanningEngine & engine, const gpr_common::Polyline & path)
{
  (void)engine;
  transit_path_->publish(to_path_msg(path, engine.frame_id(), node_->now()));
}

void RosIoFacade::publish_executed_trajectory(const PlanningEngine & engine)
{
  gpr_common::Polyline trace;
  {
    std::lock_guard<std::mutex> lock(engine.state_mutex_);
    trace = engine.mission_executed_trace_;
  }
  executed_path_->publish(to_path_msg(trace, engine.frame_id(), node_->now()));
}

void RosIoFacade::publish_schedule(const PlanningEngine & engine)
{
  if (!schedule_) {
    return;
  }
  const auto stamp = node_->now();
  schedule_->publish(to_ros_schedule(
    engine.schedule(), engine.catalog().revision(), engine.frame_id(), stamp));
}

void RosIoFacade::publish_coverage_report(const PlanningEngine & engine)
{
  if (!engine.reporter().has_baseline()) {
    return;
  }
  const auto metrics = engine.reporter().compute_metrics(engine.catalog().segments());
  const auto uncovered = engine.reporter().compute_uncovered(engine.catalog().segments());
  const auto stamp = node_->now();
  coverage_report_->publish(to_ros_report(
    metrics, engine.catalog().segments(), uncovered,
    engine.reporter().baseline().mission_id, engine.frame_id(), stamp));
  publish_uncovered_markers(engine, uncovered);

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), config_.coverage_report_log_throttle_ms,
    "coverage: %.1f%% swath arc (%.1f%% partial tail), %.1f%% blocked, "
    "%.1f%% skipped, %.1f%% remaining, lateral mean %.2f m",
    metrics.swath_coverage_pct, metrics.partial_pct, metrics.blocked_pct,
    metrics.skipped_pct, metrics.remaining_pct, metrics.mean_lateral_error_m);
}

void RosIoFacade::publish_uncovered_markers(
  const PlanningEngine & engine,
  const std::vector<gpr_metrics::UncoveredRegion> & regions)
{
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  int id = 0;
  for (const auto & region : regions) {
    visualization_msgs::msg::Marker box;
    box.header.frame_id = engine.frame_id();
    box.header.stamp = node_->now();
    box.ns = "uncovered_regions";
    box.id = id++;
    box.type = visualization_msgs::msg::Marker::LINE_STRIP;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.scale.x = 0.03;
    box.color.a = 0.9f;
    switch (region.reason) {
      case gpr_metrics::UncoveredReason::Blocked:
        box.color.r = 1.0f; box.color.g = 0.1f; box.color.b = 0.1f;
        break;
      case gpr_metrics::UncoveredReason::Skipped:
        box.color.r = 0.6f; box.color.g = 0.1f; box.color.b = 0.8f;
        break;
      default:
        box.color.r = 1.0f; box.color.g = 0.5f; box.color.b = 0.0f;
        break;
    }
    geometry_msgs::msg::Point p0;
    p0.x = region.bbox_x_min; p0.y = region.bbox_y_min;
    geometry_msgs::msg::Point p1;
    p1.x = region.bbox_x_max; p1.y = region.bbox_y_min;
    geometry_msgs::msg::Point p2;
    p2.x = region.bbox_x_max; p2.y = region.bbox_y_max;
    geometry_msgs::msg::Point p3;
    p3.x = region.bbox_x_min; p3.y = region.bbox_y_max;
    box.points = {p0, p1, p2, p3, p0};
    array.markers.push_back(box);
  }
  uncovered_markers_->publish(array);
}

}  // namespace gpr_planning

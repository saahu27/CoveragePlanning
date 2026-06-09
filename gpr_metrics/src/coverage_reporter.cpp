#include "gpr_metrics/coverage_reporter.hpp"

#include "gpr_metrics/coverage_accountant.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace gpr_metrics
{

void CoverageReporter::configure(
  gpr_common::ScanArea scan_area, double coverage_inset, double swath_width_m,
  MetricsConfig metrics_config)
{
  scan_area_ = scan_area;
  coverage_inset_ = coverage_inset;
  swath_width_m_ = swath_width_m;
  metrics_config_ = metrics_config;
  next_mission_id_ = metrics_config.mission_id_start;
}

void CoverageReporter::record_baseline(std::vector<gpr_common::CoverageSegment> master)
{
  baseline_ = CoverageBaseline::from_segments(
    std::move(master), scan_area_, coverage_inset_, swath_width_m_, next_mission_id_++);
}

CoverageMetrics CoverageReporter::compute_metrics(
  const std::vector<gpr_common::CoverageSegment> & current) const
{
  if (!baseline_.has_value()) {
    return {};
  }
  return CoverageAccountant::compute(*baseline_, current, metrics_config_);
}

std::vector<UncoveredRegion> CoverageReporter::compute_uncovered(
  const std::vector<gpr_common::CoverageSegment> & current) const
{
  return CoverageAccountant::group_uncovered(current, swath_width_m_);
}

namespace
{
const char * reason_label(UncoveredReason reason)
{
  switch (reason) {
    case UncoveredReason::Pending: return "pending";
    case UncoveredReason::Blocked: return "blocked";
    case UncoveredReason::Skipped: return "skipped";
  }
  return "unknown";
}
}  // namespace

std::string CoverageReporter::format_text_report(
  const CoverageMetrics & metrics,
  const std::vector<UncoveredRegion> & regions) const
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(1);
  if (baseline_.has_value()) {
    out << "Mission " << baseline_->mission_id << " coverage report\n";
  }
  out << "Baseline swath: " << metrics.baseline_swath_length_m << " m ("
      << metrics.segments_total << " segments)\n";
  out << "Coverage:  " << metrics.coverage_pct << "% swath arc ("
      << metrics.completed_swath_length_m << " m, "
      << metrics.segments_completed << " segments)\n";
  if (metrics.segments_partial > 0U || metrics.partial_swath_length_m > 0.0) {
    out << "Partial:   " << metrics.partial_pct << "% ("
        << metrics.partial_swath_length_m << " m tail, "
        << metrics.segments_partial << " segments)\n";
  }
  if (metrics.mean_lateral_error_m > 0.0) {
    out << "Lateral error: mean " << metrics.mean_lateral_error_m
        << " m, max " << metrics.max_lateral_error_m << " m\n";
  }
  out << "Blocked:   " << metrics.blocked_pct << "% ("
      << metrics.blocked_swath_length_m << " m, "
      << metrics.segments_blocked << " segments)\n";
  out << "Skipped:   " << metrics.skipped_pct << "% ("
      << metrics.skipped_swath_length_m << " m, "
      << metrics.segments_skipped << " segments)\n";
  out << "Remaining: " << metrics.remaining_pct << "% ("
      << metrics.pending_swath_length_m << " m, "
      << metrics.segments_pending << " segments)\n";
  out << "Plan retention: " << metrics.plan_retention_pct << "%\n";

  out << "\nUncovered regions (" << regions.size() << "):\n";
  for (const auto & region : regions) {
    out << "  lane " << region.lane_index << ", " << reason_label(region.reason)
        << ", " << region.segment_ids.size() << " segments, "
        << region.swath_length_m << " m, bbox [("
        << region.bbox_x_min << "," << region.bbox_y_min << ")-("
        << region.bbox_x_max << "," << region.bbox_y_max << ")]\n";
  }
  return out.str();
}

std::string CoverageReporter::make_export_path(
  const std::string & export_dir, uint64_t mission_id, int64_t sec, uint32_t nanosec)
{
  if (export_dir.empty()) {
    return {};
  }
  namespace fs = std::filesystem;
  fs::path dir(export_dir);
  if (!dir.is_absolute()) {
    if (const char * ws = std::getenv("ROS_WORKSPACE"); ws != nullptr && ws[0] != '\0') {
      dir = fs::path(ws) / dir;
    } else {
      dir = fs::current_path() / dir;
    }
  }
  std::error_code ec;
  fs::create_directories(dir, ec);
  std::ostringstream name;
  name << "coverage_report_" << mission_id << "_"
       << sec << "_" << std::setfill('0') << std::setw(9) << nanosec << ".txt";
  return (dir / name.str()).string();
}

bool CoverageReporter::export_text_report(
  const std::string & path,
  const CoverageMetrics & metrics,
  const std::vector<UncoveredRegion> & regions) const
{
  if (path.empty()) {
    return false;
  }
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
  }
  std::ofstream file(path);
  if (!file) {
    return false;
  }
  file << format_text_report(metrics, regions);
  return static_cast<bool>(file);
}

}  // namespace gpr_metrics

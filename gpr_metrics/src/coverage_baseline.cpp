#include "gpr_metrics/coverage_baseline.hpp"

namespace gpr_metrics
{

CoverageBaseline CoverageBaseline::from_segments(
  std::vector<gpr_common::CoverageSegment> master,
  gpr_common::ScanArea scan_area,
  double coverage_inset,
  double swath_width_m,
  uint64_t mission_id)
{
  CoverageBaseline baseline;
  baseline.mission_id = mission_id;
  baseline.scan_area = scan_area;
  baseline.coverage_inset = coverage_inset;
  baseline.swath_width_m = swath_width_m;
  baseline.segments = std::move(master);
  baseline.segment_count = static_cast<uint32_t>(baseline.segments.size());

  for (const auto & seg : baseline.segments) {
    baseline.planned_swath_length_m += seg.centerline.length();
  }
  baseline.planned_swath_area_m2 = baseline.planned_swath_length_m * swath_width_m;
  return baseline;
}

}  // namespace gpr_metrics

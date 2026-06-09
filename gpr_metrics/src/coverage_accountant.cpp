#include "gpr_metrics/coverage_accountant.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "gpr_common/swath_coverage.hpp"

namespace gpr_metrics
{

namespace
{
constexpr double kPctScale = 100.0;

double safe_pct(double part, double whole)
{
  if (whole <= 0.0) {
    return 0.0;
  }
  return (part / whole) * kPctScale;
}

void expand_bbox(
  const gpr_common::CoverageSegment & seg,
  double & x_min, double & y_min, double & x_max, double & y_max)
{
  for (const auto & p : seg.centerline.points) {
    x_min = std::min(x_min, p.x);
    y_min = std::min(y_min, p.y);
    x_max = std::max(x_max, p.x);
    y_max = std::max(y_max, p.y);
  }
}

double covered_arc_length_m(const gpr_common::CoverageSegment & seg) noexcept
{
  const double total = seg.centerline.length();
  if (seg.outcome == gpr_common::SegmentOutcome::Completed) {
    return total;
  }
  if (seg.outcome == gpr_common::SegmentOutcome::PartiallyCompleted) {
    return gpr_common::intervals_union_length(
      gpr_common::merge_intervals(seg.covered_intervals_m));
  }
  return 0.0;
}

struct BaselineRollup
{
  double completed{0.0};
  double partial_covered{0.0};
  double partial_pending{0.0};
  double blocked{0.0};
  double skipped{0.0};
  double pending{0.0};
};

BaselineRollup rollup_for_baseline(
  const gpr_common::SegmentId baseline_id,
  const std::vector<gpr_common::CoverageSegment> & current)
{
  BaselineRollup rollup;
  for (const auto & seg : current) {
    if (gpr_common::effective_baseline_id(seg) != baseline_id) {
      continue;
    }
    const double len = seg.centerline.length();
    switch (seg.outcome) {
      case gpr_common::SegmentOutcome::Completed:
        rollup.completed += len;
        break;
      case gpr_common::SegmentOutcome::PartiallyCompleted:
        rollup.partial_covered += covered_arc_length_m(seg);
        rollup.partial_pending += gpr_common::uncovered_arc_length_m(seg);
        break;
      case gpr_common::SegmentOutcome::Skipped:
        rollup.skipped += len;
        break;
      case gpr_common::SegmentOutcome::Pending:
        if (seg.blocked) {
          rollup.blocked += len;
        } else {
          rollup.pending += len;
        }
        break;
    }
  }
  return rollup;
}
}  // namespace

UncoveredReason CoverageAccountant::reason_for(
  const gpr_common::CoverageSegment & seg) noexcept
{
  if (seg.outcome == gpr_common::SegmentOutcome::Skipped) {
    return UncoveredReason::Skipped;
  }
  if (seg.outcome == gpr_common::SegmentOutcome::Pending && seg.blocked) {
    return UncoveredReason::Blocked;
  }
  return UncoveredReason::Pending;
}

CoverageMetrics CoverageAccountant::compute(
  const CoverageBaseline & baseline,
  const std::vector<gpr_common::CoverageSegment> & current,
  const MetricsConfig & metrics_config)
{
  CoverageMetrics metrics;
  metrics.baseline_swath_length_m = baseline.planned_swath_length_m;
  metrics.baseline_swath_area_m2 = baseline.planned_swath_area_m2;
  metrics.segments_total = static_cast<uint32_t>(baseline.segments.size());

  bool any_splits = false;
  for (const auto & seg : current) {
    if (seg.source == gpr_common::SegmentSource::Split ||
      seg.baseline_id.has_value())
    {
      any_splits = true;
      break;
    }
  }

  double lateral_sum = 0.0;
  std::size_t lateral_count = 0U;

  if (!any_splits) {
    for (const auto & seg : current) {
      const double len = seg.centerline.length();
      if (seg.last_mean_lateral_error_m > 0.0) {
        lateral_sum += seg.last_mean_lateral_error_m;
        ++lateral_count;
        metrics.max_lateral_error_m = std::max(
          metrics.max_lateral_error_m, seg.last_max_lateral_error_m);
      }
      switch (seg.outcome) {
        case gpr_common::SegmentOutcome::Completed:
          metrics.completed_swath_length_m += len;
          metrics.segments_completed++;
          break;
        case gpr_common::SegmentOutcome::PartiallyCompleted: {
          const double covered = covered_arc_length_m(seg);
          metrics.completed_swath_length_m += covered;
          metrics.partial_swath_length_m += gpr_common::uncovered_arc_length_m(seg);
          metrics.segments_partial++;
          metrics.pending_swath_length_m += gpr_common::uncovered_arc_length_m(seg);
          break;
        }
        case gpr_common::SegmentOutcome::Skipped:
          metrics.skipped_swath_length_m += len;
          metrics.segments_skipped++;
          break;
        case gpr_common::SegmentOutcome::Pending:
          if (seg.blocked) {
            metrics.blocked_swath_length_m += len;
            metrics.segments_blocked++;
          } else {
            metrics.pending_swath_length_m += len;
            metrics.segments_pending++;
          }
          break;
      }
    }
  } else {
    for (const auto & base_seg : baseline.segments) {
      const auto rollup = rollup_for_baseline(base_seg.id, current);
      const double base_len = base_seg.centerline.length();
      metrics.completed_swath_length_m += rollup.completed + rollup.partial_covered;
      metrics.partial_swath_length_m += rollup.partial_pending;
      metrics.skipped_swath_length_m += rollup.skipped;
      metrics.pending_swath_length_m += rollup.pending + rollup.partial_pending;

      const double child_total = rollup.completed + rollup.partial_covered +
        rollup.partial_pending + rollup.skipped + rollup.pending + rollup.blocked;
      const double implicit_blocked = std::max(0.0, base_len - child_total);
      metrics.blocked_swath_length_m += rollup.blocked + implicit_blocked;

      if (rollup.completed > 0.0) {
        metrics.segments_completed++;
      }
      if (rollup.partial_covered > 0.0 || rollup.partial_pending > 0.0) {
        metrics.segments_partial++;
      }
      if (rollup.pending > 0.0 || rollup.partial_pending > 0.0) {
        metrics.segments_pending++;
      }
      if (rollup.blocked > 0.0 ||
        implicit_blocked > metrics_config.implicit_blocked_length_epsilon_m)
      {
        metrics.segments_blocked++;
      }
      if (rollup.skipped > 0.0) {
        metrics.segments_skipped++;
      }
    }

    for (const auto & seg : current) {
      if (seg.last_mean_lateral_error_m > 0.0) {
        lateral_sum += seg.last_mean_lateral_error_m;
        ++lateral_count;
        metrics.max_lateral_error_m = std::max(
          metrics.max_lateral_error_m, seg.last_max_lateral_error_m);
      }
      if (seg.source != gpr_common::SegmentSource::OarpRank) {
        continue;
      }
      const double len = seg.centerline.length();
      switch (seg.outcome) {
        case gpr_common::SegmentOutcome::Completed:
          metrics.completed_swath_length_m += len;
          break;
        case gpr_common::SegmentOutcome::PartiallyCompleted:
          metrics.completed_swath_length_m += covered_arc_length_m(seg);
          metrics.partial_swath_length_m += gpr_common::uncovered_arc_length_m(seg);
          break;
        case gpr_common::SegmentOutcome::Skipped:
          metrics.skipped_swath_length_m += len;
          break;
        case gpr_common::SegmentOutcome::Pending:
          if (seg.blocked) {
            metrics.blocked_swath_length_m += len;
          } else {
            metrics.pending_swath_length_m += len;
          }
          break;
      }
    }
  }

  if (lateral_count > 0U) {
    metrics.mean_lateral_error_m = lateral_sum / static_cast<double>(lateral_count);
  }

  metrics.completed_swath_area_m2 =
    metrics.completed_swath_length_m * baseline.swath_width_m;

  const double baseline_len = metrics.baseline_swath_length_m;
  metrics.swath_coverage_pct = safe_pct(metrics.completed_swath_length_m, baseline_len);
  metrics.coverage_pct = metrics.swath_coverage_pct;
  metrics.partial_pct = safe_pct(metrics.partial_swath_length_m, baseline_len);
  metrics.blocked_pct = safe_pct(metrics.blocked_swath_length_m, baseline_len);
  metrics.skipped_pct = safe_pct(metrics.skipped_swath_length_m, baseline_len);
  metrics.remaining_pct = safe_pct(metrics.pending_swath_length_m, baseline_len);
  metrics.plan_retention_pct = safe_pct(
    metrics.completed_swath_length_m + metrics.pending_swath_length_m, baseline_len);

  return metrics;
}

std::vector<UncoveredRegion> CoverageAccountant::group_uncovered(
  const std::vector<gpr_common::CoverageSegment> & current,
  double swath_width_m)
{
  std::vector<UncoveredRegion> regions;
  if (current.empty()) {
    return regions;
  }

  struct SortKey
  {
    uint32_t lane;
    UncoveredReason reason;
    gpr_common::SegmentId id;
  };
  std::vector<SortKey> keys;
  keys.reserve(current.size());
  for (const auto & seg : current) {
    if (gpr_common::is_segment_covered(seg)) {
      continue;
    }
    keys.push_back({
      seg.lane_index.value_or(std::numeric_limits<uint32_t>::max()),
      reason_for(seg),
      seg.id,
    });
  }
  std::sort(keys.begin(), keys.end(), [](const SortKey & a, const SortKey & b) {
      if (a.lane != b.lane) {
        return a.lane < b.lane;
      }
      if (a.reason != b.reason) {
        return static_cast<uint8_t>(a.reason) < static_cast<uint8_t>(b.reason);
      }
      return a.id < b.id;
    });

  const auto find_seg = [&](gpr_common::SegmentId id) -> const gpr_common::CoverageSegment * {
      for (const auto & seg : current) {
        if (seg.id == id) {
          return &seg;
        }
      }
      return nullptr;
    };

  for (const auto & key : keys) {
    const gpr_common::CoverageSegment * seg = find_seg(key.id);
    if (seg == nullptr) {
      continue;
    }

    if (!regions.empty()) {
      auto & last = regions.back();
      if (last.lane_index == key.lane && last.reason == key.reason) {
        last.segment_ids.push_back(seg->id);
        last.swath_length_m += seg->centerline.length();
        last.swath_area_m2 = last.swath_length_m * swath_width_m;
        expand_bbox(
          *seg, last.bbox_x_min, last.bbox_y_min, last.bbox_x_max, last.bbox_y_max);
        continue;
      }
    }

    UncoveredRegion region;
    region.reason = key.reason;
    region.lane_index = key.lane;
    region.segment_ids.push_back(seg->id);
    region.swath_length_m = seg->centerline.length();
    region.swath_area_m2 = region.swath_length_m * swath_width_m;
    region.bbox_x_min = std::numeric_limits<double>::infinity();
    region.bbox_y_min = std::numeric_limits<double>::infinity();
    region.bbox_x_max = -std::numeric_limits<double>::infinity();
    region.bbox_y_max = -std::numeric_limits<double>::infinity();
    expand_bbox(*seg, region.bbox_x_min, region.bbox_y_min, region.bbox_x_max, region.bbox_y_max);
    regions.push_back(std::move(region));
  }

  return regions;
}

}  // namespace gpr_metrics

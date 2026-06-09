#include "gpr_common/swath_coverage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace gpr_common
{

namespace
{
constexpr double kEpsilon = 1e-9;

double normalize_angle(double yaw) noexcept
{
  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
}

double heading_error(const double a, const double b) noexcept
{
  return std::abs(normalize_angle(a - b));
}

/// @brief Allow forward or reverse travel along the same swath centerline.
double bidirectional_heading_error(const double heading_err_rad) noexcept
{
  return std::min(heading_err_rad, std::abs(M_PI - heading_err_rad));
}

void build_cumulative_lengths(
  const Polyline & line, std::vector<double> & cumulative) noexcept
{
  cumulative.clear();
  cumulative.reserve(line.points.size());
  double total = 0.0;
  cumulative.push_back(0.0);
  for (std::size_t i = 1U; i < line.points.size(); ++i) {
    const auto & a = line.points[i - 1U];
    const auto & b = line.points[i];
    total += std::hypot(b.x - a.x, b.y - a.y);
    cumulative.push_back(total);
  }
}
}  // namespace

double polyline_arc_length(const Polyline & line) noexcept
{
  if (line.size() < 2U) {
    return 0.0;
  }
  double total = 0.0;
  for (std::size_t i = 1U; i < line.points.size(); ++i) {
    const auto & a = line.points[i - 1U];
    const auto & b = line.points[i];
    total += std::hypot(b.x - a.x, b.y - a.y);
  }
  return total;
}

PolylineProjection project_point_onto_polyline(
  const Pose2D & point, const Polyline & line) noexcept
{
  PolylineProjection out;
  if (line.size() < 2U) {
    return out;
  }
  std::vector<double> cumulative;
  build_cumulative_lengths(line, cumulative);
  const double total = cumulative.back();
  double best_dist_sq = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  double best_yaw = line.points.front().yaw;
  for (std::size_t i = 0U; i + 1U < line.points.size(); ++i) {
    const auto & a = line.points[i];
    const auto & b = line.points[i + 1U];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len_sq = vx * vx + vy * vy;
    double t = 0.0;
    if (len_sq > kEpsilon) {
      t = ((point.x - a.x) * vx + (point.y - a.y) * vy) / len_sq;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double px = a.x + t * vx;
    const double py = a.y + t * vy;
    const double dx = px - point.x;
    const double dy = py - point.y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      const double seg_len = std::hypot(vx, vy);
      best_s = cumulative[i] + t * seg_len;
      best_yaw = a.yaw + t * (b.yaw - a.yaw);
    }
  }
  out.valid = true;
  out.arc_s_m = std::clamp(best_s, 0.0, total);
  out.lateral_m = std::sqrt(best_dist_sq);
  out.heading_err_rad = heading_error(point.yaw, best_yaw);
  return out;
}

std::vector<std::pair<double, double>> merge_intervals(
  std::vector<std::pair<double, double>> intervals) noexcept
{
  if (intervals.empty()) {
    return intervals;
  }
  std::sort(intervals.begin(), intervals.end());
  std::vector<std::pair<double, double>> merged;
  merged.reserve(intervals.size());
  double cur_lo = intervals.front().first;
  double cur_hi = intervals.front().second;
  for (std::size_t i = 1U; i < intervals.size(); ++i) {
    if (intervals[i].first <= cur_hi + kEpsilon) {
      cur_hi = std::max(cur_hi, intervals[i].second);
    } else {
      if (cur_hi > cur_lo + kEpsilon) {
        merged.emplace_back(cur_lo, cur_hi);
      }
      cur_lo = intervals[i].first;
      cur_hi = intervals[i].second;
    }
  }
  if (cur_hi > cur_lo + kEpsilon) {
    merged.emplace_back(cur_lo, cur_hi);
  }
  return merged;
}

double intervals_union_length(
  const std::vector<std::pair<double, double>> & intervals) noexcept
{
  double total = 0.0;
  for (const auto & iv : intervals) {
    total += std::max(0.0, iv.second - iv.first);
  }
  return total;
}

std::vector<std::pair<double, double>> complement_intervals(
  const std::vector<std::pair<double, double>> & covered,
  const double total_length_m) noexcept
{
  std::vector<std::pair<double, double>> uncovered;
  if (total_length_m <= kEpsilon) {
    return uncovered;
  }
  double cursor = 0.0;
  for (const auto & iv : covered) {
    const double lo = std::clamp(iv.first, 0.0, total_length_m);
    const double hi = std::clamp(iv.second, 0.0, total_length_m);
    if (lo > cursor + kEpsilon) {
      uncovered.emplace_back(cursor, lo);
    }
    cursor = std::max(cursor, hi);
  }
  if (cursor < total_length_m - kEpsilon) {
    uncovered.emplace_back(cursor, total_length_m);
  }
  return uncovered;
}

Pose2D point_at_arc_on_polyline(
  const Polyline & line, const double arc_s_m) noexcept
{
  if (line.size() < 2U) {
    return {};
  }
  std::vector<double> cumulative;
  build_cumulative_lengths(line, cumulative);
  const double total = cumulative.back();
  const double s = std::clamp(arc_s_m, 0.0, total);
  for (std::size_t i = 1U; i < cumulative.size(); ++i) {
    if (s <= cumulative[i] + kEpsilon) {
      const auto & a = line.points[i - 1U];
      const auto & b = line.points[i];
      const double seg_len = cumulative[i] - cumulative[i - 1U];
      double t = 0.0;
      if (seg_len > kEpsilon) {
        t = (s - cumulative[i - 1U]) / seg_len;
      }
      return Pose2D{
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.yaw + t * (b.yaw - a.yaw)};
    }
  }
  return line.points.back();
}

Polyline sub_polyline_by_arc_length(
  const Polyline & line, const double s0_m, const double s1_m) noexcept
{
  Polyline out;
  if (line.size() < 2U) {
    return out;
  }
  const double lo = std::min(s0_m, s1_m);
  const double hi = std::max(s0_m, s1_m);
  if (hi <= lo + kEpsilon) {
    return out;
  }
  std::vector<double> cumulative;
  build_cumulative_lengths(line, cumulative);
  const double total = cumulative.back();
  const double s0 = std::clamp(lo, 0.0, total);
  const double s1 = std::clamp(hi, 0.0, total);

  out.points.push_back(point_at_arc_on_polyline(line, s0));
  for (std::size_t i = 1U; i < cumulative.size(); ++i) {
    if (cumulative[i] > s0 + kEpsilon && cumulative[i] < s1 - kEpsilon) {
      out.points.push_back(line.points[i]);
    }
  }
  const auto end_pt = point_at_arc_on_polyline(line, s1);
  if (out.points.empty() ||
    std::hypot(end_pt.x - out.points.back().x, end_pt.y - out.points.back().y) > kEpsilon)
  {
    out.points.push_back(end_pt);
  }
  if (out.size() < 2U) {
    out.points.clear();
  }
  return out;
}

std::vector<std::pair<double, double>> map_covered_intervals_between_polylines(
  const std::vector<std::pair<double, double>> & src_intervals_m,
  const Polyline & src_line,
  const Polyline & dst_line) noexcept
{
  std::vector<std::pair<double, double>> mapped;
  if (src_intervals_m.empty() || src_line.size() < 2U || dst_line.size() < 2U) {
    return mapped;
  }
  const auto merged = merge_intervals(src_intervals_m);
  mapped.reserve(merged.size());
  for (const auto & iv : merged) {
    const auto p0 = point_at_arc_on_polyline(src_line, iv.first);
    const auto p1 = point_at_arc_on_polyline(src_line, iv.second);
    const auto proj0 = project_point_onto_polyline(p0, dst_line);
    const auto proj1 = project_point_onto_polyline(p1, dst_line);
    if (!proj0.valid || !proj1.valid) {
      continue;
    }
    mapped.emplace_back(
      std::min(proj0.arc_s_m, proj1.arc_s_m),
      std::max(proj0.arc_s_m, proj1.arc_s_m));
  }
  return merge_intervals(std::move(mapped));
}

SwathCoverageResult compute_swath_coverage(
  const Polyline & centerline,
  const Polyline & driven,
  const SwathCoverageConfig & config) noexcept
{
  SwathCoverageResult result;
  result.total_length_m = polyline_arc_length(centerline);
  if (result.total_length_m <= kEpsilon || driven.size() < 2U) {
    return result;
  }

  std::vector<std::pair<double, double>> raw_intervals;
  double lateral_sum = 0.0;
  std::optional<PolylineProjection> prev_on_swath;
  for (const auto & pose : driven.points) {
    const auto proj = project_point_onto_polyline(pose, centerline);
    if (!proj.valid) {
      prev_on_swath.reset();
      continue;
    }
    if (proj.lateral_m > config.lateral_max_m + kEpsilon) {
      prev_on_swath.reset();
      continue;
    }
    if (bidirectional_heading_error(proj.heading_err_rad) >
      config.heading_max_rad + kEpsilon)
    {
      prev_on_swath.reset();
      continue;
    }
    ++result.on_swath_sample_count;
    lateral_sum += proj.lateral_m;
    result.max_lateral_error_m = std::max(result.max_lateral_error_m, proj.lateral_m);
    const double half = std::max(config.sample_half_width_m, kEpsilon);
    raw_intervals.emplace_back(
      std::max(0.0, proj.arc_s_m - half),
      std::min(result.total_length_m, proj.arc_s_m + half));
    if (prev_on_swath.has_value()) {
      const double lo = std::min(prev_on_swath->arc_s_m, proj.arc_s_m);
      const double hi = std::max(prev_on_swath->arc_s_m, proj.arc_s_m);
      if (hi > lo + kEpsilon) {
        raw_intervals.emplace_back(lo, hi);
      }
    }
    prev_on_swath = proj;
  }

  result.covered_intervals_m = merge_intervals(std::move(raw_intervals));
  result.covered_length_m = intervals_union_length(result.covered_intervals_m);
  result.covered_fraction = result.covered_length_m / result.total_length_m;
  if (result.on_swath_sample_count > 0U) {
    result.mean_lateral_error_m = lateral_sum /
      static_cast<double>(result.on_swath_sample_count);
  }
  return result;
}

double interval_overlap_length(
  const std::vector<std::pair<double, double>> & a,
  const std::vector<std::pair<double, double>> & b) noexcept
{
  double overlap = 0.0;
  for (const auto & iv_a : a) {
    for (const auto & iv_b : b) {
      const double lo = std::max(iv_a.first, iv_b.first);
      const double hi = std::min(iv_a.second, iv_b.second);
      if (hi > lo + kEpsilon) {
        overlap += hi - lo;
      }
    }
  }
  return overlap;
}

SwathCoverageResult coverage_for_segment_from_pass(
  const Polyline & segment_centerline,
  const Polyline & pass_centerline,
  const SwathCoverageResult & pass_measured) noexcept
{
  SwathCoverageResult out;
  out.total_length_m = polyline_arc_length(segment_centerline);
  if (out.total_length_m <= kEpsilon || pass_centerline.size() < 2U ||
    pass_measured.covered_intervals_m.empty())
  {
    return out;
  }

  const auto proj0 = project_point_onto_polyline(
    segment_centerline.front_pose(), pass_centerline);
  const auto proj1 = project_point_onto_polyline(
    segment_centerline.back_pose(), pass_centerline);
  if (!proj0.valid || !proj1.valid) {
    return out;
  }

  const double span_lo = std::min(proj0.arc_s_m, proj1.arc_s_m);
  const double span_hi = std::max(proj0.arc_s_m, proj1.arc_s_m);
  const double span_len = span_hi - span_lo;
  if (span_len <= kEpsilon) {
    return out;
  }

  std::vector<std::pair<double, double>> seg_intervals;
  for (const auto & iv : pass_measured.covered_intervals_m) {
    const double lo = std::max(iv.first, span_lo);
    const double hi = std::min(iv.second, span_hi);
    if (hi <= lo + kEpsilon) {
      continue;
    }
    const double s0 = (lo - span_lo) / span_len * out.total_length_m;
    const double s1 = (hi - span_lo) / span_len * out.total_length_m;
    seg_intervals.emplace_back(
      std::max(0.0, s0), std::min(out.total_length_m, s1));
  }

  out.covered_intervals_m = merge_intervals(std::move(seg_intervals));
  out.covered_length_m = intervals_union_length(out.covered_intervals_m);
  out.covered_fraction = out.covered_length_m / out.total_length_m;
  out.mean_lateral_error_m = pass_measured.mean_lateral_error_m;
  out.max_lateral_error_m = pass_measured.max_lateral_error_m;
  return out;
}

}  // namespace gpr_common

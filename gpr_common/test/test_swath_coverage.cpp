#include <cmath>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/swath_coverage.hpp"
#include "gtest/gtest.h"

namespace
{
gpr_common::Polyline horizontal_line(const double x0, const double x1, const double y = 0.0)
{
  gpr_common::Polyline line;
  const double yaw = std::atan2(0.0, x1 - x0);
  line.points = {{x0, y, yaw}, {x1, y, yaw}};
  return line;
}

gpr_common::Polyline vertical_line(const double y0, const double y1, const double x = 0.0)
{
  gpr_common::Polyline line;
  const double yaw = std::atan2(y1 - y0, 0.0);
  line.points = {{x, y0, yaw}, {x, y1, yaw}};
  return line;
}

gpr_common::Polyline trace_along(
  const double x0, const double x1, const double y, const double step = 0.05)
{
  gpr_common::Polyline trace;
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    trace.points.push_back({x, y, 0.0});
  }
  return trace;
}

gpr_common::SwathCoverageConfig test_config()
{
  gpr_common::SwathCoverageConfig cfg;
  cfg.lateral_max_m = 0.22;
  cfg.heading_max_rad = 0.61;
  cfg.sample_half_width_m = 0.025;
  cfg.min_complete_fraction = 0.85;
  cfg.min_partial_fraction = 0.35;
  cfg.partial_enabled = true;
  return cfg;
}
}  // namespace

TEST(SwathCoverage, OffPathApproachDoesNotCountAsCovered)
{
  const auto centerline = horizontal_line(0.0, 4.0);
  const auto approach = trace_along(0.0, 3.5, 1.0);
  const auto cfg = test_config();
  const auto measured = gpr_common::compute_swath_coverage(centerline, approach, cfg);
  EXPECT_LT(measured.covered_fraction, cfg.min_partial_fraction);
  EXPECT_EQ(measured.on_swath_sample_count, 0U);
}

TEST(SwathCoverage, OnPathTraceAchievesHighCoverage)
{
  const auto centerline = horizontal_line(0.0, 4.0);
  const auto driven = trace_along(0.0, 4.0, 0.0);
  const auto cfg = test_config();
  const auto measured = gpr_common::compute_swath_coverage(centerline, driven, cfg);
  EXPECT_GE(measured.covered_fraction, cfg.min_complete_fraction);
  EXPECT_GT(measured.on_swath_sample_count, 0U);
}

TEST(SwathCoverage, ShortOnPathTraceIsPartialNotComplete)
{
  const auto centerline = horizontal_line(0.0, 4.0);
  const auto driven = trace_along(0.0, 1.5, 0.0);
  const auto cfg = test_config();
  const auto measured = gpr_common::compute_swath_coverage(centerline, driven, cfg);
  EXPECT_LT(measured.covered_fraction, cfg.min_complete_fraction);
  EXPECT_GE(measured.covered_fraction, cfg.min_partial_fraction);
}

TEST(SwathCoverage, ReverseTravelCountsOnSameCenterline)
{
  const auto centerline = horizontal_line(4.0, 0.0);
  gpr_common::Polyline driven;
  for (double x = 0.0; x <= 4.0 + 1e-9; x += 0.05) {
    driven.points.push_back({x, 0.0, 0.0});
  }
  const auto cfg = test_config();
  const auto measured = gpr_common::compute_swath_coverage(centerline, driven, cfg);
  EXPECT_GE(measured.covered_fraction, cfg.min_complete_fraction);
  EXPECT_GT(measured.on_swath_sample_count, 0U);
}

TEST(SwathCoverage, PassCoverageMapsOntoConstituentSegments)
{
  gpr_common::Polyline pass;
  for (double y = 0.0; y <= 2.0 + 1e-9; y += 0.1) {
    pass.points.push_back({0.0, y, M_PI_2});
  }
  const auto seg0 = vertical_line(0.0, 1.0);
  const auto seg1 = vertical_line(1.0, 2.0);

  gpr_common::Polyline driven;
  for (double y = 0.0; y <= 1.0 + 1e-9; y += 0.05) {
    driven.points.push_back({0.0, y, M_PI_2});
  }
  const auto cfg = test_config();
  const auto pass_measured = gpr_common::compute_swath_coverage(pass, driven, cfg);
  const auto mapped0 = gpr_common::coverage_for_segment_from_pass(seg0, pass, pass_measured);
  const auto mapped1 = gpr_common::coverage_for_segment_from_pass(seg1, pass, pass_measured);

  EXPECT_GE(mapped0.covered_fraction, cfg.min_complete_fraction);
  EXPECT_LT(mapped1.covered_fraction, cfg.min_partial_fraction);
}

TEST(SwathCoverage, MapCoveredIntervalsProjectsArcEndpoints)
{
  const auto src = horizontal_line(1.0, 3.0);
  const auto dst = horizontal_line(1.0, 4.0);
  const std::vector<std::pair<double, double>> src_iv{{0.5, 1.5}};
  const auto mapped = gpr_common::map_covered_intervals_between_polylines(
    src_iv, src, dst);
  ASSERT_EQ(mapped.size(), 1U);
  EXPECT_NEAR(mapped.front().first, 0.5, 0.05);
  EXPECT_NEAR(mapped.front().second, 1.5, 0.05);
}

TEST(SwathCoverage, MapCoveredIntervalsOntoVerticalLanePiece)
{
  const auto lane_piece = vertical_line(1.0, 2.0, 0.0);
  const auto full_lane = vertical_line(0.0, 3.0, 0.0);
  const std::vector<std::pair<double, double>> covered{{0.0, 1.0}};
  const auto mapped = gpr_common::map_covered_intervals_between_polylines(
    covered, lane_piece, full_lane);
  ASSERT_EQ(mapped.size(), 1U);
  EXPECT_NEAR(mapped.front().first, 1.0, 0.05);
  EXPECT_NEAR(mapped.front().second, 2.0, 0.05);
}

TEST(SwathCoverage, SchedulableTailAfterPartialCoverage)
{
  gpr_common::CoverageSegment seg;
  seg.centerline = horizontal_line(0.0, 4.0);
  seg.outcome = gpr_common::SegmentOutcome::PartiallyCompleted;
  seg.covered_intervals_m = {{0.0, 1.2}};
  const auto tail = gpr_common::schedulable_centerline(
    seg, gpr_common::DriveDirection::Forward);
  ASSERT_GE(tail.size(), 2U);
  EXPECT_NEAR(tail.front_pose().x, 1.2, 0.15);
  EXPECT_NEAR(tail.back_pose().x, 4.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

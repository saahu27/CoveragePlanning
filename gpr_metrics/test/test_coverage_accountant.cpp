#include "gpr_common/coverage_types.hpp"
#include "gpr_metrics/coverage_accountant.hpp"
#include "gpr_metrics/coverage_baseline.hpp"
#include "gtest/gtest.h"

namespace
{
gpr_common::CoverageSegment make_seg(
  uint32_t lane, uint32_t ord, double len,
  gpr_common::SegmentOutcome outcome = gpr_common::SegmentOutcome::Pending,
  bool blocked = false)
{
  gpr_common::CoverageSegment seg;
  seg.id = gpr_common::make_segment_id(lane, ord);
  seg.lane_index = lane;
  seg.centerline.points = {{0.0, 0.0, 0.0}, {len, 0.0, 0.0}};
  seg.outcome = outcome;
  seg.blocked = blocked;
  return seg;
}
}  // namespace

TEST(CoverageAccountant, ComputesLengthWeightedPercentages)
{
  auto master = {
    make_seg(0, 0, 1.0),
    make_seg(0, 1, 1.0),
    make_seg(1, 0, 2.0),
    make_seg(1, 1, 2.0),
  };
  const auto baseline = gpr_metrics::CoverageBaseline::from_segments(
    master, {-4, 4, -2.5, 2.5}, 0.25, 0.5);

  std::vector<gpr_common::CoverageSegment> current = {
    make_seg(0, 0, 1.0, gpr_common::SegmentOutcome::Completed),
    make_seg(0, 1, 1.0, gpr_common::SegmentOutcome::Skipped),
    make_seg(1, 0, 2.0, gpr_common::SegmentOutcome::Pending, true),
    make_seg(1, 1, 2.0, gpr_common::SegmentOutcome::Pending, false),
  };

  const auto metrics = gpr_metrics::CoverageAccountant::compute(baseline, current);
  EXPECT_NEAR(metrics.baseline_swath_length_m, 6.0, 1e-6);
  EXPECT_NEAR(metrics.completed_swath_length_m, 1.0, 1e-6);
  EXPECT_NEAR(metrics.skipped_swath_length_m, 1.0, 1e-6);
  EXPECT_NEAR(metrics.blocked_swath_length_m, 2.0, 1e-6);
  EXPECT_NEAR(metrics.pending_swath_length_m, 2.0, 1e-6);
  EXPECT_NEAR(metrics.coverage_pct, 100.0 / 6.0 * 1.0, 1e-6);
  EXPECT_NEAR(metrics.plan_retention_pct, 100.0 / 6.0 * 3.0, 1e-6);
  EXPECT_EQ(metrics.segments_completed, 1U);
  EXPECT_EQ(metrics.segments_skipped, 1U);
  EXPECT_EQ(metrics.segments_blocked, 1U);
  EXPECT_EQ(metrics.segments_pending, 1U);
}

TEST(CoverageAccountant, GroupsUncoveredByLaneAndReason)
{
  std::vector<gpr_common::CoverageSegment> current = {
    make_seg(2, 0, 1.0, gpr_common::SegmentOutcome::Pending, true),
    make_seg(2, 1, 1.0, gpr_common::SegmentOutcome::Pending, true),
    make_seg(3, 0, 1.0, gpr_common::SegmentOutcome::Skipped),
  };
  const auto regions = gpr_metrics::CoverageAccountant::group_uncovered(current, 0.5);
  ASSERT_EQ(regions.size(), 2U);
  EXPECT_EQ(regions[0].lane_index, 2U);
  EXPECT_EQ(regions[0].reason, gpr_metrics::UncoveredReason::Blocked);
  EXPECT_EQ(regions[0].segment_ids.size(), 2U);
  EXPECT_EQ(regions[1].reason, gpr_metrics::UncoveredReason::Skipped);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

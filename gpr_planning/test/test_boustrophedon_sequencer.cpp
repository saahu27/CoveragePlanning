#include "gpr_planning/boustrophedon_sequencer.hpp"

#include "gpr_common/scan_region.hpp"
#include "gtest/gtest.h"

namespace
{
gpr_common::CoverageSegment make_open(
  uint32_t lane, uint32_t ordinal, double x0, double y0, double x1, double y1)
{
  gpr_common::CoverageSegment seg;
  seg.centerline.points = {{x0, y0, 0.0}, {x1, y1, 0.0}};
  seg.lane_index = lane;
  seg.id = gpr_common::make_segment_id(lane, ordinal);
  return seg;
}
}  // namespace

TEST(BoustrophedonSequencer, MergesThreeCollinearSegmentsOnLane)
{
  gpr_planning::BoustrophedonConfig bconfig;
  bconfig.region = gpr_common::ScanRegion::from_rectangle(gpr_common::ScanArea{-4.0, 4.0, -2.5, 2.5});
  bconfig.scan_direction = gpr_common::ScanDirection::kX;

  std::vector<gpr_common::CoverageSegment> schedulable = {
    make_open(0, 0, -3.75, -2.0, -3.75, -1.0),
    make_open(0, 1, -3.75, -1.0, -3.75, 0.0),
    make_open(0, 2, -3.75, 0.0, -3.75, 1.0),
  };

  gpr_planning::BoustrophedonSequencer sequencer;
  const auto passes = sequencer.plan(schedulable, bconfig);
  ASSERT_EQ(passes.size(), 1U);
  EXPECT_EQ(passes.front().segment_ids.size(), 3U);
  EXPECT_NEAR(passes.front().centerline.length(), 3.0, 0.15);
}

TEST(BoustrophedonSequencer, OrdersLanesFromBoundaryOutward)
{
  gpr_planning::BoustrophedonConfig bconfig;
  bconfig.region = gpr_common::ScanRegion::from_rectangle(gpr_common::ScanArea{-4.0, 4.0, -2.5, 2.5});
  bconfig.scan_direction = gpr_common::ScanDirection::kX;

  std::vector<gpr_common::CoverageSegment> schedulable = {
    make_open(2, 0, -2.75, -2.0, -2.75, -1.0),
    make_open(0, 0, -3.75, -2.0, -3.75, -1.0),
    make_open(1, 0, -3.25, -2.0, -3.25, -1.0),
  };

  gpr_planning::BoustrophedonSequencer sequencer;
  const auto passes = sequencer.plan(schedulable, bconfig);
  ASSERT_EQ(passes.size(), 3U);
  EXPECT_EQ(passes[0].lane_index, 0U);
  EXPECT_EQ(passes[1].lane_index, 1U);
  EXPECT_EQ(passes[2].lane_index, 2U);
}

TEST(BoustrophedonSequencer, ChoosesNearerEntryDirectionAndOrder)
{
  gpr_planning::BoustrophedonConfig bconfig;
  bconfig.region = gpr_common::ScanRegion::from_rectangle(gpr_common::ScanArea{-4.0, 4.0, -2.5, 2.5});
  bconfig.scan_direction = gpr_common::ScanDirection::kX;

  // Only the bottom portion of lane 1 remains (short up-pass from y=-2 to y=-1).
  std::vector<gpr_common::CoverageSegment> schedulable = {
    make_open(1, 0, -3.25, -2.0, -3.25, -1.0),
  };

  gpr_planning::BoustrophedonSequencer sequencer;
  const gpr_common::Pose2D robot{-3.25, -1.9, 0.0};
  const auto passes = sequencer.plan(schedulable, bconfig, robot);
  ASSERT_EQ(passes.size(), 1U);
  // Merged geometry on lane 1 is oriented high→low; robot near bottom → enter at y=-2.
  EXPECT_EQ(passes.front().job.direction, gpr_common::DriveDirection::Reverse);
  gpr_common::CoverageSegment tmp;
  tmp.centerline = passes.front().centerline;
  const auto entry = gpr_common::job_entry_pose(tmp, passes.front().job.direction);
  EXPECT_NEAR(entry.y, -2.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

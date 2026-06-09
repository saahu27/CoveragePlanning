#include "gpr_common/scan_region.hpp"
#include "gpr_planning/oarp_lite_planner.hpp"
#include "gtest/gtest.h"

namespace
{
gpr_common::CoverageSegment make_baseline(
  uint32_t lane, uint32_t ordinal, double x0, double y0, double x1, double y1,
  bool blocked)
{
  gpr_common::CoverageSegment seg;
  seg.centerline.points = {{x0, y0, 0.0}, {x1, y1, 0.0}};
  seg.lane_index = lane;
  seg.id = gpr_common::make_segment_id(lane, ordinal);
  seg.blocked = blocked;
  return seg;
}
}  // namespace

TEST(OarpLitePlanner, GeneratesRankWhenLaneFullyBlocked)
{
  gpr_planning::BoustrophedonConfig bconfig;
  bconfig.region = gpr_common::ScanRegion::from_rectangle(gpr_common::ScanArea{-1.0, 1.0, -1.0, 1.0});
  bconfig.lane_spacing = 1.0;
  bconfig.waypoint_spacing = 0.2;
  bconfig.segment_length = 0.5;
  bconfig.coverage_inset = 0.0;

  gpr_planning::OarpLiteConfig oconfig;
  oconfig.enabled = true;
  oconfig.min_rank_length_m = 0.2;
  gpr_planning::OarpLitePlanner planner(bconfig, oconfig);

  gpr_common::GridMap grid;
  grid.resolution = 0.1;
  grid.width = 20;
  grid.height = 20;
  grid.origin_x = -1.0;
  grid.origin_y = -1.0;
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);

  std::vector<gpr_common::CoverageSegment> catalog;
  catalog.push_back(make_baseline(0, 0, -0.5, -0.8, -0.5, 0.8, true));

  gpr_planning::PathInvalidatorConfig inv;
  const auto ranks = planner.generate_ranks(grid, inv, catalog, 1U);
  EXPECT_FALSE(ranks.empty());
  EXPECT_EQ(ranks.front().source, gpr_common::SegmentSource::OarpRank);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

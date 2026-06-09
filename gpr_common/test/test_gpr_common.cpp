#include "gpr_common/coverage_types.hpp"
#include "gpr_common/grid_map.hpp"
#include "gpr_common/types.hpp"

#include "gtest/gtest.h"

TEST(GprCommon, PolylineLength)
{
  gpr_common::Polyline line;
  line.points = {{0, 0, 0}, {3, 4, 0}};
  EXPECT_NEAR(line.length(), 5.0, 1e-6);
}

TEST(GprCommon, JobPolylineReverse)
{
  gpr_common::CoverageSegment seg;
  seg.centerline.points = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
  const auto fwd = gpr_common::job_polyline(seg, gpr_common::DriveDirection::Forward);
  const auto rev = gpr_common::job_polyline(seg, gpr_common::DriveDirection::Reverse);
  EXPECT_NEAR(fwd.points.front().x, 0.0, 1e-6);
  EXPECT_NEAR(rev.points.front().x, 2.0, 1e-6);
}

TEST(GprCommon, SegmentIdStable)
{
  gpr_common::Polyline line;
  line.points = {{0, 0, 0}, {1, 0, 0}};
  const auto id1 = gpr_common::compute_segment_id(line, 0U);
  const auto id2 = gpr_common::compute_segment_id(line, 0U);
  EXPECT_EQ(id1, id2);
}

TEST(GprCommon, TrimPolylineAhead)
{
  gpr_common::Polyline line;
  line.points = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}};
  const gpr_common::Pose2D robot{2.1, 0.0, 0.0};
  const auto trimmed = gpr_common::trim_polyline_ahead(line, robot, 0.15);
  ASSERT_GE(trimmed.points.size(), 2U);
  EXPECT_NEAR(trimmed.points.front().x, 2.1, 1e-6);
  EXPECT_NEAR(trimmed.points.back().x, 4.0, 1e-6);
}

TEST(GprCommon, TrimPolylineAheadAtEnd)
{
  gpr_common::Polyline line;
  line.points = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
  const gpr_common::Pose2D robot{2.0, 0.0, 0.0};
  const auto trimmed = gpr_common::trim_polyline_ahead(line, robot, 0.15);
  EXPECT_LT(trimmed.size(), 2U);
}

TEST(GprCommon, GridMapCollision)
{
  gpr_common::GridMap grid;
  grid.resolution = 0.1;
  grid.width = 10;
  grid.height = 10;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.data.assign(100, 0);
  grid.data[55] = 100;

  int8_t cost = 0;
  EXPECT_TRUE(gpr_common::GridMapUtils::occupancy_cost_at(grid, 0.55, 0.55, cost));
  EXPECT_EQ(cost, 100);
  EXPECT_TRUE(gpr_common::GridMapUtils::is_pose_colliding(grid, 0.55, 0.55, 50, 0.0, true));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

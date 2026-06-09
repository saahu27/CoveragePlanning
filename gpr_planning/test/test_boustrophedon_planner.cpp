#include "gpr_planning/boustrophedon_planner.hpp"

#include "gpr_common/polygon_geometry.hpp"
#include "gpr_common/scan_region.hpp"
#include "gtest/gtest.h"

namespace
{

gpr_common::ScanRegion default_test_region()
{
  return gpr_common::ScanRegion::from_rectangle(gpr_common::ScanArea{-4, 4, -2.5, 2.5});
}

}  // namespace

TEST(BoustrophedonPlanner, GeneratesPath)
{
  gpr_planning::BoustrophedonConfig config;
  config.region = default_test_region();
  config.coverage_inset = 0.25;
  gpr_planning::BoustrophedonPlanner planner(config);
  const auto path = planner.generate();
  EXPECT_GT(path.size(), 10U);
}

TEST(BoustrophedonPlanner, GeneratesStableSegments)
{
  gpr_planning::BoustrophedonConfig config;
  config.region = default_test_region();
  config.coverage_inset = 0.25;
  config.segment_length = 1.0;
  gpr_planning::BoustrophedonPlanner planner(config);

  const auto segments = planner.generate_segments();
  ASSERT_FALSE(segments.empty());

  // Each segment must be a usable 2+ point edge with a lane index.
  for (const auto & seg : segments) {
    EXPECT_GE(seg.centerline.size(), 2U);
    EXPECT_TRUE(seg.lane_index.has_value());
  }

  // IDs are unique and deterministic (stable) across calls.
  const auto again = planner.generate_segments();
  ASSERT_EQ(segments.size(), again.size());
  for (std::size_t i = 0; i < segments.size(); ++i) {
    EXPECT_EQ(segments[i].id, again[i].id);
  }

  // A ~5 m lane split at 1 m must produce multiple sub-segments per lane.
  EXPECT_GT(segments.size(), 10U);
}

TEST(BoustrophedonPlanner, GeneratesLaneConnectors)
{
  gpr_planning::BoustrophedonConfig config;
  config.region = default_test_region();
  config.coverage_inset = 0.25;
  config.lane_spacing = 0.5;
  gpr_planning::BoustrophedonPlanner planner(config);
  const auto connectors = planner.generate_connectors();
  const auto lanes = planner.generate_lane_centerlines();
  ASSERT_GE(lanes.size(), 2U);
  EXPECT_EQ(connectors.size(), lanes.size() - 1U);
  for (const auto & connector : connectors) {
    EXPECT_GE(connector.size(), 2U);
  }
}

TEST(BoustrophedonPlanner, RectanglePolygonMatchesLegacyLaneCount)
{
  gpr_planning::BoustrophedonConfig config;
  config.region = default_test_region();
  config.coverage_inset = 0.25;
  config.lane_spacing = 0.5;
  config.segment_length = 1.0;
  gpr_planning::BoustrophedonPlanner planner(config);
  const auto segments = planner.generate_segments();
  ASSERT_FALSE(segments.empty());
  // 16 inset lanes x ceil(4.5 m / 1 m segment_length) = 80 (legacy rectangle parity).
  EXPECT_EQ(segments.size(), 80U);
  for (const auto & seg : segments) {
    const auto & p = seg.centerline.points.front();
    EXPECT_TRUE(gpr_common::point_in_polygon(gpr_common::Point2D{p.x, p.y}, config.region.vertices));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

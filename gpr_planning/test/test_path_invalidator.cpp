#include "gpr_planning/path_invalidator.hpp"
#include "gtest/gtest.h"

TEST(PathInvalidator, SplitsOnObstacle)
{
  gpr_common::Polyline path;
  for (int i = 0; i <= 10; ++i) {
    path.points.push_back({static_cast<double>(i) * 0.1, 0.0, 0.0});
  }
  gpr_common::GridMap grid;
  grid.resolution = 0.05;
  grid.width = 20;
  grid.height = 5;
  grid.origin_x = 0.0;
  grid.origin_y = -0.5;
  grid.data.assign(100, 0);
  grid.data[60] = 100;

  gpr_planning::PathInvalidatorConfig config;
  const auto segments = gpr_planning::PathInvalidator::invalidate_segments(path, grid, config);
  EXPECT_GE(segments.size(), 1U);
}

TEST(PathInvalidator, FreeSubpolylinesSplitPartialBlock)
{
  gpr_common::Polyline path;
  for (int i = 0; i <= 40; ++i) {
    path.points.push_back({static_cast<double>(i) * 0.05, 0.0, 0.0});
  }
  gpr_common::GridMap grid;
  grid.resolution = 0.05;
  grid.width = 50;
  grid.height = 10;
  grid.origin_x = 0.0;
  grid.origin_y = -0.25;
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  const int mx = 20;
  const int my = 5;
  for (int dy = -1; dy <= 1; ++dy) {
    grid.data[static_cast<std::size_t>((my + dy) * grid.width + mx)] = 100;
  }

  gpr_planning::PathInvalidatorConfig config;
  config.footprint_radius = 0.08;
  const auto parts = gpr_planning::PathInvalidator::free_subpolylines(path, grid, config);
  EXPECT_GE(parts.size(), 2U);
}

TEST(PathInvalidator, BlockedSubpolylinesComplementFree)
{
  gpr_common::Polyline path;
  for (int i = 0; i <= 40; ++i) {
    path.points.push_back({static_cast<double>(i) * 0.05, 0.0, 0.0});
  }
  gpr_common::GridMap grid;
  grid.resolution = 0.05;
  grid.width = 50;
  grid.height = 10;
  grid.origin_x = 0.0;
  grid.origin_y = -0.25;
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  const int mx = 20;
  const int my = 5;
  for (int dy = -1; dy <= 1; ++dy) {
    grid.data[static_cast<std::size_t>((my + dy) * grid.width + mx)] = 100;
  }

  gpr_planning::PathInvalidatorConfig config;
  config.footprint_radius = 0.08;
  const auto free_parts = gpr_planning::PathInvalidator::free_subpolylines(path, grid, config);
  const auto blocked_parts = gpr_planning::PathInvalidator::blocked_subpolylines(
    path, grid, config);
  EXPECT_GE(free_parts.size(), 2U);
  EXPECT_GE(blocked_parts.size(), 1U);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

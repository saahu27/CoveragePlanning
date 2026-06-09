#include <gtest/gtest.h>

#include "gpr_common/polygon_geometry.hpp"
#include "gpr_common/scan_region.hpp"

TEST(PolygonGeometry, RectangleChordMatchesVerticalSpan)
{
  const auto region = gpr_common::ScanRegion::from_rectangle(gpr_common::ScanArea{0.0, 4.0, -1.0, 1.0});
  const auto inset = region.inset(0.25);
  const auto chords = gpr_common::chords_on_scan_line(
    inset.vertices, gpr_common::ScanDirection::kX, 2.0);
  ASSERT_EQ(chords.size(), 1U);
  EXPECT_NEAR(chords.front().a.y, -0.75, 1e-6);
  EXPECT_NEAR(chords.front().b.y, 0.75, 1e-6);
}

TEST(PolygonGeometry, RectangleChordOnRightBoundary)
{
  const auto region = gpr_common::ScanRegion::from_rectangle(gpr_common::ScanArea{-3.75, 3.75, -2.25, 2.25});
  const auto chords = gpr_common::chords_on_scan_line(
    region.vertices, gpr_common::ScanDirection::kX, region.bounds.x_max);
  ASSERT_EQ(chords.size(), 1U);
  EXPECT_NEAR(chords.front().a.y, region.bounds.y_min, 1e-6);
  EXPECT_NEAR(chords.front().b.y, region.bounds.y_max, 1e-6);
}

TEST(PolygonGeometry, PointInAxisAlignedRectangle)
{
  const auto region = gpr_common::ScanRegion::from_rectangle(gpr_common::ScanArea{-1.0, 1.0, -1.0, 1.0});
  EXPECT_TRUE(gpr_common::point_in_polygon(gpr_common::Point2D{0.0, 0.0}, region.vertices));
  EXPECT_FALSE(gpr_common::point_in_polygon(gpr_common::Point2D{2.0, 0.0}, region.vertices));
}

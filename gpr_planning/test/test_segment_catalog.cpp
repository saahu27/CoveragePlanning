#include "gpr_common/scan_region.hpp"
#include "gpr_common/swath_coverage.hpp"
#include "gpr_planning/segment_catalog.hpp"
#include "gtest/gtest.h"

namespace
{
gpr_common::CoverageSegment make_segment(
  uint32_t lane, uint32_t ordinal, double x0, double y0, double x1, double y1)
{
  gpr_common::CoverageSegment seg;
  seg.centerline.points = {{x0, y0, 0.0}, {x1, y1, 0.0}};
  seg.lane_index = lane;
  seg.id = gpr_common::make_segment_id(lane, ordinal);
  return seg;
}

gpr_common::GridMap empty_grid()
{
  gpr_common::GridMap grid;
  grid.resolution = 0.05;
  grid.width = 40;
  grid.height = 40;
  grid.origin_x = -1.0;
  grid.origin_y = -1.0;
  grid.data.assign(static_cast<std::size_t>(grid.width) * grid.height, 0);
  return grid;
}
}  // namespace

TEST(SegmentCatalog, CompletedPersistsAcrossUpdates)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 1.0, 0.0);
  catalog.initialize({seg});
  ASSERT_EQ(catalog.open_segments().size(), 1U);

  catalog.mark_completed(seg.id);
  EXPECT_TRUE(catalog.open_segments().empty());

  // A subsequent grid update must NOT resurrect the completed segment.
  catalog.update_blocked(empty_grid(), {});
  EXPECT_TRUE(catalog.open_segments().empty());
  ASSERT_NE(catalog.find(seg.id), nullptr);
  EXPECT_EQ(catalog.find(seg.id)->outcome, gpr_common::SegmentOutcome::Completed);
}

TEST(SegmentCatalog, BlockedTogglesWithGrid)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 0.5, 0.0);
  catalog.initialize({seg});

  gpr_planning::PathInvalidatorConfig config;
  auto grid = empty_grid();
  catalog.update_blocked(grid, config);
  EXPECT_EQ(catalog.open_segments().size(), 1U);

  // Drop an obstacle onto the segment -> becomes blocked -> excluded.
  for (auto & cell : grid.data) {
    cell = 100;
  }
  catalog.update_blocked(grid, config);
  EXPECT_TRUE(catalog.open_segments().empty());
  ASSERT_NE(catalog.find(seg.id), nullptr);
  EXPECT_TRUE(catalog.find(seg.id)->blocked);

  // Clear it -> open again (re-coverable).
  for (auto & cell : grid.data) {
    cell = 0;
  }
  catalog.update_blocked(grid, config);
  EXPECT_EQ(catalog.open_segments().size(), 1U);
}

TEST(SegmentCatalog, SkippedExcludedFromOpenAndMetrics)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 1.0, 0.0);
  catalog.initialize({seg});
  catalog.mark_job_skipped(seg.id);
  EXPECT_TRUE(catalog.open_segments().empty());
  ASSERT_NE(catalog.find(seg.id), nullptr);
  EXPECT_EQ(catalog.find(seg.id)->outcome, gpr_common::SegmentOutcome::Skipped);
  EXPECT_FALSE(gpr_common::is_segment_covered(*catalog.find(seg.id)));
}

TEST(SegmentCatalog, MarkBlockedExcludesFromOpen)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 1.0, 0.0);
  catalog.initialize({seg});
  catalog.mark_blocked(seg.id);
  EXPECT_TRUE(catalog.open_segments().empty());
  ASSERT_NE(catalog.find(seg.id), nullptr);
  EXPECT_TRUE(catalog.find(seg.id)->blocked);
  EXPECT_EQ(catalog.find(seg.id)->outcome, gpr_common::SegmentOutcome::Pending);
}

TEST(SegmentCatalog, BlockedUnattemptedIsProbeSchedulable)
{
  gpr_planning::SegmentCatalogConfig config;
  config.schedule_blocked_probes = true;
  gpr_planning::SegmentCatalog catalog(config);
  auto seg = make_segment(0, 0, 0.0, 0.0, 1.0, 0.0);
  catalog.initialize({seg});
  catalog.mark_blocked(seg.id);
  EXPECT_TRUE(catalog.open_segments().empty());
  EXPECT_EQ(catalog.schedulable_segments().size(), 1U);
}

TEST(SegmentCatalog, AttemptedBlockedNotResurrectedBySplit)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 2.0, 0.0);
  seg.centerline.points = {
    {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {1.0, 0.0, 0.0},
    {1.5, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  catalog.initialize({seg});
  catalog.mark_attempted(seg.id);
  catalog.mark_blocked(seg.id);
  EXPECT_TRUE(catalog.open_segments().empty());
  EXPECT_TRUE(catalog.schedulable_segments().empty());

  gpr_planning::PathInvalidatorConfig config;
  config.footprint_radius = 0.08;
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

  catalog.update_blocked(grid, config);
  EXPECT_EQ(catalog.segments().size(), 1U);
  EXPECT_TRUE(catalog.open_segments().empty());
  EXPECT_TRUE(catalog.schedulable_segments().empty());
  ASSERT_NE(catalog.find(seg.id), nullptr);
  EXPECT_TRUE(catalog.find(seg.id)->blocked);
  EXPECT_TRUE(catalog.find(seg.id)->attempted);
}

TEST(SegmentCatalog, JobStillActionableAfterSplit)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 2.0, 0.0);
  seg.centerline.points = {
    {0.0, 0.0, 0.0}, {0.25, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.75, 0.0, 0.0},
    {1.0, 0.0, 0.0}, {1.25, 0.0, 0.0}, {1.5, 0.0, 0.0}, {1.75, 0.0, 0.0},
    {2.0, 0.0, 0.0}};
  const auto original_id = seg.id;
  catalog.initialize({seg});

  gpr_planning::PathInvalidatorConfig config;
  config.footprint_radius = 0.08;
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

  catalog.update_blocked(grid, config);
  EXPECT_EQ(catalog.find(original_id), nullptr);

  gpr_common::CoverageJob job;
  job.segment_id = original_id;
  EXPECT_TRUE(gpr_planning::job_still_actionable(job, catalog));
}

TEST(SegmentCatalog, SplitsPartiallyBlockedSegment)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 2.0, 0.0);
  seg.centerline.points = {
    {0.0, 0.0, 0.0}, {0.25, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.75, 0.0, 0.0},
    {1.0, 0.0, 0.0}, {1.25, 0.0, 0.0}, {1.5, 0.0, 0.0}, {1.75, 0.0, 0.0},
    {2.0, 0.0, 0.0}};
  catalog.initialize({seg});

  gpr_planning::PathInvalidatorConfig config;
  config.footprint_radius = 0.08;
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

  catalog.update_blocked(grid, config);
  EXPECT_GE(catalog.segments().size(), 2U);
  EXPECT_GE(catalog.open_segments().size(), 1U);
  for (const auto & piece : catalog.segments()) {
    if (piece.source == gpr_common::SegmentSource::Split) {
      EXPECT_TRUE(piece.baseline_id.has_value());
    }
  }
}

TEST(SegmentCatalog, CompletedForJobMarksSplitChildren)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 2.0, 0.0);
  seg.centerline.points = {
    {0.0, 0.0, 0.0}, {0.25, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.75, 0.0, 0.0},
    {1.0, 0.0, 0.0}, {1.25, 0.0, 0.0}, {1.5, 0.0, 0.0}, {1.75, 0.0, 0.0},
    {2.0, 0.0, 0.0}};
  catalog.initialize({seg});

  gpr_planning::PathInvalidatorConfig config;
  config.footprint_radius = 0.08;
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

  catalog.update_blocked(grid, config);
  ASSERT_GE(catalog.segments().size(), 2U);

  gpr_common::Polyline driven;
  driven.points = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  catalog.mark_completed_for_job({seg.id}, driven);

  std::size_t completed_splits = 0U;
  for (const auto & piece : catalog.segments()) {
    if (piece.source == gpr_common::SegmentSource::Split &&
      piece.outcome == gpr_common::SegmentOutcome::Completed)
    {
      ++completed_splits;
    }
  }
  EXPECT_GE(completed_splits, 1U);
}

TEST(SegmentCatalog, OarpReplanGenerationCapStopsSchedulableWork)
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
  oconfig.max_replan_generations = 1U;
  gpr_planning::OarpLitePlanner planner(bconfig, oconfig);

  gpr_common::GridMap grid;
  grid.resolution = 0.1;
  grid.width = 20;
  grid.height = 20;
  grid.origin_x = -1.0;
  grid.origin_y = -1.0;
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);

  gpr_planning::PathInvalidatorConfig inv;
  gpr_planning::SegmentCatalogConfig cat_config;
  cat_config.schedule_blocked_probes = false;
  gpr_planning::SegmentCatalog catalog(cat_config);
  auto seg = make_segment(0, 0, -0.5, -0.8, -0.5, 0.8);
  seg.blocked = true;
  catalog.initialize({seg});
  EXPECT_TRUE(catalog.schedulable_segments().empty());

  catalog.refresh_oarp_ranks(grid, inv, planner);
  ASSERT_FALSE(catalog.schedulable_segments().empty());

  std::vector<gpr_common::SegmentId> rank_ids;
  for (const auto & rank : catalog.segments()) {
    if (rank.source == gpr_common::SegmentSource::OarpRank) {
      rank_ids.push_back(rank.id);
    }
  }
  ASSERT_FALSE(rank_ids.empty());
  catalog.mark_blocked_for_job(rank_ids);
  catalog.mark_attempted_for_job(rank_ids);
  EXPECT_TRUE(catalog.schedulable_segments().empty());

  catalog.refresh_oarp_ranks(grid, inv, planner);
  EXPECT_TRUE(catalog.schedulable_segments().empty());
  EXPECT_FALSE(catalog.has_schedulable_work(grid, inv, planner));
}

TEST(SegmentCatalog, ShortTraceDoesNotCompleteLongSegment)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 4.0, 0.0);
  catalog.initialize({seg});

  gpr_common::Polyline short_trace;
  short_trace.points = {{0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}};
  catalog.mark_completed_for_job({seg.id}, short_trace, 0.25, 0.55);

  ASSERT_NE(catalog.find(seg.id), nullptr);
  EXPECT_EQ(
    catalog.find(seg.id)->outcome, gpr_common::SegmentOutcome::PartiallyCompleted);
  EXPECT_TRUE(catalog.schedulable_segments().empty());
}

TEST(SegmentCatalog, PassLevelCoverageMarksAllCoverSegments)
{
  gpr_planning::SegmentCatalog catalog;
  std::vector<gpr_common::CoverageSegment> segments;
  for (uint32_t ord = 0; ord < 3U; ++ord) {
    auto seg = make_segment(0, ord, static_cast<double>(ord), 0.0,
      static_cast<double>(ord) + 1.0, 0.0);
    segments.push_back(seg);
  }

  gpr_common::Polyline pass;
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.1) {
    pass.points.push_back({x, 0.0, 0.0});
  }

  gpr_common::SwathCoverageConfig config;
  config.lateral_max_m = 0.25;
  config.min_complete_fraction = 0.85;
  config.partial_enabled = true;

  gpr_common::Polyline trace;
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.05) {
    trace.points.push_back({x, 0.0, 0.0});
  }

  std::vector<gpr_common::SegmentId> cover_ids;
  cover_ids.reserve(segments.size());
  for (const auto & seg : segments) {
    cover_ids.push_back(seg.id);
  }
  catalog.initialize(segments);
  catalog.apply_swath_coverage_for_job(cover_ids, trace, config, &pass);

  for (const auto & id : cover_ids) {
    ASSERT_NE(catalog.find(id), nullptr);
    EXPECT_EQ(
      catalog.find(id)->outcome, gpr_common::SegmentOutcome::Completed);
  }
}

TEST(SegmentCatalog, PassCoverIdsOnlyOnShortcut)
{
  gpr_planning::SegmentCatalog catalog;
  std::vector<gpr_common::CoverageSegment> segments;
  constexpr uint32_t lane = 4U;
  for (uint32_t ord = 0; ord < 3U; ++ord) {
    const double y0 = static_cast<double>(ord);
    auto seg = make_segment(lane, ord, 0.0, y0, 0.0, y0 + 1.0);
    segments.push_back(seg);
  }
  catalog.initialize(segments);

  gpr_common::Polyline pass;
  for (double y = 0.0; y <= 3.0 + 1e-9; y += 0.1) {
    pass.points.push_back({0.0, y, M_PI_2});
  }
  gpr_common::Polyline trace;
  for (double y = 0.0; y <= 3.0 + 1e-9; y += 0.05) {
    trace.points.push_back({0.0, y, M_PI_2});
  }

  gpr_common::SwathCoverageConfig config;
  config.lateral_max_m = 0.25;
  config.min_complete_fraction = 0.85;
  config.partial_enabled = true;

  catalog.apply_swath_coverage_for_job(
    {segments.front().id}, trace, config, &pass,
    gpr_common::DriveDirection::Forward, lane);

  ASSERT_NE(catalog.find(segments.front().id), nullptr);
  EXPECT_EQ(
    catalog.find(segments.front().id)->outcome,
    gpr_common::SegmentOutcome::Completed);
  for (std::size_t i = 1; i < segments.size(); ++i) {
    ASSERT_NE(catalog.find(segments[i].id), nullptr);
    EXPECT_EQ(
      catalog.find(segments[i].id)->outcome,
      gpr_common::SegmentOutcome::Pending);
  }
}

TEST(SegmentCatalog, LowCoverageProgressNotSchedulable)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 4.0, 0.0);
  catalog.initialize({seg});

  gpr_common::SwathCoverageConfig config;
  config.lateral_max_m = 0.25;
  config.min_complete_fraction = 0.85;
  config.min_partial_fraction = 0.35;
  config.partial_enabled = true;

  gpr_common::Polyline trace;
  for (double x = 0.0; x <= 0.4; x += 0.05) {
    trace.points.push_back({x, 0.0, 0.0});
  }
  catalog.apply_swath_coverage_for_job({seg.id}, trace, config, nullptr);

  ASSERT_NE(catalog.find(seg.id), nullptr);
  const auto & updated = *catalog.find(seg.id);
  EXPECT_EQ(updated.outcome, gpr_common::SegmentOutcome::PartiallyCompleted);
  EXPECT_FALSE(updated.covered_intervals_m.empty());
  EXPECT_TRUE(catalog.schedulable_segments().empty());
  EXPECT_FALSE(gpr_common::is_segment_schedulable(updated, false));
}

TEST(SegmentCatalog, PartialTailBlockedOnGridUpdate)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 4.0, 0.0);
  catalog.initialize({seg});

  gpr_common::SwathCoverageConfig config;
  config.lateral_max_m = 0.25;
  config.min_complete_fraction = 0.85;
  config.min_partial_fraction = 0.35;
  config.partial_enabled = true;

  gpr_common::Polyline trace;
  for (double x = 0.0; x <= 1.5; x += 0.05) {
    trace.points.push_back({x, 0.0, 0.0});
  }
  catalog.apply_swath_coverage_for_job({seg.id}, trace, config, nullptr);
  ASSERT_NE(catalog.find(seg.id), nullptr);
  EXPECT_EQ(
    catalog.find(seg.id)->outcome, gpr_common::SegmentOutcome::PartiallyCompleted);
  EXPECT_FALSE(catalog.find(seg.id)->blocked);

  gpr_planning::PathInvalidatorConfig inv_config;
  auto grid = empty_grid();
  grid.width = 120;
  grid.height = 40;
  grid.data.assign(static_cast<std::size_t>(grid.width) * grid.height, 0);
  const int mx = static_cast<int>((3.0 - grid.origin_x) / grid.resolution);
  const int my = static_cast<int>((0.0 - grid.origin_y) / grid.resolution);
  for (int dy = -3; dy <= 3; ++dy) {
    for (int dx = -3; dx <= 3; ++dx) {
      const int idx = (my + dy) * grid.width + (mx + dx);
      if (idx >= 0 && idx < static_cast<int>(grid.data.size())) {
        grid.data[static_cast<std::size_t>(idx)] = 100;
      }
    }
  }
  catalog.update_blocked(grid, inv_config);

  ASSERT_NE(catalog.find(seg.id), nullptr);
  EXPECT_TRUE(catalog.find(seg.id)->blocked);
  EXPECT_TRUE(catalog.schedulable_segments().empty());
}

TEST(SegmentCatalog, PartialCoverageLeavesSchedulableTail)
{
  gpr_planning::SegmentCatalog catalog;
  auto seg = make_segment(0, 0, 0.0, 0.0, 4.0, 0.0);
  catalog.initialize({seg});

  gpr_common::SwathCoverageConfig config;
  config.lateral_max_m = 0.25;
  config.min_complete_fraction = 0.85;
  config.min_partial_fraction = 0.35;
  config.partial_enabled = true;

  gpr_common::Polyline trace;
  for (double x = 0.0; x <= 1.5; x += 0.05) {
    trace.points.push_back({x, 0.0, 0.0});
  }
  catalog.apply_swath_coverage_for_job({seg.id}, trace, config, nullptr);

  ASSERT_NE(catalog.find(seg.id), nullptr);
  const auto & updated = *catalog.find(seg.id);
  EXPECT_EQ(updated.outcome, gpr_common::SegmentOutcome::PartiallyCompleted);
  EXPECT_FALSE(catalog.open_segments().empty());
  EXPECT_TRUE(catalog.schedulable_segments().empty());
  const auto tail = gpr_common::schedulable_centerline(
    updated, gpr_common::DriveDirection::Forward);
  EXPECT_GE(tail.length(), 1.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

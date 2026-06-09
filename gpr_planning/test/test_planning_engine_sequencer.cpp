#include <gtest/gtest.h>

#include "gpr_common/scan_region.hpp"
#include "gpr_planning/planning_engine.hpp"

namespace
{

gpr_planning::PlanningEngine make_engine(const bool use_boustrophedon_sequencer)
{
  gpr_planning::BoustrophedonConfig bconfig;
  bconfig.region = gpr_common::ScanRegion::from_rectangle(
    gpr_common::ScanArea{0.0, 4.0, 0.0, 4.0});
  bconfig.lane_spacing = 0.5;
  gpr_planning::SegmentCatalogConfig catalog_config;
  catalog_config.schedule_blocked_probes = false;
  return gpr_planning::PlanningEngine(
    bconfig, {}, {}, "map", {}, catalog_config, {}, use_boustrophedon_sequencer);
}

std::size_t schedulable_count(const gpr_planning::PlanningEngine & engine)
{
  return engine.catalog().schedulable_segments().size();
}

void compare_catalog_states(
  const gpr_planning::PlanningEngine & a,
  const gpr_planning::PlanningEngine & b)
{
  ASSERT_EQ(a.catalog().segments().size(), b.catalog().segments().size());
  for (std::size_t i = 0; i < a.catalog().segments().size(); ++i) {
    const auto & sa = a.catalog().segments()[i];
    const auto & sb = b.catalog().segments()[i];
    EXPECT_EQ(sa.id, sb.id);
    EXPECT_EQ(sa.outcome, sb.outcome);
    EXPECT_EQ(sa.blocked, sb.blocked);
    EXPECT_EQ(sa.covered_intervals_m.size(), sb.covered_intervals_m.size());
  }
  EXPECT_EQ(schedulable_count(a), schedulable_count(b));
}

}  // namespace

TEST(SequencerParity, InitialSchedulableSetMatches)
{
  auto eng_atsp = make_engine(false);
  auto eng_boust = make_engine(true);
  eng_atsp.generate_initial_coverage();
  eng_boust.generate_initial_coverage();
  compare_catalog_states(eng_atsp, eng_boust);
}

TEST(SequencerParity, RecomputeDoesNotChangeCatalog)
{
  auto eng_atsp = make_engine(false);
  auto eng_boust = make_engine(true);
  eng_atsp.generate_initial_coverage();
  eng_boust.generate_initial_coverage();
  const gpr_common::Pose2D robot{0.0, 0.0, 0.0};
  (void)eng_atsp.recompute_schedule(robot);
  (void)eng_boust.recompute_schedule(robot);
  compare_catalog_states(eng_atsp, eng_boust);
}

TEST(SequencerParity, SchedulableSetMatchesAfterGridBlock)
{
  auto eng_atsp = make_engine(false);
  auto eng_boust = make_engine(true);
  eng_atsp.generate_initial_coverage();
  eng_boust.generate_initial_coverage();

  gpr_common::GridMap grid;
  grid.resolution = 0.05;
  grid.width = 80;
  grid.height = 80;
  grid.origin_x = -1.0;
  grid.origin_y = -1.0;
  grid.data.assign(static_cast<std::size_t>(grid.width) * grid.height, 0);
  for (auto & cell : grid.data) {
    cell = 100;
  }

  eng_atsp.update_from_grid(std::make_shared<gpr_common::GridMap>(grid));
  eng_boust.update_from_grid(std::make_shared<gpr_common::GridMap>(grid));
  compare_catalog_states(eng_atsp, eng_boust);
}

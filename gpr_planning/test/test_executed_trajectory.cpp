#include <gtest/gtest.h>

#include "gpr_common/scan_region.hpp"
#include "gpr_planning/planning_engine.hpp"

namespace
{

gpr_planning::PlanningEngine make_engine()
{
  gpr_planning::BoustrophedonConfig bconfig;
  bconfig.region = gpr_common::ScanRegion::from_rectangle(gpr_common::ScanArea{0.0, 4.0, 0.0, 4.0});
  bconfig.lane_spacing = 0.5;
  return gpr_planning::PlanningEngine(bconfig, {}, {}, "map");
}

gpr_common::Pose2D pose_at(const double x, const double y)
{
  return gpr_common::Pose2D{x, y, 0.0};
}

}  // namespace

TEST(ExecutedTrajectory, decimates_mission_trace_at_five_centimeters)
{
  auto engine = make_engine();
  EXPECT_TRUE(engine.append_executed_pose(pose_at(0.0, 0.0)));
  EXPECT_FALSE(engine.append_executed_pose(pose_at(0.02, 0.0)));
  EXPECT_TRUE(engine.append_executed_pose(pose_at(0.06, 0.0)));
  EXPECT_EQ(engine.mission_executed_trace().size(), 2U);
}

TEST(ExecutedTrajectory, clear_executed_trace_preserves_mission_trace)
{
  auto engine = make_engine();
  EXPECT_TRUE(engine.append_executed_pose(pose_at(0.0, 0.0)));
  EXPECT_TRUE(engine.append_executed_pose(pose_at(1.0, 0.0)));
  const auto mission_size = engine.mission_executed_trace().size();
  ASSERT_GT(mission_size, 0U);

  engine.clear_executed_trace();
  EXPECT_TRUE(engine.mission_executed_trace().size() == mission_size);
}

TEST(ExecutedTrajectory, generate_initial_coverage_clears_mission_trace)
{
  auto engine = make_engine();
  EXPECT_TRUE(engine.append_executed_pose(pose_at(0.0, 0.0)));
  EXPECT_TRUE(engine.append_executed_pose(pose_at(2.0, 0.0)));
  ASSERT_GT(engine.mission_executed_trace().size(), 0U);

  engine.generate_initial_coverage();
  EXPECT_TRUE(engine.mission_executed_trace().empty());
}

TEST(ExecutedTrajectory, mission_trace_accumulates_across_jobs)
{
  auto engine = make_engine();
  engine.generate_initial_coverage();

  EXPECT_TRUE(engine.append_executed_pose(pose_at(0.0, 0.0)));
  EXPECT_TRUE(engine.append_executed_pose(pose_at(1.0, 0.0)));
  const auto after_job_one = engine.mission_executed_trace().size();

  engine.clear_executed_trace();
  EXPECT_TRUE(engine.append_executed_pose(pose_at(2.0, 0.0)));
  EXPECT_TRUE(engine.append_executed_pose(pose_at(3.0, 0.0)));

  EXPECT_GT(engine.mission_executed_trace().size(), after_job_one);
}

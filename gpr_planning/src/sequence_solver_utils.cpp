#include "gpr_planning/sequence_solver_utils.hpp"

namespace gpr_planning
{

std::vector<DirectedJob> build_directed_jobs(
  const std::vector<gpr_common::CoverageSegment> & segments,
  const std::unordered_map<gpr_common::SegmentId, gpr_common::DriveDirection> *
  fixed_directions)
{
  std::vector<DirectedJob> jobs;
  for (const auto & seg : segments) {
    if (seg.centerline.size() < 2U) {
      continue;
    }
    if (fixed_directions != nullptr) {
      const auto it = fixed_directions->find(seg.id);
      if (it != fixed_directions->end()) {
        jobs.push_back({
          {seg.id, it->second}, seg.id,
          gpr_common::job_entry_pose(seg, it->second),
          gpr_common::job_exit_pose(seg, it->second)
        });
        continue;
      }
    }
    for (const auto dir : {gpr_common::DriveDirection::Forward,
      gpr_common::DriveDirection::Reverse})
    {
      jobs.push_back({
        {seg.id, dir}, seg.id,
        gpr_common::job_entry_pose(seg, dir),
        gpr_common::job_exit_pose(seg, dir)
      });
    }
  }
  return jobs;
}

}  // namespace gpr_planning

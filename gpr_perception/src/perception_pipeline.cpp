#include "gpr_perception/perception_pipeline.hpp"

namespace gpr_perception
{

PerceptionPipeline::PerceptionPipeline(OccupancyGridMapper mapper)
: mapper_(std::move(mapper))
{}

void PerceptionPipeline::integrate_ray(
  const double origin_x, const double origin_y,
  const double hit_x, const double hit_y,
  const bool endpoint_is_obstacle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  mapper_.integrate_ray(origin_x, origin_y, hit_x, hit_y, endpoint_is_obstacle);
}

void PerceptionPipeline::refresh_grids()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_grid_ = std::make_shared<gpr_common::GridMap>(mapper_.to_planning_grid_map());
  latest_inflated_grid_ = std::make_shared<gpr_common::GridMap>(mapper_.to_grid_map());
  has_grid_ = latest_grid_ && !latest_grid_->empty();
  ++grid_seq_;
}

gpr_common::GridMapConstPtr PerceptionPipeline::latest_grid_ptr() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_grid_;
}

gpr_common::GridMap PerceptionPipeline::latest_grid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_grid_ ? *latest_grid_ : gpr_common::GridMap{};
}

gpr_common::GridMapConstPtr PerceptionPipeline::latest_inflated_grid_ptr() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_inflated_grid_;
}

gpr_common::GridMap PerceptionPipeline::latest_inflated_grid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_inflated_grid_ ? *latest_inflated_grid_ : gpr_common::GridMap{};
}

bool PerceptionPipeline::has_grid() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_grid_;
}

}  // namespace gpr_perception

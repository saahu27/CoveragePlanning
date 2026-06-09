#ifndef GPR_PERCEPTION__PERCEPTION_PIPELINE_HPP_
#define GPR_PERCEPTION__PERCEPTION_PIPELINE_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>

#include "gpr_common/grid_map.hpp"
#include "gpr_perception/occupancy_grid_mapper.hpp"

namespace gpr_perception
{

/// @brief Domain perception state (mapper + latest grids) without ROS types.
class PerceptionPipeline
{
public:
  explicit PerceptionPipeline(OccupancyGridMapper mapper);

  [[nodiscard]] OccupancyGridMapper & mapper() noexcept {return mapper_;}
  [[nodiscard]] const OccupancyGridMapper & mapper() const noexcept {return mapper_;}

  void integrate_ray(
    double origin_x, double origin_y,
    double hit_x, double hit_y,
    bool endpoint_is_obstacle);

  void refresh_grids();

  [[nodiscard]] std::uint64_t grid_seq() const noexcept {return grid_seq_.load();}
  [[nodiscard]] gpr_common::GridMapConstPtr latest_grid_ptr() const;
  [[nodiscard]] gpr_common::GridMap latest_grid() const;
  [[nodiscard]] gpr_common::GridMapConstPtr latest_inflated_grid_ptr() const;
  [[nodiscard]] gpr_common::GridMap latest_inflated_grid() const;
  [[nodiscard]] bool has_grid() const noexcept;

private:
  OccupancyGridMapper mapper_;
  mutable std::mutex mutex_;
  gpr_common::GridMapConstPtr latest_grid_;
  gpr_common::GridMapConstPtr latest_inflated_grid_;
  bool has_grid_{false};
  std::atomic<std::uint64_t> grid_seq_{0};
};

}  // namespace gpr_perception

#endif  // GPR_PERCEPTION__PERCEPTION_PIPELINE_HPP_

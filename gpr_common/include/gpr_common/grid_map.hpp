#ifndef GPR_COMMON__GRID_MAP_HPP_
#define GPR_COMMON__GRID_MAP_HPP_

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gpr_common/types.hpp"

namespace gpr_common
{

/// @brief Thrown when a grid map is constructed with invalid dimensions.
class GridMapConfigError : public std::invalid_argument
{
public:
  using std::invalid_argument::invalid_argument;
};

/**
 * @brief Inflated occupancy grid snapshot (ROS-independent).
 *
 * Cell values: -1 unknown, 0-100 cost, 100 lethal.
 */
struct GridMap
{
  double resolution{0.05};
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;

  [[nodiscard]] bool empty() const noexcept {return data.empty();}
  std::size_t revision{0};
};

using GridMapConstPtr = std::shared_ptr<const GridMap>;

/// @brief Stateless helpers for querying collisions against a GridMap.
class GridMapUtils
{
public:
  /// @brief Convert world coordinates to cell indices; false if out of bounds.
  [[nodiscard]] static bool world_to_map(
    const GridMap & grid, double x, double y, int & mx, int & my) noexcept;

  /// @brief Read the cost of a cell by index; false if out of bounds.
  [[nodiscard]] static bool cell_cost(
    const GridMap & grid, int mx, int my, int8_t & cost) noexcept;

  /// @brief Read the cost at a world point; false if out of bounds.
  [[nodiscard]] static bool occupancy_cost_at(
    const GridMap & grid, double x, double y, int8_t & cost) noexcept;

  /// @brief Decide whether a cost value counts as blocked.
  [[nodiscard]] static bool is_cell_blocked(
    int8_t cost, int obstacle_threshold, bool treat_unknown_as_free) noexcept;

  /// @brief True if a disk of @p footprint_radius around (x, y) hits an obstacle.
  [[nodiscard]] static bool is_pose_colliding(
    const GridMap & grid, double x, double y,
    int obstacle_threshold, double footprint_radius,
    bool treat_unknown_as_free);

  /// @brief True if the swept footprint along a segment hits an obstacle.
  [[nodiscard]] static bool is_segment_colliding(
    const GridMap & grid,
    double x0, double y0, double x1, double y1,
    int obstacle_threshold, double segment_sample_spacing,
    double footprint_radius, bool treat_unknown_as_free,
    double resolution_sample_factor = 0.5);
};

}  // namespace gpr_common

#endif  // GPR_COMMON__GRID_MAP_HPP_

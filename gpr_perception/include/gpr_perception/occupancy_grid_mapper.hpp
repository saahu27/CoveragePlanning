#ifndef GPR_PERCEPTION__OCCUPANCY_GRID_MAPPER_HPP_
#define GPR_PERCEPTION__OCCUPANCY_GRID_MAPPER_HPP_

#include <cstdint>
#include <vector>

#include "gpr_common/grid_map.hpp"
#include "gpr_common/types.hpp"

namespace gpr_perception
{

/// @brief Tunables for the log-odds occupancy mapper.
struct OccupancyGridConfig
{
  gpr_common::ScanArea scan_area;   ///< Grid extent in world coordinates.
  double resolution{0.05};          ///< Cell size (m).
  double inflation_radius{0.35};    ///< Obstacle safety inflation (m).
  double hit_marking_radius{0.15};  ///< Disk radius marked occupied per hit (m).
  double prob_hit{0.85};            ///< Sensor model: P(occupied | hit).
  double prob_miss{0.35};           ///< Sensor model: P(occupied | miss).
  double log_odds_clamp{5.0};       ///< Symmetric clamp on accumulated log-odds.
  int occupied_export_threshold{50};  ///< Cost at/above which a cell is exported lethal.
};

/// @brief Thrown on invalid OccupancyGridConfig values.
class OccupancyGridConfigError : public std::invalid_argument
{
public:
  using std::invalid_argument::invalid_argument;
};

/// @brief Rolling 2D log-odds occupancy grid built from ray observations.
class OccupancyGridMapper
{
public:
  explicit OccupancyGridMapper(OccupancyGridConfig config);

  /// @brief Clear all accumulated occupancy back to unknown.
  void reset();

  /// @brief Fuse one sensor ray: free along the beam, occupied at the endpoint.
  void integrate_ray(
    double origin_x, double origin_y,
    double hit_x, double hit_y,
    bool endpoint_is_obstacle);

  /// @brief Export the current state as an inflated, thresholded GridMap (for RViz).
  [[nodiscard]] gpr_common::GridMap to_grid_map() const;
  /// @brief Export thresholded occupancy without inflation (for segment invalidation).
  [[nodiscard]] gpr_common::GridMap to_planning_grid_map() const;
  [[nodiscard]] const OccupancyGridConfig & config() const noexcept {return config_;}

  /// @brief Validate config; throws OccupancyGridConfigError on bad values.
  static void validate_config(const OccupancyGridConfig & config);

private:
  static constexpr double kEpsilon = 1e-9;

  [[nodiscard]] static double probability_to_log_odds(double probability);
  [[nodiscard]] int cell_index(int mx, int my) const noexcept;
  [[nodiscard]] bool cell_in_bounds(int mx, int my) const noexcept;
  [[nodiscard]] bool world_to_map(double x, double y, int & mx, int & my) const noexcept;

  void apply_log_odds_update(int mx, int my, double delta);
  void mark_hit_disk(double hit_x, double hit_y);
  void ray_trace(int x0, int y0, int x1, int y1, bool endpoint_is_obstacle);
  [[nodiscard]] std::vector<int8_t> build_inflated_export_grid() const;
  [[nodiscard]] std::vector<int8_t> build_threshold_export_grid() const;

  OccupancyGridConfig config_;
  int width_{0};
  int height_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double log_odds_hit_{0.0};
  double log_odds_miss_{0.0};
  std::vector<double> log_odds_;
  std::vector<uint8_t> observed_;
  mutable std::size_t revision_{0};
};

}  // namespace gpr_perception

#endif  // GPR_PERCEPTION__OCCUPANCY_GRID_MAPPER_HPP_

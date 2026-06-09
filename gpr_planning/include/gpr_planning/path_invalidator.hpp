#ifndef GPR_PLANNING__PATH_INVALIDATOR_HPP_
#define GPR_PLANNING__PATH_INVALIDATOR_HPP_

#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/grid_map.hpp"
#include "gpr_common/scan_region.hpp"
#include "gpr_common/types.hpp"

namespace gpr_planning
{

/// @brief Tunables controlling how segments are tested against the grid.
struct PathInvalidatorConfig
{
  int obstacle_cost_threshold{50};   ///< Cost at/above which a cell blocks.
  double segment_sample_spacing{0.05};  ///< Collision sampling step along an edge (m).
  /// @brief Edge sample step uses max(segment_sample_spacing, factor × resolution).
  double collision_sample_resolution_factor{0.5};
  /// @brief Spatial quantization (m) for legacy geometry-based segment ids.
  double legacy_segment_quantize_m{0.05};
  double footprint_radius{0.12};     ///< Robot radius used for clearance (m).
  /// @brief Map inflation radius (m); standoff from occupied cells (use max with footprint).
  double map_inflation_radius{0.0};
  bool treat_unknown_as_free{true};  ///< Whether unknown cells are passable.
  /// @brief When true, only observed occupied cells block (-1 never blocks).
  bool block_only_observed_occupied{true};
  gpr_common::ScanRegion region{};  ///< Mission boundary for perimeter filtering.
  /// @brief Ignore obstacles within this margin of @p scan_area edges (m).
  /// Perimeter walls sit just outside the scan region; inflated lidar hits
  /// should not block inset coverage lanes.
  double boundary_ignore_margin{0.0};
};

  /// @brief Collision tests that decide which coverage segments are blocked.
class PathInvalidator
{
public:
  /// @brief Clearance radius used for collision checks (matches RViz inflated grid).
  [[nodiscard]] static double effective_footprint_radius(
    const PathInvalidatorConfig & config) noexcept;

  /// @brief Split a path into collision-free segments against @p grid.
  [[nodiscard]] static std::vector<gpr_common::CoverageSegment> invalidate_segments(
    const gpr_common::Polyline & initial_path,
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & config);

  // True if any point or edge of the centerline collides with the grid. Used to
  // (re)evaluate whether a stable coverage segment is currently blocked.
  [[nodiscard]] static bool is_blocked(
    const gpr_common::Polyline & centerline,
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & config);

  /// @brief Collision-free sub-polylines after splitting on occupied cells.
  [[nodiscard]] static std::vector<gpr_common::Polyline> free_subpolylines(
    const gpr_common::Polyline & centerline,
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & config);

  /// @brief Occupied / in-collision sub-polylines (complement of @p free_subpolylines).
  [[nodiscard]] static std::vector<gpr_common::Polyline> blocked_subpolylines(
    const gpr_common::Polyline & centerline,
    const gpr_common::GridMap & grid,
    const PathInvalidatorConfig & config);
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__PATH_INVALIDATOR_HPP_

#ifndef GPR_PLANNING__BOUSTROPHEDON_PLANNER_HPP_
#define GPR_PLANNING__BOUSTROPHEDON_PLANNER_HPP_

#include <stdexcept>
#include <string>
#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/scan_region.hpp"
#include "gpr_common/types.hpp"

namespace gpr_planning
{

/// @brief Tunables for boustrophedon coverage generation.
struct BoustrophedonConfig
{
  gpr_common::ScanRegion region;  ///< Mission boundary (rectangle = 4 corners from bounds).
  double lane_spacing{0.5};         ///< Distance between adjacent scan lanes (m).
  double waypoint_spacing{0.1};     ///< Sampling distance along a lane (m).
  double coverage_inset{0.0};       ///< Shrink applied to each area edge (m).
  // Length of each fixed coverage sub-segment a lane is split into. Stable
  // sub-segments give persistent identity so obstacles only block the
  // overlapping pieces while the rest of the lane is still covered.
  double segment_length{1.0};
  gpr_common::ScanDirection scan_direction{gpr_common::ScanDirection::kX};
};

/// @brief Thrown on invalid BoustrophedonConfig values.
class PlannerConfigurationError : public std::invalid_argument
{
public:
  using std::invalid_argument::invalid_argument;
};

/// @brief Generates a zig-zag (boustrophedon) coverage path over a rectangle.
class BoustrophedonPlanner
{
public:
  /// @brief Lane index plus discretized centerline for rank replanning.
  struct LaneCenterline
  {
    uint32_t index{0};
    gpr_common::Polyline centerline;
  };

  explicit BoustrophedonPlanner(BoustrophedonConfig config);

  /// @brief Full continuous coverage polyline (lanes + connectors) for display.
  [[nodiscard]] gpr_common::Polyline generate() const;

  // Stable master set of coverage sub-segments (one group per scan lane, each
  // lane split into segment_length pieces). Connector turns between lanes are
  // NOT included; transit between segments is handled by the A* planner.
  [[nodiscard]] std::vector<gpr_common::CoverageSegment> generate_segments() const;

  /// @brief One centerline polyline per scan lane (connectors omitted).
  [[nodiscard]] std::vector<LaneCenterline> generate_lane_centerlines() const;

  /// @brief Horizontal/turn edges between consecutive scan lanes (transit only, not coverage).
  [[nodiscard]] std::vector<gpr_common::Polyline> generate_connectors() const;

  [[nodiscard]] const BoustrophedonConfig & config() const noexcept {return config_;}

  /// @brief Parse "x"/"y" into a ScanDirection; throws on anything else.
  [[nodiscard]] static gpr_common::ScanDirection scan_direction_from_string(
    const std::string & value);
  /// @brief Validate config; throws PlannerConfigurationError on bad values.
  static void validate_config(const BoustrophedonConfig & config);

private:
  static constexpr double kEpsilon = 1e-9;
  BoustrophedonConfig config_;

  // A single scan lane (start -> end), in boustrophedon visiting order.
  struct Lane
  {
    uint32_t index;
    double x0;
    double y0;
    double x1;
    double y1;
  };

  void append_discretized_segment(
    gpr_common::Polyline & polyline,
    double x0, double y0, double x1, double y1,
    bool skip_first) const;

  [[nodiscard]] gpr_common::Polyline discretize_segment(
    double x0, double y0, double x1, double y1) const;

  [[nodiscard]] std::vector<Lane> compute_lanes() const;

  [[nodiscard]] static std::vector<double> compute_lane_coordinates(
    double min_coord, double max_coord, double spacing);
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__BOUSTROPHEDON_PLANNER_HPP_

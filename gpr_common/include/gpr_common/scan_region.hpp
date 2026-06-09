#ifndef GPR_COMMON__SCAN_REGION_HPP_
#define GPR_COMMON__SCAN_REGION_HPP_

#include <vector>

#include "gpr_common/types.hpp"

namespace gpr_common
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/// @brief Mission coverage boundary (polygon) plus axis-aligned bounds for grid / lane spacing.
struct ScanRegion
{
  ScanArea bounds{};
  std::vector<Point2D> vertices;

  [[nodiscard]] static ScanRegion from_rectangle(ScanArea area);
  [[nodiscard]] static ScanRegion from_vertices(std::vector<Point2D> verts);

  /// @brief Shrink boundary by @p inset_m (rectangle: inset bounds; polygon: edge-normal offset).
  [[nodiscard]] ScanRegion inset(double inset_m) const;
};

}  // namespace gpr_common

#endif  // GPR_COMMON__SCAN_REGION_HPP_

#ifndef GPR_COMMON__POLYGON_GEOMETRY_HPP_
#define GPR_COMMON__POLYGON_GEOMETRY_HPP_

#include <stdexcept>
#include <vector>

#include "gpr_common/scan_region.hpp"
#include "gpr_common/types.hpp"

namespace gpr_common
{

class PolygonGeometryError : public std::invalid_argument
{
public:
  using std::invalid_argument::invalid_argument;
};

struct Chord
{
  Point2D a;
  Point2D b;
};

[[nodiscard]] ScanArea bounding_box(const std::vector<Point2D> & vertices);

[[nodiscard]] double polygon_area(const std::vector<Point2D> & vertices);

void validate_simple_polygon(const std::vector<Point2D> & vertices);

[[nodiscard]] bool point_in_polygon(const Point2D & point, const std::vector<Point2D> & vertices);

/// @brief Distance from @p point to the nearest edge of @p vertices (m).
[[nodiscard]] double distance_to_polygon_boundary(
  double x, double y, const std::vector<Point2D> & vertices);

/// @brief Parallel scan chords inside a simple polygon at fixed lane coordinate.
[[nodiscard]] std::vector<Chord> chords_on_scan_line(
  const std::vector<Point2D> & vertices,
  ScanDirection direction,
  double coord);

}  // namespace gpr_common

#endif  // GPR_COMMON__POLYGON_GEOMETRY_HPP_

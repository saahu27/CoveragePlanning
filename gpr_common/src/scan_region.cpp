#include "gpr_common/scan_region.hpp"

#include <algorithm>
#include <cmath>

#include "gpr_common/polygon_geometry.hpp"

namespace gpr_common
{

namespace
{
constexpr double kEpsilon = 1e-9;

ScanArea bounds_from_vertices(const std::vector<Point2D> & vertices)
{
  ScanArea area;
  if (vertices.empty()) {
    return area;
  }
  area.x_min = area.x_max = vertices.front().x;
  area.y_min = area.y_max = vertices.front().y;
  for (const auto & v : vertices) {
    area.x_min = std::min(area.x_min, v.x);
    area.x_max = std::max(area.x_max, v.x);
    area.y_min = std::min(area.y_min, v.y);
    area.y_max = std::max(area.y_max, v.y);
  }
  return area;
}

bool is_axis_aligned_rectangle(const std::vector<Point2D> & vertices, const ScanArea & bounds)
{
  if (vertices.size() != 4U) {
    return false;
  }
  for (const auto & v : vertices) {
    const bool on_vertical = std::abs(v.x - bounds.x_min) < 1e-6 ||
      std::abs(v.x - bounds.x_max) < 1e-6;
    const bool on_horizontal = std::abs(v.y - bounds.y_min) < 1e-6 ||
      std::abs(v.y - bounds.y_max) < 1e-6;
    if (!on_vertical && !on_horizontal) {
      return false;
    }
  }
  return true;
}
}  // namespace

ScanRegion ScanRegion::from_rectangle(ScanArea area)
{
  ScanRegion region;
  region.bounds = area;
  region.vertices = {
    Point2D{area.x_min, area.y_min},
    Point2D{area.x_max, area.y_min},
    Point2D{area.x_max, area.y_max},
    Point2D{area.x_min, area.y_max},
  };
  return region;
}

ScanRegion ScanRegion::from_vertices(std::vector<Point2D> verts)
{
  validate_simple_polygon(verts);
  ScanRegion region;
  region.vertices = std::move(verts);
  region.bounds = bounds_from_vertices(region.vertices);
  return region;
}

ScanRegion ScanRegion::inset(double inset_m) const
{
  if (inset_m <= kEpsilon) {
    return *this;
  }
  ScanArea shrunk = bounds;
  shrunk.x_min += inset_m;
  shrunk.x_max -= inset_m;
  shrunk.y_min += inset_m;
  shrunk.y_max -= inset_m;
  if (shrunk.x_max <= shrunk.x_min + kEpsilon || shrunk.y_max <= shrunk.y_min + kEpsilon) {
    throw PolygonGeometryError("coverage_inset exceeds scan region bounds.");
  }
  if (is_axis_aligned_rectangle(vertices, bounds)) {
    return from_rectangle(shrunk);
  }
  std::vector<Point2D> offset;
  offset.reserve(vertices.size());
  const int n = static_cast<int>(vertices.size());
  for (int i = 0; i < n; ++i) {
    const auto & prev = vertices[(static_cast<std::size_t>(i) + vertices.size() - 1U) % vertices.size()];
    const auto & curr = vertices[static_cast<std::size_t>(i)];
    const auto & next = vertices[(static_cast<std::size_t>(i) + 1U) % vertices.size()];
    const double e1x = curr.x - prev.x;
    const double e1y = curr.y - prev.y;
    const double e2x = next.x - curr.x;
    const double e2y = next.y - curr.y;
    const double len1 = std::hypot(e1x, e1y);
    const double len2 = std::hypot(e2x, e2y);
    if (len1 < kEpsilon || len2 < kEpsilon) {
      throw PolygonGeometryError("degenerate polygon edge during inset.");
    }
    const double n1x = -e1y / len1;
    const double n1y = e1x / len1;
    const double n2x = -e2y / len2;
    const double n2y = e2x / len2;
    double nx = n1x + n2x;
    double ny = n1y + n2y;
    const double nlen = std::hypot(nx, ny);
    if (nlen < kEpsilon) {
      nx = n1x;
      ny = n1y;
    } else {
      nx /= nlen;
      ny /= nlen;
    }
    const double cross = e1x * e2y - e1y * e2x;
    if (cross < 0.0) {
      nx = -nx;
      ny = -ny;
    }
    offset.push_back(Point2D{curr.x + nx * inset_m, curr.y + ny * inset_m});
  }
  return from_vertices(std::move(offset));
}

}  // namespace gpr_common

#include "gpr_common/polygon_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gpr_common
{

namespace
{
constexpr double kEpsilon = 1e-9;

void append_unique_sorted(std::vector<double> & values, double value)
{
  for (const double existing : values) {
    if (std::abs(existing - value) <= 1e-8) {
      return;
    }
  }
  values.push_back(value);
}

void sort_and_dedupe(std::vector<double> & values)
{
  std::sort(values.begin(), values.end());
  values.erase(
    std::unique(
      values.begin(), values.end(),
      [](const double a, const double b) {return std::abs(a - b) <= 1e-8;}),
    values.end());
}

double point_segment_distance(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double len_sq = abx * abx + aby * aby;
  if (len_sq < kEpsilon) {
    return std::hypot(px - ax, py - ay);
  }
  double t = ((px - ax) * abx + (py - ay) * aby) / len_sq;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(px - (ax + t * abx), py - (ay + t * aby));
}

void collect_line_intersections(
  const std::vector<Point2D> & vertices,
  const ScanDirection direction,
  const double coord,
  std::vector<double> & crossings)
{
  const std::size_t n = vertices.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto & a = vertices[i];
    const auto & b = vertices[(i + 1U) % n];
    if (direction == ScanDirection::kX) {
      const double min_x = std::min(a.x, b.x);
      const double max_x = std::max(a.x, b.x);
      if (coord < min_x - kEpsilon || coord > max_x + kEpsilon) {
        continue;
      }
      if (std::abs(b.x - a.x) < kEpsilon) {
        if (std::abs(coord - a.x) <= kEpsilon) {
          append_unique_sorted(crossings, a.y);
          append_unique_sorted(crossings, b.y);
        }
        continue;
      }
      const double t = (coord - a.x) / (b.x - a.x);
      if (t < -kEpsilon || t > 1.0 + kEpsilon) {
        continue;
      }
      append_unique_sorted(crossings, a.y + t * (b.y - a.y));
    } else {
      const double min_y = std::min(a.y, b.y);
      const double max_y = std::max(a.y, b.y);
      if (coord < min_y - kEpsilon || coord > max_y + kEpsilon) {
        continue;
      }
      if (std::abs(b.y - a.y) < kEpsilon) {
        if (std::abs(coord - a.y) <= kEpsilon) {
          append_unique_sorted(crossings, a.x);
          append_unique_sorted(crossings, b.x);
        }
        continue;
      }
      const double t = (coord - a.y) / (b.y - a.y);
      if (t < -kEpsilon || t > 1.0 + kEpsilon) {
        continue;
      }
      append_unique_sorted(crossings, a.x + t * (b.x - a.x));
    }
  }
  sort_and_dedupe(crossings);
}
}  // namespace

ScanArea bounding_box(const std::vector<Point2D> & vertices)
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

double polygon_area(const std::vector<Point2D> & vertices)
{
  double area = 0.0;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const auto & a = vertices[i];
    const auto & b = vertices[(i + 1U) % vertices.size()];
    area += a.x * b.y - b.x * a.y;
  }
  return 0.5 * area;
}

void validate_simple_polygon(const std::vector<Point2D> & vertices)
{
  if (vertices.size() < 3U) {
    throw PolygonGeometryError("polygon requires at least 3 vertices.");
  }
  for (const auto & v : vertices) {
    if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
      throw PolygonGeometryError("polygon vertex coordinates must be finite.");
    }
  }
  const double area = polygon_area(vertices);
  if (std::abs(area) < kEpsilon) {
    throw PolygonGeometryError("polygon area is zero.");
  }
  if (area < 0.0) {
    throw PolygonGeometryError("polygon vertices must be counter-clockwise.");
  }
}

bool point_in_polygon(const Point2D & point, const std::vector<Point2D> & vertices)
{
  bool inside = false;
  for (std::size_t i = 0, j = vertices.size() - 1U; i < vertices.size(); j = i++) {
    const auto & a = vertices[i];
    const auto & b = vertices[j];
    const bool intersects = ((a.y > point.y) != (b.y > point.y)) &&
      (point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y + kEpsilon) + a.x);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

double distance_to_polygon_boundary(
  const double x, const double y, const std::vector<Point2D> & vertices)
{
  if (vertices.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }
  double min_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const auto & a = vertices[i];
    const auto & b = vertices[(i + 1U) % vertices.size()];
    min_dist = std::min(min_dist, point_segment_distance(x, y, a.x, a.y, b.x, b.y));
  }
  return min_dist;
}

std::vector<Chord> chords_on_scan_line(
  const std::vector<Point2D> & vertices,
  const ScanDirection direction,
  const double coord)
{
  std::vector<double> crossings;
  collect_line_intersections(vertices, direction, coord, crossings);
  std::vector<Chord> chords;
  if (crossings.size() < 2U) {
    return chords;
  }
  for (std::size_t i = 0; i + 1U < crossings.size(); i += 2U) {
    const double c0 = crossings[i];
    const double c1 = crossings[i + 1U];
    if (std::abs(c1 - c0) < kEpsilon) {
      continue;
    }
    Chord chord;
    if (direction == ScanDirection::kX) {
      chord.a = Point2D{coord, c0};
      chord.b = Point2D{coord, c1};
    } else {
      chord.a = Point2D{c0, coord};
      chord.b = Point2D{c1, coord};
    }
    const Point2D mid{(chord.a.x + chord.b.x) * 0.5, (chord.a.y + chord.b.y) * 0.5};
    // Ray-casting treats some boundary midpoints as outside; inset lanes may lie on edges.
    const bool inside = point_in_polygon(mid, vertices) ||
      distance_to_polygon_boundary(mid.x, mid.y, vertices) <= 1e-6;
    if (!inside) {
      continue;
    }
    chords.push_back(chord);
  }
  return chords;
}

}  // namespace gpr_common

#ifndef GPR_COMMON__TYPES_HPP_
#define GPR_COMMON__TYPES_HPP_

#include <cmath>
#include <cstdint>
#include <vector>

namespace gpr_common
{

/// @brief Planar pose (x, y, heading) used throughout the coverage stack.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  /// @brief Euclidean distance to another pose (ignores heading).
  [[nodiscard]] double distance_to(const Pose2D & other) const noexcept
  {
    return std::hypot(other.x - x, other.y - y);
  }
};

/// @brief Ordered sequence of poses (a path or segment centerline).
struct Polyline
{
  std::vector<Pose2D> points;

  [[nodiscard]] bool empty() const noexcept {return points.empty();}
  [[nodiscard]] std::size_t size() const noexcept {return points.size();}

  [[nodiscard]] double length() const noexcept
  {
    double total = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
      total += points[i - 1].distance_to(points[i]);
    }
    return total;
  }

  [[nodiscard]] Pose2D front_pose() const {return points.front();}
  [[nodiscard]] Pose2D back_pose() const {return points.back();}
};

/// @brief Axis-aligned rectangular region to be covered.
struct ScanArea
{
  double x_min{0.0};
  double x_max{0.0};
  double y_min{0.0};
  double y_max{0.0};
};

/// @brief Axis along which boustrophedon lanes run.
enum class ScanDirection
{
  kX,
  kY,
};

/// @brief Direction a coverage segment is traversed (start->end or end->start).
enum class DriveDirection
{
  Forward,
  Reverse,
};

/// @brief High-level state of the coverage mission, published on /coverage_status.
enum class MissionState
{
  Idle,
  WaitingForMap,
  Planning,
  Executing,
  Recovering,
  Complete,
  Failed,
};

}  // namespace gpr_common

#endif  // GPR_COMMON__TYPES_HPP_

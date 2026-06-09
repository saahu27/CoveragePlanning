#ifndef GPR_CONTROL__ROS_CONVERSIONS_HPP_
#define GPR_CONTROL__ROS_CONVERSIONS_HPP_

#include "gpr_common/types.hpp"
#include "nav_msgs/msg/path.hpp"

namespace gpr_control
{

/// @brief Convert an internal Polyline to a ROS nav_msgs/Path.
[[nodiscard]] nav_msgs::msg::Path to_nav_path(
  const gpr_common::Polyline & polyline, const std::string & frame_id);

}  // namespace gpr_control

#endif  // GPR_CONTROL__ROS_CONVERSIONS_HPP_

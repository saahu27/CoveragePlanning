#ifndef GPR_PERCEPTION__ROS_CONVERSIONS_HPP_
#define GPR_PERCEPTION__ROS_CONVERSIONS_HPP_

#include <string>

#include "gpr_common/grid_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/time.hpp"

namespace gpr_perception
{

/// @brief Convert an internal GridMap to a ROS OccupancyGrid message.
[[nodiscard]] nav_msgs::msg::OccupancyGrid to_ros(
  const gpr_common::GridMap & map,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

}  // namespace gpr_perception

#endif  // GPR_PERCEPTION__ROS_CONVERSIONS_HPP_

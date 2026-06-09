#include "gpr_perception/ros_conversions.hpp"

namespace gpr_perception
{

nav_msgs::msg::OccupancyGrid to_ros(
  const gpr_common::GridMap & map,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.info.resolution = static_cast<float>(map.resolution);
  msg.info.width = static_cast<uint32_t>(map.width);
  msg.info.height = static_cast<uint32_t>(map.height);
  msg.info.origin.position.x = map.origin_x;
  msg.info.origin.position.y = map.origin_y;
  msg.info.origin.orientation.w = 1.0;
  msg.data = map.data;
  return msg;
}

}  // namespace gpr_perception

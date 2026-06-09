#include "gpr_control/ros_conversions.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace gpr_control
{

nav_msgs::msg::Path to_nav_path(
  const gpr_common::Polyline & polyline, const std::string & frame_id)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  for (const auto & p : polyline.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, p.yaw);
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace gpr_control

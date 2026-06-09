#ifndef GPR_CONTROL__CONTROL_BRIDGE_HPP_
#define GPR_CONTROL__CONTROL_BRIDGE_HPP_

#include <memory>
#include <string>

#include "gpr_control/i_path_tracker.hpp"
#include "gpr_control/nav2_follow_path_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace gpr_control
{

/// @brief ROS-facing path execution (Nav2 FollowPath).
class ControlBridge
{
public:
  ControlBridge(
    const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::string frame_id,
    Nav2TrackerConfig tracker_config,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);

  void follow(const gpr_common::Polyline & path, TrackCallback cb);
  void cancel();

private:
  std::unique_ptr<Nav2FollowPathTracker> tracker_;
};

}  // namespace gpr_control

#endif  // GPR_CONTROL__CONTROL_BRIDGE_HPP_

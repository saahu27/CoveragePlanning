#include "gpr_control/control_bridge.hpp"

#include <utility>

namespace gpr_control
{

ControlBridge::ControlBridge(
  const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::string frame_id,
  Nav2TrackerConfig tracker_config,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: tracker_(std::make_unique<Nav2FollowPathTracker>(
      node, std::move(tf_buffer), std::move(frame_id),
      std::move(tracker_config), std::move(callback_group)))
{}

void ControlBridge::follow(const gpr_common::Polyline & path, TrackCallback cb)
{
  tracker_->follow(path, std::move(cb));
}

void ControlBridge::cancel()
{
  tracker_->cancel();
}

}  // namespace gpr_control

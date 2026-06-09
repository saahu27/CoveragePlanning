#include "gpr_mission/mission_context.hpp"

#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace gpr_mission
{

std::optional<gpr_common::Pose2D> MissionContext::robot_pose() const
{
  if (!tf_buffer) {
    return std::nullopt;
  }
  try {
    const auto tf = tf_buffer->lookupTransform(
      frame_id, robot_base_frame, tf2::TimePointZero);
    gpr_common::Pose2D pose;
    pose.x = tf.transform.translation.x;
    pose.y = tf.transform.translation.y;
    pose.yaw = tf2::getYaw(tf.transform.rotation);
    return pose;
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

}  // namespace gpr_mission

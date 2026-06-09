#include "gpr_control/nav2_follow_path_tracker.hpp"

#include <chrono>
#include <cmath>

#include "rcl_action/rcl_action.h"

#include "gpr_control/ros_conversions.hpp"
#include "tf2/exceptions.h"

namespace gpr_control
{

Nav2FollowPathTracker::Nav2FollowPathTracker(
  const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::string frame_id,
  Nav2TrackerConfig config,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: node_(node),
  tf_buffer_(std::move(tf_buffer)),
  frame_id_(std::move(frame_id)),
  config_(std::move(config)),
  callback_group_(std::move(callback_group))
{
  client_ = rclcpp_action::create_client<FollowPath>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    config_.action_name,
    callback_group_,
    rcl_action_client_get_default_options());
}

void Nav2FollowPathTracker::cancel()
{
  if (active_goal_) {
    client_->async_cancel_goal(active_goal_);
  }
}

/// @brief Current robot pose from TF (map -> base), or nullopt if unavailable.
std::optional<geometry_msgs::msg::PoseStamped> Nav2FollowPathTracker::robot_pose() const
{
  try {
    const auto tf = tf_buffer_->lookupTransform(
      frame_id_, config_.robot_base_frame, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.stamp = tf.header.stamp;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.orientation = tf.transform.rotation;
    return pose;
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

/// @brief Trim a path to start near the robot and prepend the robot pose, so
///        FollowPath receives a clean, forward-only path from the current pose.
nav_msgs::msg::Path Nav2FollowPathTracker::prepare_path(
  const gpr_common::Polyline & polyline,
  const geometry_msgs::msg::PoseStamped & robot) const
{
  auto path = to_nav_path(polyline, frame_id_);
  if (path.poses.size() < 2U) {
    return nav_msgs::msg::Path{};
  }

  const auto dist_sq = [](const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b) {
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      return dx * dx + dy * dy;
    };

  std::size_t closest = 0U;
  double closest_sq = dist_sq(robot.pose.position, path.poses[0].pose.position);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const double d = dist_sq(robot.pose.position, path.poses[i].pose.position);
    if (d < closest_sq) {
      closest_sq = d;
      closest = i;
    }
  }

  std::size_t start = closest;
  const double prune_sq = config_.path_prune_distance * config_.path_prune_distance;
  if (dist_sq(robot.pose.position, path.poses[start].pose.position) <= prune_sq &&
    start + 1U < path.poses.size())
  {
    ++start;
  }

  nav_msgs::msg::Path trimmed;
  trimmed.header = path.header;
  if (start >= path.poses.size()) {
    return trimmed;
  }
  trimmed.poses.assign(path.poses.begin() + static_cast<std::ptrdiff_t>(start),
    path.poses.end());

  nav_msgs::msg::Path prepared;
  prepared.header = trimmed.header;
  geometry_msgs::msg::PoseStamped robot_pose = robot;
  robot_pose.header.frame_id = frame_id_;
  prepared.poses.push_back(robot_pose);
  for (const auto & pose : trimmed.poses) {
    if (dist_sq(robot_pose.pose.position, pose.pose.position) > 0.01 * 0.01) {
      prepared.poses.push_back(pose);
    }
  }
  if (prepared.poses.size() < 2U) {
    prepared.poses.clear();
  }
  return prepared;
}

namespace
{
double path_length_m(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2U) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1U].pose.position;
    const auto & b = path.poses[i].pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}
}  // namespace

void Nav2FollowPathTracker::follow(
  const gpr_common::Polyline & path, TrackCallback on_done)
{
  callback_ = std::move(on_done);
  const auto robot = robot_pose();
  if (!robot) {
    RCLCPP_WARN(node_->get_logger(), "follow: no robot pose (TF), aborting job");
    if (callback_) {
      callback_(TrackResult::Aborted);
    }
    return;
  }

  const auto nav_path = prepare_path(path, *robot);
  if (nav_path.poses.size() < 2U) {
    RCLCPP_WARN(node_->get_logger(), "follow: empty path after prepare, aborting");
    if (callback_) {
      callback_(TrackResult::Aborted);
    }
    return;
  }

  const double follow_length = path_length_m(nav_path);
  if (config_.allow_instant_success &&
    follow_length < config_.min_follow_distance && nav_path.poses.size() <= 3U)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "follow: transit path too short (%.2f m), marking done",
      follow_length);
    if (callback_) {
      callback_(TrackResult::Succeeded);
    }
    return;
  }

  const auto wait_timeout = std::chrono::duration<double>(
    config_.action_server_wait_timeout_sec);
  if (!client_->wait_for_action_server(
      std::chrono::duration_cast<std::chrono::nanoseconds>(wait_timeout)))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "follow: FollowPath action server unavailable, aborting");
    if (callback_) {
      callback_(TrackResult::Aborted);
    }
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(), "follow: sending FollowPath goal with %zu poses",
    nav_path.poses.size());

  FollowPath::Goal goal;
  goal.path = nav_path;
  goal.controller_id = config_.controller_id;
  goal.goal_checker_id = config_.goal_checker_id;

  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.goal_response_callback = [this](const GoalHandle::SharedPtr & gh) {
      active_goal_ = gh;
    };
  options.result_callback = [this](const GoalHandle::WrappedResult & result) {
      handle_result(result);
    };

  client_->async_send_goal(goal, options);
}

/// @brief Map the action result code to a TrackResult and fire the callback.
void Nav2FollowPathTracker::handle_result(const GoalHandle::WrappedResult & result)
{
  active_goal_.reset();
  if (!callback_) {
    return;
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      callback_(TrackResult::Succeeded);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      callback_(TrackResult::Canceled);
      break;
    default:
      callback_(TrackResult::Aborted);
      break;
  }
  callback_ = nullptr;
}

}  // namespace gpr_control

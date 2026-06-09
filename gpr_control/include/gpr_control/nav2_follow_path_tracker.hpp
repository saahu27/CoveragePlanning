#ifndef GPR_CONTROL__NAV2_FOLLOW_PATH_TRACKER_HPP_
#define GPR_CONTROL__NAV2_FOLLOW_PATH_TRACKER_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gpr_control/i_path_tracker.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"

namespace gpr_control
{

/// @brief All tunables for the Nav2 FollowPath tracker (from ROS parameters).
struct Nav2TrackerConfig
{
  std::string action_name{"follow_path"};          ///< FollowPath action name.
  std::string controller_id{"FollowPath"};         ///< Nav2 controller plugin id.
  std::string goal_checker_id{"general_goal_checker"};  ///< Nav2 goal checker id.
  std::string robot_base_frame{"base_link"};       ///< Robot base frame for TF.
  double path_prune_distance{0.15};                ///< Drop waypoints within this of the robot (m).
  double min_follow_distance{0.25};  ///< Paths shorter than this succeed without Nav2 (m).
  bool allow_instant_success{false}; ///< Auto-succeed short paths (transit only).
  double action_server_wait_timeout_sec{2.0};      ///< Max wait for the action server (s).
};

/// @brief IPathTracker implemented via the Nav2 FollowPath action.
class Nav2FollowPathTracker : public IPathTracker
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  Nav2FollowPathTracker(
    const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::string frame_id,
    Nav2TrackerConfig config,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);

  void follow(const gpr_common::Polyline & path, TrackCallback on_done) override;
  void cancel() override;

private:
  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> robot_pose() const;
  [[nodiscard]] nav_msgs::msg::Path prepare_path(
    const gpr_common::Polyline & path,
    const geometry_msgs::msg::PoseStamped & robot) const;
  void handle_result(const GoalHandle::WrappedResult & result);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string frame_id_;
  Nav2TrackerConfig config_;
  rclcpp_action::Client<FollowPath>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  GoalHandle::SharedPtr active_goal_;
  TrackCallback callback_;
};

}  // namespace gpr_control

#endif  // GPR_CONTROL__NAV2_FOLLOW_PATH_TRACKER_HPP_

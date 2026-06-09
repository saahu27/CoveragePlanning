#include "gpr_perception/perception_bridge.hpp"

#include <cmath>
#include <functional>

#include "gpr_perception/ros_conversions.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace gpr_perception
{

PerceptionBridge::PerceptionBridge(
  const rclcpp::Node::SharedPtr & node,
  OccupancyGridMapper mapper,
  std::string frame_id,
  std::string scan_topic,
  std::string map_topic,
  std::string nav_map_topic,
  const double publish_rate_hz,
  const double scan_tf_timeout_sec,
  const int tf_warn_throttle_ms,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: node_(node),
  pipeline_(std::move(mapper)),
  frame_id_(std::move(frame_id)),
  scan_topic_(std::move(scan_topic)),
  map_topic_(std::move(map_topic)),
  nav_map_topic_(std::move(nav_map_topic)),
  scan_tf_timeout_sec_(scan_tf_timeout_sec),
  tf_warn_throttle_ms_(tf_warn_throttle_ms),
  callback_group_(std::move(callback_group))
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SubscriptionOptions sub_options;
  if (callback_group_) {
    sub_options.callback_group = callback_group_;
  }
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PerceptionBridge::scan_callback, this, std::placeholders::_1),
    sub_options);

  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, map_qos);
  nav_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(nav_map_topic_, map_qos);

  if (publish_rate_hz > 0.0) {
    const auto period_ms = static_cast<int>(1000.0 / publish_rate_hz);
    publish_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PerceptionBridge::publish_timer_callback, this),
      callback_group_);
  }
}

gpr_common::GridMapConstPtr PerceptionBridge::latest_grid_ptr() const
{
  return pipeline_.latest_grid_ptr();
}

gpr_common::GridMap PerceptionBridge::latest_grid() const
{
  return pipeline_.latest_grid();
}

gpr_common::GridMap PerceptionBridge::latest_inflated_grid() const
{
  return pipeline_.latest_inflated_grid();
}

bool PerceptionBridge::has_grid() const noexcept
{
  return pipeline_.has_grid();
}

void PerceptionBridge::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  geometry_msgs::msg::TransformStamped tf_sensor_in_map;
  try {
    tf_sensor_in_map = tf_buffer_->lookupTransform(
      frame_id_, scan->header.frame_id, scan->header.stamp,
      rclcpp::Duration::from_seconds(scan_tf_timeout_sec_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), tf_warn_throttle_ms_,
      "TF unavailable: %s", ex.what());
    return;
  }

  const double sensor_x = tf_sensor_in_map.transform.translation.x;
  const double sensor_y = tf_sensor_in_map.transform.translation.y;
  const double sensor_yaw = tf2::getYaw(tf_sensor_in_map.transform.rotation);

  for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
    const float range = scan->ranges[i];
    if (!std::isfinite(range) || range < scan->range_min || range >= scan->range_max) {
      continue;
    }
    const double beam_angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    const double hit_local_x = range * std::cos(beam_angle);
    const double hit_local_y = range * std::sin(beam_angle);
    const double cos_yaw = std::cos(sensor_yaw);
    const double sin_yaw = std::sin(sensor_yaw);
    const double hit_x = sensor_x + cos_yaw * hit_local_x - sin_yaw * hit_local_y;
    const double hit_y = sensor_y + sin_yaw * hit_local_x + cos_yaw * hit_local_y;
    pipeline_.integrate_ray(sensor_x, sensor_y, hit_x, hit_y, true);
  }
}

void PerceptionBridge::publish_timer_callback()
{
  pipeline_.refresh_grids();
  const auto planning_grid = pipeline_.latest_grid_ptr();
  const auto display_grid = pipeline_.latest_inflated_grid_ptr();
  const auto stamp = node_->now();
  if (display_grid && !display_grid->empty()) {
    map_pub_->publish(to_ros(*display_grid, frame_id_, stamp));
  }
  if (planning_grid && !planning_grid->empty()) {
    nav_map_pub_->publish(to_ros(*planning_grid, frame_id_, stamp));
  }
}

}  // namespace gpr_perception

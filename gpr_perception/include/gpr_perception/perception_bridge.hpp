#ifndef GPR_PERCEPTION__PERCEPTION_BRIDGE_HPP_
#define GPR_PERCEPTION__PERCEPTION_BRIDGE_HPP_

#include <memory>
#include <string>

#include "gpr_common/grid_map.hpp"
#include "gpr_perception/occupancy_grid_mapper.hpp"
#include "gpr_perception/perception_pipeline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace gpr_perception
{

/// @brief ROS glue: feeds LaserScan into the pipeline and publishes the grid.
class PerceptionBridge
{
public:
  PerceptionBridge(
    const rclcpp::Node::SharedPtr & node,
    OccupancyGridMapper mapper,
    std::string frame_id,
    std::string scan_topic,
    std::string map_topic,
    std::string nav_map_topic,
    double publish_rate_hz,
    double scan_tf_timeout_sec = 0.2,
    int tf_warn_throttle_ms = 3000,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);

  [[nodiscard]] PerceptionPipeline & pipeline() noexcept {return pipeline_;}
  [[nodiscard]] OccupancyGridMapper & mapper() noexcept {return pipeline_.mapper();}
  [[nodiscard]] std::uint64_t grid_seq() const noexcept {return pipeline_.grid_seq();}
  [[nodiscard]] gpr_common::GridMapConstPtr latest_grid_ptr() const;
  [[nodiscard]] gpr_common::GridMap latest_grid() const;
  [[nodiscard]] gpr_common::GridMap latest_inflated_grid() const;
  [[nodiscard]] bool has_grid() const noexcept;

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publish_timer_callback();

  rclcpp::Node::SharedPtr node_;
  PerceptionPipeline pipeline_;
  std::string frame_id_;
  std::string scan_topic_;
  std::string map_topic_;
  std::string nav_map_topic_;
  double scan_tf_timeout_sec_;
  int tf_warn_throttle_ms_{3000};

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr nav_map_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace gpr_perception

#endif  // GPR_PERCEPTION__PERCEPTION_BRIDGE_HPP_

#ifndef GPR_MISSION__COVERAGE_STACK_HPP_
#define GPR_MISSION__COVERAGE_STACK_HPP_

#include <memory>

#include "gpr_mission/mission_context.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gpr_mission
{

/// @brief Composition root: wires perception, planning, control, and the planning worker.
class CoverageStack
{
public:
  struct InitOptions
  {
    rclcpp::CallbackGroup::SharedPtr sensor_callback_group;
    rclcpp::CallbackGroup::SharedPtr control_callback_group;
  };

  explicit CoverageStack(const rclcpp::Node::SharedPtr & node);

  void initialize(const InitOptions & options);
  void shutdown();

  [[nodiscard]] MissionContext & context() noexcept {return context_;}
  [[nodiscard]] const MissionContext & context() const noexcept {return context_;}

private:
  rclcpp::Node::SharedPtr node_;
  MissionContext context_;
};

}  // namespace gpr_mission

#endif  // GPR_MISSION__COVERAGE_STACK_HPP_

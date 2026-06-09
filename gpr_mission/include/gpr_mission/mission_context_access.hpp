#ifndef GPR_MISSION__MISSION_CONTEXT_ACCESS_HPP_
#define GPR_MISSION__MISSION_CONTEXT_ACCESS_HPP_

#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "gpr_mission/mission_context.hpp"
#include "gpr_mission/mission_context_provider.hpp"

namespace gpr_mission
{

/// @brief Shared blackboard lookup for all behavior-tree nodes.
[[nodiscard]] inline MissionContext * get_mission_context(const BT::NodeConfig & config)
{
  const auto provider =
    config.blackboard->get<std::shared_ptr<MissionContextProvider>>("mission_provider");
  return provider ? provider->context() : nullptr;
}

}  // namespace gpr_mission

#endif  // GPR_MISSION__MISSION_CONTEXT_ACCESS_HPP_

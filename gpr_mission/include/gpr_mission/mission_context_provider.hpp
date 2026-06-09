#ifndef GPR_MISSION__MISSION_CONTEXT_PROVIDER_HPP_
#define GPR_MISSION__MISSION_CONTEXT_PROVIDER_HPP_

namespace gpr_mission
{

struct MissionContext;

/// @brief Indirection for BT nodes: resolves MissionContext via node lifetime.
class MissionContextProvider
{
public:
  virtual ~MissionContextProvider() = default;

  [[nodiscard]] virtual MissionContext * context() noexcept = 0;
  [[nodiscard]] virtual const MissionContext * context() const noexcept = 0;
};

/// @brief Non-owning holder; context must outlive this object (e.g. node member).
class MissionContextHolder : public MissionContextProvider
{
public:
  explicit MissionContextHolder(MissionContext * context) noexcept
  : context_(context)
  {}

  [[nodiscard]] MissionContext * context() noexcept override {return context_;}
  [[nodiscard]] const MissionContext * context() const noexcept override {return context_;}

private:
  MissionContext * context_;
};

}  // namespace gpr_mission

#endif  // GPR_MISSION__MISSION_CONTEXT_PROVIDER_HPP_
